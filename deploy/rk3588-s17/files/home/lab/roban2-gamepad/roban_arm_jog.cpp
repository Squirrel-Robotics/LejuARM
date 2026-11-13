#include "jog_core.hpp"
#include <lejusdk-lowlevel/leju_sdk.h>
#include <lejusdk-vr/lejusdk_vr.h>
#include <cstdio>
#include <sstream>
#include <atomic>
#include <chrono>
#include <csignal>
#include <deque>
#include <iostream>
#include <mutex>
#include <thread>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>

using Clock=std::chrono::steady_clock;
using namespace std::chrono_literals;
static volatile std::sig_atomic_t quit=0;
static void signal_stop(int) { quit=1; }
static double seconds(Clock::duration d) { return std::chrono::duration<double>(d).count(); }
static double wall_time() { return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); }
static void log(const std::string& s) { std::cout<<s<<std::endl; }

struct Status {
  bool ready=false;
  std::string controller;
  bool transitioning=true;
  int mode=-1;
  Clock::time_point at{};
};
struct Shared {
  std::mutex mutex;
  leju::RobotState joints;
  leju::ImuData imu;
  Clock::time_point jt{},it{},yt{};
  Status status;
  jog::Gate gate;
  int direction=0;
  unsigned epoch=0;
  bool emergency=false, owned=false, foreign=false;
  std::deque<jog::Pose> expected;
};
struct Snapshot {
  leju::RobotState joints;
  leju::ImuData imu;
  Clock::time_point jt,it,yt;
  Status status;
  int direction;
  unsigned epoch;
  bool emergency,foreign;
};
static Snapshot snapshot(Shared& s) {
  std::lock_guard<std::mutex> lock(s.mutex);
  return {s.joints,s.imu,s.jt,s.it,s.yt,s.status,s.direction,s.epoch,s.emergency,s.foreign};
}
static bool runtime_ok(const Snapshot& s) {
  return seconds(Clock::now()-s.status.at)<jog::status_timeout && s.status.ready &&
         s.status.controller=="amp" && !s.status.transitioning;
}
static bool sensors_ok(const Snapshot& s, jog::Pose& p) {
  auto now=Clock::now();
  if(seconds(now-s.jt)>jog::feedback_timeout || seconds(now-s.it)>jog::feedback_timeout ||
     s.joints.q.size()!=23 || s.joints.v.size()!=23 || s.joints.tau.size()!=23) return false;
  for(unsigned i=0;i<23;++i) {
    if(!std::isfinite(s.joints.q[i]) || !std::isfinite(s.joints.v[i]) ||
       !std::isfinite(s.joints.tau[i]) || std::abs(s.joints.v[i])>(i>=13 && i<21 ? .6 : .4)) return false;
  }
  for(unsigned i=0;i<8;++i) {
    p[i]=s.joints.q[i+13];
    if(std::abs(s.joints.tau[i+13])>8.) return false;
  }
  if(!jog::within_envelope(p)) return false;
  double norm=0;
  for(double v:s.imu.quat) { if(!std::isfinite(v)) return false; norm+=v*v; }
  if(norm<.9 || norm>1.1) return false;
  double upright=1-2*(s.imu.quat[1]*s.imu.quat[1]+s.imu.quat[2]*s.imu.quat[2])/norm;
  if(upright<std::cos(15*jog::pi/180)) return false;
  for(double v:s.imu.gyro) if(!std::isfinite(v) || std::abs(v)>.3) return false;
  return true;
}
static bool held(const Snapshot& s, int direction, unsigned epoch) {
  return s.direction==direction && s.epoch==epoch && !s.emergency && !s.foreign &&
         seconds(Clock::now()-s.yt)<jog::input_timeout;
}

// Use the existing validated Python wire definitions for read-only queries.
// The service disables shared memory: this machine's SHM RPC path did not
// return Python replies and crashed a native query during observation.
static void status_worker(Shared& shared, std::atomic<bool>& stop) {
  FILE* stream=popen("/usr/bin/python3 /home/lab/roban2-gamepad/status_monitor.py", "r");
  if(!stream) {log("STATUS ERROR: cannot start read-only monitor");return;}
  char line[256];
  while(!stop && fgets(line,sizeof(line),stream)) {
    Status st;double timestamp=0;int ready=-1,amp=-1,transition=-1,mode=-1;std::string extra;
    std::istringstream values(line);
    if((values>>timestamp>>ready>>amp>>transition>>mode) && !(values>>extra) &&
       std::isfinite(timestamp) && std::abs(wall_time()-timestamp)<.35 &&
       (ready==0||ready==1) && (amp==0||amp==1) && (transition==0||transition==1) && mode>=0 && mode<=2) {
      st.ready=ready;st.controller=amp ? "amp" : "other";st.transitioning=transition;
      st.mode=mode;st.at=Clock::now()-std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(std::max(0.,wall_time()-timestamp)));
    }
    {std::lock_guard<std::mutex> lock(shared.mutex);shared.status=st;}
  }
  pclose(stream);
}

int main(int argc,char** argv) {
  bool execute=argc==2 && std::string(argv[1])=="--execute";
  bool observe=argc==2 && std::string(argv[1])=="--observe";
  if(!execute && !observe) { std::cerr<<"Usage: roban_arm_jog --observe|--execute\n";return 2; }
  if(!std::getenv("ROBOT_VERSION") || std::string(std::getenv("ROBOT_VERSION"))!="17") return 2;
  const char* dds_uri=std::getenv("CYCLONEDDS_URI");
  bool isolated=dds_uri && std::string(dds_uri)=="file:///home/lab/roban2-gamepad/test-dds.xml";
  int lockfd=open(isolated ? "/tmp/roban2-arm-jog-isolated.lock" : "/tmp/roban2-arm-jog.lock",O_CREAT|O_RDWR|O_NOFOLLOW,0600);
  if(lockfd<0 || flock(lockfd,LOCK_EX|LOCK_NB)!=0) { log("Another arm jog instance is running");return 2; }
  std::signal(SIGINT,signal_stop);std::signal(SIGTERM,signal_stop);
  auto shared=std::make_shared<Shared>();
  std::atomic<bool> stop_status{false};
  std::thread status_thread;
  bool owning=false;
  bool confirmed=false;
  auto version=leju::RobotVersion::from_env();
  leju::vr::RobanVRAPI api(version);
  auto release=[&](const char* reason) {
    if(!owning) return;
    owning=false;
    auto s=snapshot(*shared);
    // Do not override a different owner or queue a mode change behind Quit.
    if(!s.foreign && !s.emergency && runtime_ok(s) && (s.status.mode==2 || !confirmed)) {
      if(!api.setArmMode(leju::vr::ControlMode::kKeepPose,150))
        log("HOLD request was not acknowledged; no further trajectory commands sent");
    }
    {std::lock_guard<std::mutex> lock(shared->mutex);shared->owned=false;shared->gate.reset();shared->direction=0;++shared->epoch;}
    log(std::string("STOP: ")+reason);
  };
  try {
    if(!leju::GlobalRobot::init_env(version) || !api.initialize()) throw std::runtime_error("SDK initialization failed");
    auto& robot=leju::GlobalRobot::getInstance();
    robot.subscribeRobotState([shared](const leju::RobotStateConstPtr& p) {
      if(!std::isfinite(p->timestamp) || std::abs(wall_time()-p->timestamp)>.25) return;
      std::lock_guard<std::mutex> lock(shared->mutex);shared->joints=*p;shared->jt=Clock::now();
    });
    robot.subscribeImuData([shared](const leju::ImuDataConstPtr& p) {
      if(!std::isfinite(p->timestamp) || std::abs(wall_time()-p->timestamp)>.25) return;
      std::lock_guard<std::mutex> lock(shared->mutex);shared->imu=*p;shared->it=Clock::now();
    });
    robot.subscribeJoyData([shared](const leju::JoyDataConstPtr& p) {
      auto now=Clock::now(); const auto& b=p->buttons; const auto& a=p->axes;
      jog::Input in;in.a=b.south;in.y=b.north;in.close=b.left_stick;in.open=b.right_stick;
      in.other=b.east||b.west||b.back||b.guide||b.start||
        b.left_shoulder||b.right_shoulder||b.dpad_up||b.dpad_down||b.dpad_left||b.dpad_right||b.misc1||b.misc2;
      in.axes=!std::isfinite(a.left_x)||!std::isfinite(a.left_y)||!std::isfinite(a.right_x)||!std::isfinite(a.right_y)||
        !std::isfinite(a.left_trigger)||!std::isfinite(a.right_trigger)||
        std::abs(a.left_x)>.05||std::abs(a.left_y)>.05||std::abs(a.right_x)>.05||std::abs(a.right_y)>.05||
        std::abs(a.left_trigger)>.1||std::abs(a.right_trigger)>.1;
      in.fresh=std::isfinite(p->timestamp) && std::abs(wall_time()-p->timestamp)<.25;
      std::lock_guard<std::mutex> lock(shared->mutex);
      if(seconds(now-shared->yt)>jog::input_timeout) shared->gate.reset();
      int direction=shared->gate.update(in,seconds(now.time_since_epoch()));
      if(direction!=shared->direction) ++shared->epoch;
      shared->direction=direction; shared->yt=in.fresh ? now : Clock::time_point{};
      shared->emergency=b.east || (b.back&&b.start);
    });
    api.subscribeArmJointCmd([shared](const leju::vr::JointTrajectoryPoint& cmd) {
      std::lock_guard<std::mutex> lock(shared->mutex);
      if(!shared->owned) return;
      bool match=false;
      if(cmd.q.size()==8) for(const auto& expected:shared->expected) {
        bool same=true;
        for(unsigned i=0;i<8;++i) if(!std::isfinite(cmd.q[i]) || std::abs(cmd.q[i]-expected[i])>1e-7) same=false;
        if(same) { match=true;break; }
      }
      if(!match) { shared->foreign=true;++shared->epoch; }
    });
    status_thread=std::thread(status_worker,std::ref(*shared),std::ref(stop_status));
    auto publish=[&](const jog::Pose& pose) {
      // Record before write, because loopback delivery may be immediate.
      {std::lock_guard<std::mutex> lock(shared->mutex);shared->expected.push_back(pose);
       if(shared->expected.size()>64) shared->expected.pop_front();}
      leju::vr::JointTrajectoryPoint cmd;
      cmd.q.assign(pose.begin(),pose.end());cmd.v.assign(8,0);cmd.acc.assign(8,0);
      if(!api.publishArmJointCmd(cmd)) throw std::runtime_error("arm command publish failed");
    };
    log(observe ? "OBSERVE ONLY: no mode or motion commands" : "ARM JOG READY: hold Y=extend, A=retract, L3=wrists close, R3=wrists open; release=hold. Waiting for neutral input.");
    auto start=Clock::now(),last=start,began=start;
    int direction=0;unsigned epoch=0;
    jog::Pose command{};double speed=0;
    while(!quit) {
      auto now=Clock::now();double dt=seconds(now-last);last=now;
      auto s=snapshot(*shared);jog::Pose measured{};
      if(observe) {
        if(seconds(now-start)>5) {
          log("OBSERVE: joy_age="+std::to_string(seconds(now-s.yt))+" joints_age="+std::to_string(seconds(now-s.jt))+
              " imu_age="+std::to_string(seconds(now-s.it))+" controller="+s.status.controller+
              " arm_mode="+std::to_string(s.status.mode)+" runtime_ready="+std::to_string(runtime_ok(s))+
              " sensors_safe="+std::to_string(sensors_ok(s,measured))+" direction="+std::to_string(s.direction));
          break;
        }
      } else if(owning) {
        if(!held(s,direction,epoch)) release("button released / competing input / stale joystick");
        else if(!runtime_ok(s) || !sensors_ok(s,measured)) release("runtime or feedback guard");
        else if(!jog::tracking(measured,command,std::abs(direction)==2 ? .04 : .15)) release("tracking error / wrist contact resistance");
        else if(!confirmed) {
          if(s.status.mode==2) {confirmed=true;last=Clock::now();}
          else if(seconds(now-began)>.6) release("external mode not confirmed");
        } else if(s.status.mode!=2) release("arm mode changed");
        else if(dt>.05) release("control deadline missed");
        else {
          bool done=jog::advance(command,direction,dt,speed);
          // Check the latest callback again immediately before any write.
          if(!held(snapshot(*shared),direction,epoch)) release("release before publish");
          else { publish(command); if(done) release("travel limit reached"); }
        }
      } else if(s.direction && seconds(now-s.yt)<jog::input_timeout) {
        if(!runtime_ok(s) || (s.status.mode!=0 && s.status.mode!=1) || !sensors_ok(s,measured) ||
           (std::abs(s.direction)==2 && !jog::wrists_forward(measured))) {
          std::lock_guard<std::mutex> lock(shared->mutex);shared->gate.reset();shared->direction=0;++shared->epoch;
          log("IGNORED: requires stationary AMP, fresh feedback, bounded pose, Auto/KeepPose; wrist controls require both arms extended at least 60 degrees");
        } else {
          direction=s.direction;epoch=s.epoch;command=measured;speed=0;confirmed=false;began=now;
          {std::lock_guard<std::mutex> lock(shared->mutex);shared->owned=true;shared->foreign=false;shared->expected.clear();}
          owning=true;
          if(!held(snapshot(*shared),direction,epoch)) release("released before acquisition");
          else {
            publish(command); // preload the measured pose before External
            if(!held(snapshot(*shared),direction,epoch)) release("released during preload");
            else if(!api.setArmMode(leju::vr::ControlMode::kExternal,150)) release("external mode request failed");
            else { last=Clock::now();log(direction==1 ? "JOG: forward held" : direction==-1 ? "JOG: retract held" : direction==2 ? "JOG: wrists closing held" : "JOG: wrists opening held"); }
          }
        }
      }
      std::this_thread::sleep_for(10ms);
    }
    release("service stopped");
  } catch(const std::exception& e) { log(std::string("ERROR: ")+e.what());release("exception");quit=1; }
  stop_status=true;if(status_thread.joinable()) status_thread.join();
  api.shutdown();if(leju::GlobalRobot::is_initialized()) leju::GlobalRobot::getInstance().shutdown();
  close(lockfd);
  return quit ? 1 : 0;
}
