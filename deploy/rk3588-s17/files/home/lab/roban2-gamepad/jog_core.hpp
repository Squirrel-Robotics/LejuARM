#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace jog {
using Pose = std::array<double, 8>;
constexpr double pi = 3.14159265358979323846;
// v17 config_amp.yaml start_stand_pos, and the eight arm joints from
// the existing 前伸双手.tact, keyframe 100. No hand/head/waist commands.
inline const Pose retracted{0, .05, 0, -.2, 0, -.05, 0, -.2};
inline const Pose extended{-95*pi/180, 13*pi/180, -5*pi/180, -3*pi/180,
                          -95*pi/180, -13*pi/180, 5*pi/180, -3*pi/180};
constexpr double max_speed = .18; // radians/second per joint
constexpr double acceleration = .3;
constexpr double input_timeout = .15;
constexpr double feedback_timeout = .15;
constexpr double status_timeout = .35;

// URDF biped_s17: upper arm, elbow-to-hand mounting joint (bare wrist),
// shoulder lateral offset, and mirrored fixed shoulder roll.
constexpr double upper_arm=.2085, wrist_length=.085236, shoulder_y=.181, mount_roll=.17453;
constexpr double min_wrist_gap=.18;
constexpr double wrist_speed=.08;
constexpr double forward_pitch=-pi/3;

struct Input { bool a=false, y=false, close=false, open=false, other=false, axes=false, fresh=false; };
// A modifier invalidates the entire press, including modifier-first releases.
// Reconnect/startup and failures require a neutral sample before another press.
class Gate {
  bool armed_ = false;
  int candidate_ = 0;
  double since_ = 0;
public:
  void reset() { armed_=false; candidate_=0; }
  int update(const Input& in, double now) {
    int count=int(in.a)+int(in.y)+int(in.close)+int(in.open);
    if (!in.fresh || in.other || in.axes || count>1) {
      reset(); return 0;
    }
    if (!count) { armed_=true; candidate_=0; return 0; }
    if (!armed_) return 0;
    int direction=in.y ? 1 : in.a ? -1 : in.close ? 2 : -2;
    if (candidate_ && candidate_!=direction) { reset(); return 0; }
    if (!candidate_) { candidate_=direction; since_=now; }
    // Allow time to form existing multi-button combinations.
    return now-since_ >= .25 ? direction : 0;
  }
};

inline bool within_envelope(const Pose& p) {
  for (unsigned i=0;i<8;++i) {
    double lower=std::min(retracted[i],extended[i])-.15;
    double upper=std::max(retracted[i],extended[i])+.15;
    if(i==1) lower=-.52;
    if(i==5) upper=.52;
    if (!std::isfinite(p[i]) || p[i]<lower || p[i]>upper) return false;
  }
  return true;
}

inline bool tracking(const Pose& measured, const Pose& command, double tolerance=.15) {
  for (unsigned i=0;i<8;++i)
    if (!std::isfinite(measured[i]) || std::abs(measured[i]-command[i])>tolerance)
      return false;
  return true;
}

inline double wrist_y(const Pose& p, unsigned side) {
  unsigned i=side*4;
  double sign=side==0 ? 1. : -1.;
  double alpha=p[i+1]+sign*mount_roll;
  double a=upper_arm+wrist_length*std::cos(p[i+3]);
  double b=-wrist_length*std::sin(p[i+3])*std::sin(p[i+2]);
  return sign*shoulder_y+a*std::sin(alpha)+b*std::cos(alpha);
}
inline double wrist_gap(const Pose& p) { return wrist_y(p,0)-wrist_y(p,1); }
inline bool wrists_forward(const Pose& p) {
  return p[0]<=forward_pitch+1e-6 && p[4]<=forward_pitch+1e-6;
}
inline bool set_wrist_y(Pose& p,unsigned side,double y) {
  unsigned i=side*4;double sign=side==0 ? 1. : -1.;
  double a=upper_arm+wrist_length*std::cos(p[i+3]);
  double b=-wrist_length*std::sin(p[i+3])*std::sin(p[i+2]);
  double s=(y-sign*shoulder_y)/std::hypot(a,b);
  if(!std::isfinite(s) || std::abs(s)>1) return false;
  p[i+1]=std::asin(s)-std::atan2(b,a)-sign*mount_roll;
  return within_envelope(p);
}

// Advance all eight joints along one line, with a bounded speed ramp.
// The caller stops calling immediately on release. No queued trajectory exists.
inline bool advance(Pose& p, int direction, double dt, double& speed) {
  if ((std::abs(direction)!=1 && std::abs(direction)!=2) || !std::isfinite(dt) || dt<=0 || dt>.05)
    throw std::runtime_error("invalid direction or missed control deadline");
  bool wrist=std::abs(direction)==2;
  if(wrist && !wrists_forward(p)) {speed=0;return true;}
  Pose goal=p;
  double left_y=wrist_y(p,0),right_y=wrist_y(p,1);
  if(wrist) {
    goal[1]=direction==2 ? -.50 : extended[1];
    goal[5]=direction==2 ? .50 : extended[5];
  } else {
    const Pose& endpoint=direction==1 ? extended : retracted;
    for(unsigned i: {0u,3u,4u,7u}) goal[i]=endpoint[i];
    // Preserve both wrist lateral coordinates during Y/A movements.
    // Once wrists are inside shoulder width, do not fold them into the torso.
    if(wrist_gap(p)<2*shoulder_y) {
      goal[0]=std::min(goal[0],forward_pitch);
      goal[4]=std::min(goal[4],forward_pitch);
    }
  }
  double remaining=0;
  for (unsigned i=0;i<8;++i) remaining=std::max(remaining,std::abs(goal[i]-p[i]));
  if (remaining<1e-8) { speed=0; return true; }
  double limit=wrist ? wrist_speed : max_speed;
  speed=std::min({limit,speed+acceleration*dt,std::sqrt(2*acceleration*remaining)});
  double fraction=std::min(1.,speed*dt/remaining);
  Pose next=p;
  for (unsigned i=0;i<8;++i) next[i]+=(goal[i]-p[i])*fraction;
  if(!wrist && (!set_wrist_y(next,0,left_y) || !set_wrist_y(next,1,right_y))) {speed=0;return true;}
  if(!within_envelope(next) || wrist_gap(next)<min_wrist_gap ||
      wrist_y(next,0)<min_wrist_gap/2 || wrist_y(next,1)>-min_wrist_gap/2) {speed=0;return true;}
  for(unsigned i=0;i<8;++i)
    if(std::abs(next[i]-p[i])>limit*dt+1e-9) {speed=0;return true;}
  p=next;
  return fraction==1.;
}
}
