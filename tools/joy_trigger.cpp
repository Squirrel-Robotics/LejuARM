// Joystick DDS trigger tool - replicates leju_joystick keyboard mapping
//
// Button mode:  joy_trigger <button>
//   start  (O/F) - start controller
//   guide  (G)   - trigger dance
//   back   (B)   - stop controller
//   south  (C)   - A / Cross
//   east   (T)   - B / Circle
//   north  (R)   - Y / Triangle
//   left_shoulder (P) - LB / L1
//
// Axis mode:    joy_trigger walk [lx] [ly] [rx] [ry] [duration_s]
//   Continuously publishes axis values at 60Hz
//   lx/ly: left stick (-1.0~1.0), controls linear velocity
//   rx/ry: right stick (-1.0~1.0), controls angular velocity
//   duration_s: how long to publish (default 3.0s)
//
// Examples:
//   joy_trigger walk 0 -0.5 0 0 5    # forward at 50% speed for 5s
//   joy_trigger walk 0 0.3 0 0        # backward at 30% for 3s
//   joy_trigger walk 0.4 0 0 0        # strafe right for 3s
//   joy_trigger walk 0 -0.5 0.3 0     # forward + turn right

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-utils/time_utils.hpp"

static volatile bool g_running = true;
static void sigHandler(int) { g_running = false; }

void printUsage(const char* prog) {
  std::cerr << "Usage:" << std::endl;
  std::cerr << "  " << prog << " <button>                          # send button press" << std::endl;
  std::cerr << "  " << prog << " walk [lx] [ly] [rx] [ry] [sec]   # continuous axis" << std::endl;
  std::cerr << std::endl;
  std::cerr << "Buttons: start, guide, back, south, east, north, left_shoulder" << std::endl;
  std::cerr << std::endl;
  std::cerr << "Walk examples:" << std::endl;
  std::cerr << "  " << prog << " walk 0 -0.5 0 0 5   # forward 50% for 5s" << std::endl;
  std::cerr << "  " << prog << " walk 0 -0.5 0.3 0   # forward + turn right 3s" << std::endl;
}

int doButton(leju::RobotBaseAPI& robot, const std::string& action) {
  leju::JoyData joy;
  joy.timestamp = leju::common::GetUnixTimestampS();

  if (action == "start")            joy.buttons.start = 1;
  else if (action == "guide")       joy.buttons.guide = 1;
  else if (action == "back")        joy.buttons.back = 1;
  else if (action == "south")       joy.buttons.south = 1;
  else if (action == "east")        joy.buttons.east = 1;
  else if (action == "north")       joy.buttons.north = 1;
  else if (action == "left_shoulder") joy.buttons.left_shoulder = 1;
  else {
    std::cerr << "Unknown button: " << action << std::endl;
    return 1;
  }

  std::printf("Sending [%s] button press...\n", action.c_str());
  robot.publishJoyData(joy);

  // button-up for edge detection
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  leju::JoyData joy_up;
  joy_up.timestamp = leju::common::GetUnixTimestampS();
  robot.publishJoyData(joy_up);

  std::printf("Done.\n");
  return 0;
}

int doWalk(leju::RobotBaseAPI& robot, float lx, float ly, float rx, float ry, double duration_s) {
  std::printf("Publishing axis: lx=%.2f ly=%.2f rx=%.2f ry=%.2f for %.1fs (Ctrl+C to stop)\n",
              lx, ly, rx, ry, duration_s);

  signal(SIGINT, sigHandler);
  signal(SIGTERM, sigHandler);

  auto t0 = std::chrono::steady_clock::now();
  auto dt = std::chrono::microseconds(16667); // ~60Hz
  int count = 0;

  while (g_running) {
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - t0).count();
    if (elapsed >= duration_s) break;

    leju::JoyData joy;
    joy.timestamp = leju::common::GetUnixTimestampS();
    joy.axes.left_x = lx;
    joy.axes.left_y = ly;
    joy.axes.right_x = rx;
    joy.axes.right_y = ry;
    robot.publishJoyData(joy);

    count++;
    auto next = t0 + dt * count;
    std::this_thread::sleep_until(next);
  }

  // Send zero to stop
  leju::JoyData stop;
  stop.timestamp = leju::common::GetUnixTimestampS();
  robot.publishJoyData(stop);
  std::printf("Stopped. Sent %d frames.\n", count);
  return 0;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    printUsage(argv[0]);
    return 1;
  }

  leju::RobotVersion version = leju::RobotVersion::from_env();
  if (!leju::GlobalRobot::init_env(version)) {
    std::cerr << "Failed to init robot env" << std::endl;
    return 1;
  }
  auto& robot = leju::GlobalRobot::getInstance();

  // Wait for DDS discovery
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  std::string action = argv[1];

  if (action == "walk") {
    float lx = (argc > 2) ? std::atof(argv[2]) : 0.f;
    float ly = (argc > 3) ? std::atof(argv[3]) : 0.f;
    float rx = (argc > 4) ? std::atof(argv[4]) : 0.f;
    float ry = (argc > 5) ? std::atof(argv[5]) : 0.f;
    double dur = (argc > 6) ? std::atof(argv[6]) : 3.0;
    return doWalk(robot, lx, ly, rx, ry, dur);
  } else {
    return doButton(robot, action);
  }
}
