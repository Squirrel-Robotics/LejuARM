#include <lejusdk-vr/lejusdk_vr.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

// S17 head only. Keep the legacy C++ SDK in a separate process from Python DDS.
// Do not use the legacy SDK RPC Requester here (CycloneDDS 0.10.2 loan issue).
static std::unique_ptr<leju::vr::RobanVRAPI> api;

// Match scripts/traj_tracking_common.hpp::specFor(Part::Head).
static constexpr double kRadiansPerDegree = 3.14159265358979323846 / 180.;
static constexpr double kPositionEps = 1e-12;
static bool command_in_range(double yaw, double pitch, double vyaw, double vpitch) {
  return std::isfinite(yaw) && std::isfinite(pitch)
      && std::isfinite(vyaw) && std::isfinite(vpitch)
      && yaw >= -90. * kRadiansPerDegree - kPositionEps
      && yaw <= 90. * kRadiansPerDegree + kPositionEps
      && pitch >= -20. * kRadiansPerDegree - kPositionEps
      && pitch <= 45. * kRadiansPerDegree + kPositionEps
      && std::abs(vyaw) <= .18 && std::abs(vpitch) <= .18;
}

static int initialize() {
  if (api) return 0;
  try {
    api = std::make_unique<leju::vr::RobanVRAPI>(leju::RobotVersion{1, 7});
    return api->initialize() ? 1 : 0;
  } catch (...) { return 0; }
}

static int publish(double yaw, double pitch, double vyaw, double vpitch) {
  if (!api || !command_in_range(yaw, pitch, vyaw, vpitch)) return 0;
  try {
    leju::vr::JointTrajectoryPoint cmd;
    cmd.q = {yaw, pitch};
    cmd.v = {vyaw, vpitch};
    cmd.acc = {0., 0.};
    return api->publishHeadJointCmd(cmd) ? 1 : 0;
  } catch (...) { return 0; }
}

static void close_sdk() {
  if (api) {
    try { api->shutdown(); } catch (...) {}
    api.reset();
  }
}

int main() {
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream input(line);
    std::string command, extra;
    input >> command;
    int ok = 0;
    if (command == "INIT" && !(input >> extra)) {
      ok = initialize();
    } else if (command == "PUB" || command == "CHECK") {
      double yaw, pitch, vyaw, vpitch;
      if ((input >> yaw >> pitch >> vyaw >> vpitch) && !(input >> extra))
        // CHECK validates the exact publication guard without initializing SDK
        // or publishing any DDS/motor command; used by offline boundary tests.
        ok = command == "CHECK" ? command_in_range(yaw, pitch, vyaw, vpitch)
                                : publish(yaw, pitch, vyaw, vpitch);
    } else if (command == "CLOSE" && !(input >> extra)) {
      close_sdk();
      std::cout << "SDK_RESULT 1" << std::endl;
      return 0;
    }
    std::cout << "SDK_RESULT " << ok << std::endl;
  }
  close_sdk();
  return 0;
}
