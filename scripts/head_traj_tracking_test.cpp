/** ROBAN v17 头部 yaw/pitch：当前位姿 -> 配置目标 -> 保持测量 -> 返回。 */
#include "traj_tracking_common.hpp"

int main(int argc, char** argv) {
  return roban_tracking::run(roban_tracking::Part::Head, argc, argv);
}
