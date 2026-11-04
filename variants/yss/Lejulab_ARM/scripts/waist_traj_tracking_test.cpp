/** ROBAN v17 腰部 yaw：当前位姿 -> 配置目标 -> 保持测量 -> 返回。 */
#include "waist_only_tracking.hpp"

int main(int argc, char** argv) {
  return roban_tracking::runWaistOnly(argc, argv);
}
