#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET="${ROOT_DIR}/src/leju-controllers/leju-rl-controller/src/runtime/transport_fall_stand_scheduler.cpp"

if [[ ! -f "${TARGET}" ]]; then
  echo "错误：未找到目标文件：${TARGET}" >&2
  exit 1
fi

python3 - "${TARGET}" <<'PY'
from pathlib import Path
import shutil
import sys

path = Path(sys.argv[1])
text = path.read_text()
old_angle = """// 头部低头角度（正 pitch = 低头）
constexpr double kHeadDownPitchRad = 20.0 * M_PI / 180.0;
"""
new_angle = """// 头部低头角度（正 pitch = 低头，最大俯角 45°）
constexpr double kHeadDownPitchRad = M_PI_4;
"""
old_tick = """  onTransportEdge(ts, now);
  tickHandGripSequencer(now);
"""
new_tick = """  onTransportEdge(ts, now);
  // ControllerManager 将头部外部目标视为 0.5s 有效的实时指令；搬运 ACTIVE
  // 期间持续刷新最大俯角，避免单次状态边沿指令在到位前超时回到手柄目标。
  if (ts == kActive) {
    setHeadPitch(kHeadDownPitchRad);
  }
  tickHandGripSequencer(now);
"""

if new_angle in text and new_tick in text:
    print("补丁已存在，未修改文件。")
    raise SystemExit(0)
if text.count(old_angle) != 1 or text.count(old_tick) != 1:
    print("错误：当前源码与补丁适配版本不一致，未修改文件。", file=sys.stderr)
    raise SystemExit(1)

backup = path.with_suffix(path.suffix + ".bak")
if backup.exists():
    print(f"错误：备份文件已存在，未修改文件：{backup}", file=sys.stderr)
    raise SystemExit(1)
shutil.copy2(path, backup)
path.write_text(text.replace(old_angle, new_angle).replace(old_tick, new_tick))
print(f"已修改：{path}")
print(f"已备份：{backup}")
PY
