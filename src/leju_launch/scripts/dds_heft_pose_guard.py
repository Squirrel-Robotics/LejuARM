"""Joint-limit checks and plans for the S17 single-joint precision test.

All arrays use the 23-joint hardware/model order and radians. Tracking error,
model-driven leg motion, and IMU posture do not stop this precision test.
"""

from __future__ import annotations

import json
import math
from numbers import Real
from pathlib import Path

from dds_heft_pose_kinematics import load_joint_limits


MOTOR_NAMES = ([f"leg_{side}{i}_joint" for side in ("l", "r") for i in range(1, 7)]
               + ["waist_yaw_joint"]
               + [f"zarm_{side}{i}_joint" for side in ("l", "r") for i in range(1, 5)]
               + ["zhead_1_joint", "zhead_2_joint"])
AXES = [("head_yaw", 21), ("head_pitch", 22)] + [
    (f"{side}{i}", 13 + 4 * s + i - 1)
    for s, side in enumerate(("l", "r")) for i in range(1, 5)
] + [("waist", 12)]
GROUP_AXES = {"head": {"head_yaw", "head_pitch"},
              "arms": {f"{side}{i}" for side in ("l", "r") for i in range(1, 5)},
              "waist": {"waist"}}
HEAD_YAW_LIMIT_DEG = 60.0


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{label} 必须是有限数值")
    try:
        number = float(value)
    except (OverflowError, ValueError):
        raise ValueError(f"{label} 必须是有限数值") from None
    if not math.isfinite(number):
        raise ValueError(f"{label} 必须是有限数值")
    return number


def _angles(q, label):
    if not isinstance(q, (list, tuple)) or len(q) != 23:
        raise ValueError(f"{label} 必须含 23 个硬件顺序关节角")
    return [_number(value, f"{label}[{i}]") for i, value in enumerate(q)]


def _limit(limits, name):
    if name not in limits:
        raise ValueError(f"缺少关节限位：{name}")
    pair = limits[name]
    if not isinstance(pair, (list, tuple)) or len(pair) != 2:
        raise ValueError(f"{name} 限位必须为 (lower, upper)")
    lo, hi = (_number(value, f"{name} 限位") for value in pair)
    if lo > hi:
        raise ValueError(f"{name} 下限大于上限")
    return lo, hi


def load_combined_limits(urdf: Path, hardware_json: Path) -> dict:
    """Intersect finite URDF and hardware JSON limits for every S17 joint.

    The hardware min/max arrays contain exactly 23 values, in degrees. Their
    head sentinel bounds (±999 degrees) are still intersected with the URDF.
    Head yaw is additionally restricted to the confirmed ±60 degree range.
    Missing bounds, nonfinite values, and disjoint intervals are rejected.
    """
    urdf_limits = load_joint_limits(Path(urdf))
    data = json.loads(Path(hardware_json).read_text())
    if not isinstance(data, dict):
        raise ValueError("硬件配置必须是 JSON object")
    lower = _angles(data.get("min_joint_position_limits"), "硬件关节下限 [deg]")
    upper = _angles(data.get("max_joint_position_limits"), "硬件关节上限 [deg]")
    combined = {}
    for i, name in enumerate(MOTOR_NAMES):
        lo, hi = _limit(urdf_limits, name)
        if lower[i] > upper[i]:
            raise ValueError(f"{name} 硬件关节下限大于上限")
        lo, hi = max(lo, math.radians(lower[i])), min(hi, math.radians(upper[i]))
        if name == "zhead_1_joint":
            yaw_limit = math.radians(HEAD_YAW_LIMIT_DEG)
            lo, hi = max(lo, -yaw_limit), min(hi, yaw_limit)
        if lo > hi:
            raise ValueError(f"{name} URDF、硬件与测试有效限位没有交集")
        combined[name] = (lo, hi)
    return combined


def default_cases(groups: str, baseline: list, limits: dict,
                  offset_deg: float = 1.0, margin_deg: float = 0.0) -> tuple:
    """Plan +offset and -offset about the saved pose, returning skipped cases.

    Order is head yaw/pitch, left arm 1..4, right arm 1..4, then waist. Each
    target is relative to the baseline, so the caller returns to baseline
    between cases instead of accumulating previous offsets.
    """
    if not isinstance(groups, str):
        raise ValueError("--groups 只支持 head,arms,waist")
    selected = [item.strip() for item in groups.split(",")]
    if not selected or any(item not in GROUP_AXES for item in selected):
        raise ValueError("--groups 只支持 head,arms,waist")
    offset = _number(offset_deg, "默认偏移 [deg]")
    margin = _number(margin_deg, "限位余量 [deg]")
    if offset <= 0.0 or margin < 0.0:
        raise ValueError("默认偏移必须大于零，限位余量不能为负数")
    q = _angles(baseline, "基准关节角")
    # Refuse an invalid baseline even if a particular offset would be in range.
    guard_command(q, limits)
    allowed = set().union(*(GROUP_AXES[group] for group in selected))
    spec, skipped = [], []
    for axis, index in AXES:
        if axis not in allowed:
            continue
        lo, hi = _limit(limits, MOTOR_NAMES[index])
        safe_lo, safe_hi = math.degrees(lo) + margin, math.degrees(hi) - margin
        start = math.degrees(q[index])
        for delta in (offset, -offset):
            target = start + delta
            if safe_lo <= target <= safe_hi:
                spec.append(f"{axis}:{delta:+g}")
            else:
                skipped.append({"axis": axis, "motor_index": index,
                                "offset_deg": delta, "target_deg": target,
                                "lower_deg": safe_lo, "upper_deg": safe_hi,
                                "margin_deg": margin,
                                "reason": (f"目标 {target:+.3f}deg 不在留出 {margin:g}deg "
                                           f"余量后的限位 [{safe_lo:+.3f}, {safe_hi:+.3f}]deg 内")})
    return ",".join(spec), skipped


def range_cases(groups: str, baseline: list, limits: dict,
                points_per_joint: int = 7, edge_margin_deg: float = 2.0) -> tuple:
    """Plan individual targets across each selected joint's allowed range.

    Endpoints are inset by the requested margin, capped at 10% of the joint's
    range for narrow intervals. Each axis starts with the target closest to
    baseline; equally distant targets use increasing angle order. The caller
    returns to the same baseline between targets, changing only that axis.
    A zero-width interval contributes its one distinct allowed angle.
    """
    if not isinstance(groups, str):
        raise ValueError("--groups 只支持 head,arms,waist")
    selected = [item.strip() for item in groups.split(",")]
    if not selected or any(item not in GROUP_AXES for item in selected):
        raise ValueError("--groups 只支持 head,arms,waist")
    count = _number(points_per_joint, "每关节点数")
    margin = _number(edge_margin_deg, "范围端点余量 [deg]")
    if count < 1 or not count.is_integer():
        raise ValueError("每关节点数必须为正整数")
    if margin < 0:
        raise ValueError("范围端点余量不能为负数")
    count = int(count)
    q = _angles(baseline, "基准关节角")
    guard_command(q, limits)
    allowed = set().union(*(GROUP_AXES[group] for group in selected))
    spec = []
    for axis, index in AXES:
        if axis not in allowed:
            continue
        lo, hi = _limit(limits, MOTOR_NAMES[index])
        edge = min(math.radians(margin), (hi - lo) * 0.1)
        lower, upper = lo + edge, hi - edge
        if lower == upper or count == 1:
            targets = [(lower + upper) / 2.0]
        else:
            targets = [lower + (upper - lower) * i / (count - 1)
                       for i in range(count)]
            targets[0], targets[-1] = lower, upper
        targets = sorted(set(targets), key=lambda angle: (abs(angle - q[index]), angle))
        start_deg = math.degrees(q[index])
        for target in targets:
            offset_deg = math.degrees(target) - start_deg
            # Match parse_cases' degree addition, retaining enough precision to
            # reconstruct the target rather than rounding it to a coarse step.
            spec.append(f"{axis}:{offset_deg:+.17g}")
    return ",".join(spec), []


def guard_command(q, limits):
    """Reject a malformed or out-of-limit 23-joint reference before writing."""
    try:
        values = _angles(q, "指令关节角")
        for index, name in enumerate(MOTOR_NAMES):
            lo, hi = _limit(limits, name)
            if not lo <= values[index] <= hi:
                raise ValueError(f"{name} 指令 {math.degrees(values[index]):+.3f}deg "
                                 f"超出限位 [{math.degrees(lo):+.3f}, {math.degrees(hi):+.3f}]deg")
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"停止测试：{exc}") from exc


class LimitGuard:
    """Check only finite joint data and mechanical joint-position limits."""

    def __init__(self, limits):
        self.limits = limits
        for name in MOTOR_NAMES:
            _limit(limits, name)

    def check(self, io, command):
        guard_command(command, self.limits)
        try:
            actual = _angles(io.last.q if io.last is not None else None, "实测关节角")
            slack = math.radians(0.1)
            for index, name in enumerate(MOTOR_NAMES):
                lo, hi = _limit(self.limits, name)
                if not lo - slack <= actual[index] <= hi + slack:
                    raise ValueError(f"{name} 实测角 {math.degrees(actual[index]):+.3f}deg 越限")
        except (AttributeError, TypeError, ValueError) as exc:
            raise RuntimeError(f"停止测试：{exc}") from exc
