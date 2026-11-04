#!/usr/bin/env python3
"""S17 measured-baseline, one-joint-at-a-time angle accuracy test.

CLI angles and results use /rt/joint_state (hardware/model) coordinates, in
degrees. HEFT waist sign conversion is applied only at the DDS boundary.
Default execution is read-only. --execute sweeps seven targets per joint
from one range endpoint to the other, then returns once to the saved baseline.
--profile small-step selects the earlier +/-1 degree sequence.
"""
from __future__ import annotations

import argparse
import ctypes
import json
import math
import os
import secrets
import select
import signal
import statistics
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from types import new_class

ROOT = Path(__file__).resolve().parents[3]
HEFT_NAMES = (["waist_yaw_joint"]
              + [f"leg_{side}{i}_joint" for side in ("l", "r") for i in range(1, 7)]
              + [f"zarm_{side}{i}_joint" for side in ("l", "r") for i in range(1, 5)])
HEFT_TO_MOTOR = [12] + list(range(12)) + list(range(13, 21))
AXES = {"head_yaw": (21, "zhead_1_joint"), "head_pitch": (22, "zhead_2_joint"),
        "waist": (12, "waist_yaw_joint")}
AXES.update({f"{s}{i}": (13 + side * 4 + i - 1, f"zarm_{s}{i}_joint")
             for side, s in enumerate(("l", "r")) for i in range(1, 5)})
GROUPS = {"head": ["head_yaw", "head_pitch"], "arms": [f"{s}{i}" for s in ("l", "r") for i in range(1, 5)],
          "waist": ["waist"]}
JOINT_IDS = {str(number): axis for number, axis in enumerate(
    GROUPS["head"] + GROUPS["arms"] + GROUPS["waist"], 1)}
JOINT_ID_BY_AXIS = {axis: number for number, axis in JOINT_IDS.items()}
JOINT_MENU = ("关节编号（测试菜单编号）：\n"
              "  1=头yaw(head_yaw)  2=头pitch(head_pitch)\n"
              "  3=左臂1(l1)  4=左臂2(l2)  5=左臂3(l3)  6=左臂4(l4)\n"
              "  7=右臂1(r1)  8=右臂2(r2)  9=右臂3(r3)  10=右臂4(r4)\n"
              "  11=腰(waist)")
MOTOR_NAMES = HEFT_NAMES[1:13] + [HEFT_NAMES[0]] + HEFT_NAMES[13:] + ["zhead_1_joint", "zhead_2_joint"]
DEFAULT_RUN = ROOT / "validation/heft_pose_test"
DEFAULT_BASELINE = DEFAULT_RUN / "current_snapshot.json"
DEFAULT_RESULT = DEFAULT_RUN / "result.json"
DEFAULT_CSV = DEFAULT_RUN / "samples.csv"
DEFAULT_REPORT = DEFAULT_RUN / "report.md"
DEFAULT_COMMANDS = DEFAULT_RUN / "commands.jsonl"


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--groups", default="head,arms,waist", help="head,arms,waist 的组合")
    p.add_argument("--cases", default=None, help="可选自定义相对偏移 [deg]；默认按profile生成多个限位内目标")
    p.add_argument("--single-joint", action="store_true", help="连续单关节会话：从已记录姿态开始，测后保持目标，继续输入下一个关节和绝对角")
    p.add_argument("--joint", choices=list(AXES), help="只测试指定关节，与 --targets-deg 同用")
    p.add_argument("--targets-deg", help="逗号分隔的硬件绝对目标角 [deg]，按给定顺序测试；负数用 --targets-deg=... ")
    p.add_argument("--profile", choices=("range", "small-step"), default="range")
    p.add_argument("--trajectory", choices=("sweep", "per-point"), help="范围测试默认每关节一次往返；per-point为每点回基准")
    p.add_argument("--heft-for-head", action="store_true", help="仅选head时也发布全身HEFT基准，用于复现HEFT状态下的头部测试")
    p.add_argument("--points-per-joint", type=int, default=7)
    p.add_argument("--edge-margin-deg", type=float, default=2.0, help="范围采样点距机械端点的内侧距离，不是姿态退出条件")
    p.add_argument("--execute", action="store_true", help="开启测试；默认只读。确认HEFT切入后自动逐条测试")
    p.add_argument("--step-confirm", action="store_true", help="每个目标前额外等待 Enter，默认自动按顺序测试")
    p.add_argument("--output-dir", type=Path, help="输出目录；连续单关节会话默认 validation/heft_joint_test")
    p.add_argument("--snapshot", type=Path, help="只读记录的保存位置，默认 current_snapshot.json；执行时仅显式指定才另存启动快照")
    p.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE, help="执行/离线计划读取的初始姿态，默认上次记录的 current_snapshot.json；可手动指定备份")
    p.add_argument("--capture-duration", type=float, default=3.0)
    p.add_argument("--config", type=Path, default=ROOT / "src/leju-controllers/leju-rl-controller/config/17/controllers/config_heft.yaml")
    p.add_argument("--urdf", type=Path, default=ROOT / "src/leju_assets/models/biped_s17/urdf/biped_s17.urdf")
    p.add_argument("--hardware-config", type=Path, default=ROOT / "src/leju-hardware/config/roban_v17/kuavo.json")
    p.add_argument("--root-height", type=float, help="可覆盖根高度；默认用保存腿角、当前IMU根朝向与双足落地几何估计（非实测高度）")
    p.add_argument("--root-quat-wxyz", nargs=4, type=float, help="可覆盖根朝向；默认由当前torso IMU和实测腰角推导")
    p.add_argument("--rate", type=float, default=50.0)
    p.add_argument("--move-duration", type=float, default=1.5)
    p.add_argument("--max-speed-deg-s", type=float, default=10.0, help="参考插值峰值速度；大跨度只延长时间，不触发退出")
    p.add_argument("--settle-duration", type=float, default=2.0)
    p.add_argument("--avg-window", type=float, default=1.0)
    p.add_argument("--tolerance-deg", type=float, default=1.0)
    p.add_argument("--state-timeout", type=float, default=5.0)
    p.add_argument("--max-state-age", type=float, default=0.25)
    p.add_argument("--json", type=Path, help="汇总结果，默认输出目录下 result.json（离线/只读为 plan.json/capture.json）")
    p.add_argument("--csv", type=Path, help="逐条新反馈及全部23关节的已发布/实测/误差 [deg]")
    p.add_argument("--report", type=Path, help="全项目 Markdown 报告，精度FAIL也继续测试其余项目")
    p.add_argument("--commands", type=Path, help="每次实际成功发布的DDS消息原始值，JSONL")
    p.add_argument("--dry-run", action="store_true", help="只打印测试计划，不连接 DDS、不发指令")
    a = p.parse_args(argv)
    a.absolute_targets_deg = None
    if a.single_joint and a.cases is not None:
        p.error("连续单关节会话直接输入绝对角，不使用 --cases 相对偏移")
    if a.single_joint and a.trajectory == "per-point":
        p.error("连续单关节会话测后保持目标，不使用每点回基准的轨迹")
    if a.joint is not None or a.targets_deg is not None:
        if a.joint is None or a.targets_deg is None:
            p.error("单关节测试必须同时指定 --joint 和 --targets-deg，不会自动扫描限位")
        if a.cases is not None:
            p.error("--joint/--targets-deg 不能与相对偏移 --cases 混用")
        try:
            a.absolute_targets_deg = [float(v.strip()) for v in a.targets_deg.split(",")]
        except ValueError:
            p.error("--targets-deg 必须为逗号分隔的角度数值")
        if not a.absolute_targets_deg or not all(math.isfinite(v) for v in a.absolute_targets_deg):
            p.error("--targets-deg 必须为有限角度数值")
        a.groups = next(group for group, axes in GROUPS.items() if a.joint in axes)
    if a.trajectory is None:
        a.trajectory = "sweep" if a.joint is not None or (a.profile == "range" and a.cases is None) else "per-point"
    for name in ("rate", "move_duration", "max_speed_deg_s", "settle_duration", "avg_window", "tolerance_deg", "state_timeout", "max_state_age", "capture_duration"):
        if not math.isfinite(getattr(a, name)) or getattr(a, name) <= 0:
            p.error(f"--{name.replace('_', '-')} 必须是有限正数")
    if a.points_per_joint < 2:
        p.error("--points-per-joint 至少为2")
    if not math.isfinite(a.edge_margin_deg) or a.edge_margin_deg < 0:
        p.error("--edge-margin-deg 必须是有限非负数")
    if not 50 <= a.rate <= 200:
        p.error("--rate 必须在 50..200 Hz")
    if a.move_duration < 1.0 or a.settle_duration < a.avg_window + 0.5:
        p.error("move-duration 至少 1s；settle-duration 至少比 avg-window 长 0.5s（留出参考缓冲延迟）")
    if a.root_height is not None and (not math.isfinite(a.root_height) or a.root_height <= 0):
        p.error("--root-height 必须是有限正数 [m]")
    if a.root_quat_wxyz is not None and (not all(math.isfinite(x) for x in a.root_quat_wxyz)
            or abs(sum(x * x for x in a.root_quat_wxyz) - 1.0) > 1e-3):
        p.error("--root-quat-wxyz 必须是有限单位四元数")
    if a.execute and a.dry_run:
        p.error("--execute 不能与 --dry-run 同用")
    output_dir = a.output_dir or (ROOT / "validation/heft_joint_test" if a.single_joint else
                                  ROOT / "validation/heft_joint_test" / a.joint if a.joint else DEFAULT_RUN)
    if a.json is None:
        a.json = output_dir / ("result.json" if a.execute else "plan.json" if a.dry_run else "capture.json")
    if a.snapshot is None and not a.execute and not a.dry_run:
        a.snapshot = DEFAULT_BASELINE
    if a.execute and a.snapshot is not None and a.snapshot.resolve() == a.baseline.resolve():
        p.error("执行测试不覆盖初始姿态文件；请先不带 --execute 记录，再启动测试")
    for name, filename in (("csv", "samples.csv"), ("report", "report.md"), ("commands", "commands.jsonl")):
        if getattr(a, name) is None:
            setattr(a, name, output_dir / filename)
    return a


def parse_cases(spec, groups, limits, baseline=None):
    selected = [s.strip() for s in groups.split(",")]
    if not selected or any(s not in GROUPS for s in selected):
        raise ValueError("--groups 只支持 head,arms,waist")
    allowed = {axis for group in selected for axis in GROUPS[group]}
    cases = []
    if spec is None:
        return []
    for item in spec.split(","):
        axis, sep, value = item.strip().partition(":")
        aliases = {name: key for key, (_, name) in AXES.items()}
        axis = aliases.get(axis, axis)
        if not sep or axis not in AXES:
            raise ValueError(f"非法用例 {item!r}；轴为 {', '.join(AXES)}")
        if axis not in allowed:
            raise ValueError(f"{axis} 不在 --groups 范围内")
        deg = float(value)
        if not math.isfinite(deg):
            raise ValueError(f"{axis}:{value} 相对偏移必须是有限数值")
        index = AXES[axis][0]
        target_deg = None if baseline is None else math.degrees(baseline[index]) + deg
        lo, hi = limits[AXES[axis][1]]
        if target_deg is not None:
            target_rad = math.radians(target_deg)
            if target_rad < lo - 1e-12 or target_rad > hi + 1e-12:
                raise ValueError(f"{axis} 目标 {target_deg:.3f}deg 超出测试有效限位 [{math.degrees(lo):.1f}, {math.degrees(hi):.1f}] deg")
            target_deg = math.degrees(min(hi, max(lo, target_rad)))
        cases.append({"axis": axis, "motor_index": index, "offset_deg": deg, "target_deg": target_deg})
    if not cases:
        raise ValueError("没有选中的用例")
    return cases


def sweep_order(cases, baseline):
    grouped = {}
    for case in cases:
        grouped.setdefault(case["axis"], []).append(case)
    ordered = []
    for group in grouped.values():
        group = sorted(group, key=lambda case: case["target_deg"])
        start = math.degrees(baseline[group[0]["motor_index"]])
        if abs(group[-1]["target_deg"] - start) < abs(group[0]["target_deg"] - start):
            group.reverse()
        ordered.extend(group)
    return ordered


def load_config(path):
    import yaml
    cfg = yaml.safe_load(path.read_text())["HumanoidRobotCfg"]["env"]
    robot, live = cfg["robot"], cfg["heft"]["live"]
    if robot["joint_names"] != HEFT_NAMES or robot["joint_direction"] != [-1] + [1] * 20:
        raise ValueError("脚本仅支持 S17 的 21 关节顺序和 waist=-1、其余=+1 的配置")
    if any(robot["joint_default_pos"]) or live["robot_version"] != "roban_s17" or live["schema_version"] != 1:
        raise ValueError("配置与 S17 HEFT 测试契约不一致")
    return live


def to_heft(motor_q):
    return [-motor_q[12]] + list(motor_q[:12]) + list(motor_q[13:21])


def interpolate(start, target, phase, duration):
    u = max(0.0, min(1.0, phase))
    weight = u * u * (3.0 - 2.0 * u)
    velocity = 6.0 * u * (1.0 - u) / duration
    return ([x + weight * (y - x) for x, y in zip(start, target)],
            [velocity * (y - x) for x, y in zip(start, target)])


@dataclass
class Feedback:
    stamp: float
    received: float
    q: list


@dataclass
class ImuFeedback:
    stamp: float
    received: float
    quat: list


class TestIo:
    def __init__(self, topic, max_age):
        lib_dir = os.environ.get("CYCLONEDDS_LIB_DIR")
        if lib_dir:
            ctypes.CDLL(str(Path(lib_dir) / "libddsc.so"))
        from cyclonedds.domain import DomainParticipant
        from cyclonedds.idl import IdlStruct
        from cyclonedds.idl.types import array, float64, int32, int64, sequence, uint32, uint64
        from cyclonedds.sub import DataReader
        from cyclonedds.topic import Topic
        from dds_leju_types import JointState, JointTrajectoryPoint

        # Resolve annotations eagerly; new_class runs CycloneDDS metaclass
        # __prepare__, which assigns the wire typename (plain type() skips it).
        annotations = {"header_sec": int32, "header_nanosec": uint32,
                       "schema_version": uint32, "robot_version": str, "seq": uint64,
                       "calibration_epoch": uint32, "capture_timestamp_ms": int64,
                       "valid": bool, "calibrated": bool,
                       "root_pos": array[float64, 3], "root_quat": array[float64, 4],
                       "q": sequence[float64], "qd": sequence[float64]}
        self.HeftReference = dataclass(new_class(
            "HeftReference", (IdlStruct,), {"typename": "leju::msgs::HeftReference"},
            lambda ns: ns.update(__annotations__=annotations, __module__=__name__)))
        imu_annotations = {"header_sec": int32, "header_nanosec": uint32,
                           "gyro": array[float64, 3], "acc": array[float64, 3],
                           "free_acc": array[float64, 3], "quat": array[float64, 4]}
        self.ImuData = dataclass(new_class(
            "ImuData", (IdlStruct,), {"typename": "leju::msgs::ImuData"},
            lambda ns: ns.update(__annotations__=imu_annotations, __module__=__name__)))
        self.Head = JointTrajectoryPoint
        self.participant = DomainParticipant(0)
        self.heft, self.head = None, None
        self.topic, self.root_quat = topic, None
        self.publishing_enabled = False
        self.last_partial_publication = None
        self.reader = DataReader(self.participant, Topic(self.participant, "/rt/joint_state", JointState))
        self.imu_reader = DataReader(self.participant, Topic(self.participant, "/rt/imu_state", self.ImuData))
        self.seq, self.epoch = 0, secrets.randbits(32) or 1
        self.last = None
        self.last_imu = None
        self.max_age = max_age

    def enable_writers(self, use_heft, root_quat=None):
        from cyclonedds.pub import DataWriter
        from cyclonedds.topic import Topic
        if use_heft:
            if root_quat is None:
                raise ValueError("HEFT 根姿态尚未提供")
            self.heft = DataWriter(self.participant, Topic(self.participant, self.topic, self.HeftReference))
            self.root_quat = list(root_quat)
        self.head = DataWriter(self.participant, Topic(self.participant, "/rt/head_trajectory", self.Head))
        self.publishing_enabled = True

    def publish(self, motor_q, motor_v, root_height):
        if not self.publishing_enabled:
            raise RuntimeError("默认只读模式禁止发布指令；必须明确开启 --execute")
        self.last_partial_publication = None
        ns = time.time_ns()
        sec, nsec = divmod(ns, 1_000_000_000)
        self.seq += 1
        publication = {"stamp_ns": ns, "seq": self.seq,
                       "calibration_epoch": self.epoch,
                       "hardware_reference_q_rad": list(motor_q),
                       "hardware_reference_qd_rad_s": list(motor_v),
                       "heft": None, "head": None}
        if self.heft is not None:
            fields = dict(
                header_sec=sec, header_nanosec=nsec, schema_version=1,
                robot_version="roban_s17", seq=self.seq, calibration_epoch=self.epoch,
                capture_timestamp_ms=ns // 1_000_000, valid=True, calibrated=True,
                root_pos=[0.0, 0.0, root_height], root_quat=self.root_quat,
                q=to_heft(motor_q), qd=to_heft(motor_v))
            self.heft.write(self.HeftReference(**fields))
            publication["heft"] = {"topic": self.topic, **fields}
            self.last_partial_publication = publication
        fields = dict(header_sec=sec, header_nanosec=nsec,
                      q=list(motor_q[21:23]), v=list(motor_v[21:23]), acc=[0.0, 0.0])
        self.head.write(self.Head(**fields))
        publication["head"] = {"topic": "/rt/head_trajectory", **fields}
        self.last_partial_publication = None
        return publication

    def read(self):
        fresh = []
        now, wall = time.monotonic(), time.time()
        if hasattr(self, "imu_reader"):
            for sample in self.imu_reader.take(N=1000):
                if not hasattr(sample, "quat"):
                    continue
                stamp = sample.header_sec + sample.header_nanosec * 1e-9
                if not -0.05 <= wall - stamp <= self.max_age:
                    continue
                if self.last_imu is not None and stamp <= self.last_imu.stamp:
                    continue
                quat = list(sample.quat)
                if len(quat) != 4 or not all(math.isfinite(x) for x in quat):
                    continue
                norm = math.sqrt(sum(x * x for x in quat))
                if abs(norm - 1.0) > 0.01:
                    continue
                self.last_imu = ImuFeedback(stamp, now, [x / norm for x in quat])
        for sample in self.reader.take(N=1000):
            if not hasattr(sample, "q") or len(sample.q) != 23:
                continue
            stamp = sample.header_sec + sample.header_nanosec * 1e-9
            if not -0.05 <= wall - stamp <= self.max_age:
                continue
            if self.last is not None and stamp <= self.last.stamp:
                continue
            q = list(sample.q)
            if not all(math.isfinite(x) for x in q):
                continue
            self.last = Feedback(stamp, now, q)
            fresh.append(self.last)
        return fresh

    def require_fresh(self):
        if (self.last is None
                or time.monotonic() - self.last.received > self.max_age
                or not -0.05 <= time.time() - self.last.stamp <= self.max_age):
            raise RuntimeError("/rt/joint_state 反馈超时；停止测试和发布，结果不判为通过")

    def require_imu(self):
        if (self.last_imu is None or time.monotonic() - self.last_imu.received > self.max_age
                or not -0.05 <= time.time() - self.last_imu.stamp <= self.max_age):
            raise RuntimeError("/rt/imu_state 数据缺失或超时，拒绝继续 HEFT 测试")

    def wait(self, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.read()
            if self.last is not None:
                return self.last.q.copy()
            time.sleep(0.02)
        raise RuntimeError("未收到新鲜的 23 关节 /rt/joint_state；请先启动并 START 站立")


def window_stats(samples, target, case, args):
    now = time.time()
    cutoff = now - args.avg_window
    window = [s for s in samples if cutoff <= s.stamp <= now + 0.05]
    if len(window) < 10 or window[-1].stamp - window[0].stamp < args.avg_window * 0.75:
        raise RuntimeError("统计窗口内新反馈不足，拒绝使用旧样本补齐")
    if now - window[-1].stamp > args.max_state_age:
        raise RuntimeError("统计窗口末尾反馈已超时")
    gaps = [b.stamp - a.stamp for a, b in zip(window, window[1:])]
    if max(gaps, default=0) > args.max_state_age:
        raise RuntimeError("统计窗口内反馈存在超时间断")
    idx = case["motor_index"]
    values = [math.degrees(s.q[idx]) for s in window]
    errors = [v - case["target_deg"] for v in values]
    mean = statistics.fmean(values)
    err = mean - case["target_deg"]
    cross = {axis: statistics.fmean(math.degrees(s.q[i] - target[i]) for s in window)
             for axis, (i, _) in AXES.items()}
    return {**case, "actual_deg": mean, "error_deg": err,
            "abs_error_deg": abs(err),
            "std_deg": statistics.pstdev(values),
            "rmse_deg": math.sqrt(statistics.fmean(e * e for e in errors)),
            "max_abs_error_deg": max(abs(e) for e in errors),
            "samples": len(window), "held_joint_mean_error_deg": cross,
            "window_start_timestamp": window[0].stamp, "window_end_timestamp": window[-1].stamp,
            "pass": abs(err) <= args.tolerance_deg}


def controller_evidence(observer, start, end, requires_heft):
    observations = observer.observations(start, end) if observer is not None else []
    if not requires_heft:
        status = "independent_head"
    elif not observations:
        status = "unknown"
    elif all(entry["current_controller"] == "heft_pico" and not entry["controller_transitioning"]
             for entry in observations):
        status = "observed_heft"
    else:
        status = "other_or_transitioning"
    return {"controller_observation_status": status,
            "controller_observations": observations,
            "controller_observer_error": None if observer is None else observer.error}


class Runner:
    def __init__(self, io, args, height, csv_file=None, guard=None, commands_file=None):
        self.io, self.args, self.height = io, args, height
        self.command = None
        self.last_publication = None
        self.commands_file = commands_file
        self.guard = guard
        self.next_tick = time.monotonic()
        self.csv = None
        if csv_file:
            import csv
            self.csv = csv.writer(csv_file)
            self.csv.writerow(["state_timestamp", "received_monotonic", "phase", "reference_seq", "reference_stamp_ns"]
                             + [f"{name}_{field}_deg" for name in MOTOR_NAMES for field in ("published", "actual", "error")])

    def _record_feedback(self, states, phase, command, publication):
        if not self.csv:
            return
        publication = publication if isinstance(publication, dict) else {}
        for state in states:
            values = []
            for i in range(23):
                actual_deg = math.degrees(state.q[i])
                published_deg = None if command is None else math.degrees(command[i])
                values.extend(["" if published_deg is None else published_deg, actual_deg,
                               "" if published_deg is None else actual_deg - published_deg])
            self.csv.writerow([state.stamp, state.received, phase, publication.get("seq", ""),
                               publication.get("stamp_ns", "")] + values)

    def tick(self, q, v, phase):
        states = self.io.read()
        self.io.require_fresh()
        if self.guard is not None:
            self.guard.check(self.io, q)
        self._record_feedback(states, phase, self.command, self.last_publication)
        try:
            publication = self.io.publish(q, v, self.height)
        except BaseException as exc:
            partial = getattr(self.io, "last_partial_publication", None)
            if self.commands_file and isinstance(partial, dict):
                self.commands_file.write(json.dumps({"phase": phase, **partial,
                    "partial": True, "publish_error": str(exc)}, allow_nan=False) + "\n")
            raise
        self.command = list(q)
        self.last_publication = publication
        if self.commands_file:
            self.commands_file.write(json.dumps({"phase": phase, **publication}, allow_nan=False) + "\n")
        after = self.io.read()
        self._record_feedback(after, phase, self.command, publication)
        states.extend(after)
        self.next_tick += 1.0 / self.args.rate
        now = time.monotonic()
        if self.next_tick < now:
            # Never burst queued reference frames after a scheduling delay.
            self.next_tick = now
        time.sleep(max(0.0, self.next_tick - now))
        return states

    def drive(self, start, target, duration, phase):
        distance_deg = max(abs(math.degrees(b - a)) for a, b in zip(start, target))
        duration = max(duration, 1.5 * distance_deg / self.args.max_speed_deg_s)
        begin = time.monotonic()
        while True:
            u = min((time.monotonic() - begin) / duration, 1.0)
            q, v = interpolate(start, target, u, duration)
            self.tick(q, v, phase)
            if u >= 1.0:
                return

    def hold(self, target, duration, phase):
        end, samples = time.monotonic() + duration, []
        while time.monotonic() < end:
            samples.extend(self.tick(target, [0.0] * 23, phase))
        return samples

    def prompt(self, target, message):
        self.read_command(target, message)

    def read_command(self, target, message):
        if not sys.stdin.isatty():
            raise RuntimeError("实测需要交互终端（远程请 ssh -t）；不支持重定向 stdin")
        print(message, flush=True)
        while True:
            self.tick(target, [0.0] * 23, "operator_wait")
            if select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if not line:
                    raise RuntimeError("终端已断开")
                return line.strip()


def load_baseline(path):
    data = json.loads(path.read_text())
    q = data.get("motor_q_rad", [])
    if (data.get("schema") != "s17_measured_baseline_v1" or data.get("joint_names") != MOTOR_NAMES
            or len(q) != 23 or not all(isinstance(x, (int, float)) and not isinstance(x, bool)
                                     and math.isfinite(x) for x in q)):
        raise ValueError("姿态快照 schema、关节顺序或数据不匹配")
    if "motor_q_deg" in data:
        deg = data["motor_q_deg"]
        if (not isinstance(deg, list) or len(deg) != 23
                or not all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
                           and abs(x - math.degrees(y)) < 1e-6 for x, y in zip(deg, q))):
            raise ValueError("快照的 motor_q_rad 与 motor_q_deg 不一致")
    return data


def capture_baseline(io, args):
    io.wait(args.state_timeout)
    samples = [io.last]
    end = time.monotonic() + args.capture_duration
    while time.monotonic() < end:
        samples.extend(io.read())
        io.require_fresh()
        time.sleep(0.01)
    if len(samples) < 20 or samples[-1].stamp - samples[0].stamp < 0.75 * args.capture_duration:
        raise RuntimeError("采集窗口新鲜反馈不足")
    q = [statistics.fmean(s.q[i] for s in samples) for i in range(23)]
    spans = [math.degrees(max(s.q[i] for s in samples) - min(s.q[i] for s in samples)) for i in range(23)]
    imu = getattr(io, "last_imu", None)
    return {"schema": "s17_measured_baseline_v1", "captured_at": time.time(),
            "source_topic": "/rt/joint_state", "joint_names": MOTOR_NAMES,
            "controller": "operator must confirm; not queried by script",
            "motor_q_rad": q, "motor_q_deg": [math.degrees(x) for x in q],
            "joint_range_deg": spans, "sample_count": len(samples),
            "span_sec": samples[-1].stamp - samples[0].stamp,
            "imu_quat_wxyz": None if imu is None else list(imu.quat),
            "imu_timestamp": None if imu is None else imu.stamp,
            "root_pose_measured": False, "motion_validated": False}


def write_json(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def write_markdown_report(path, report):
    """Include every planned case, even when a hard fault leaves it untested."""
    def number(value):
        return "—" if value is None else f"{value:+.3f}"

    def stamp(value):
        return datetime.fromtimestamp(value, timezone(timedelta(hours=8))).strftime("%Y-%m-%d %H:%M:%S")

    cases, results = report.get("cases", []), report.get("results", [])
    passed = sum(bool(r["pass"]) for r in results)
    lines = ["# Roban S17 头、双臂、腰部关节角误差报告", "",
             f"- 开始时间：{stamp(report['started_at'])}（北京时间）",
             f"- 运行状态：`{report['status']}`",
             f"- 计划 {len(cases)} 项；已测 {len(results)} 项；通过 {passed} 项；不通过 {len(results)-passed} 项；未测 {max(0,len(cases)-len(results))} 项。",
             f"- 稳态均值误差阈值：±{report['tolerance_deg']:g}°；参考频率：{report['rate_hz']:g} Hz。",
             report.get("trajectory_description",
              "- 每关节沿同一方向依次测完全部目标，再返回一次保存的基准；每次只改变一个参考关节。" if report.get("trajectory") == "sweep" else
              "- 目标基于指定的基准文件，每个目标测完后返回同一基准；每次只改变一个参考关节。") +
             "一般精度FAIL记录后继续；仅机械越限、无效数据、通信或执行错误才中止。",
             "- 误差 = 实测均值 − 实际发布参考角，统一使用硬件角度坐标。DDS线值列记录对应q槽位的值；腰关节线值与硬件参考角符号相反。",
             "- 控制器状态通过只读RPC约每秒查询一次，逐点记录保持阶段的回复；这是离散观测。PASS/FAIL仅判角度精度，HEFT状态未知或观测到其他控制器会单列标注，不触发运动退出。", ""]
    if report.get("operator_heft_confirmed_at"):
        lines += [f"- 操作者确认已切入HEFT的时间：{stamp(report['operator_heft_confirmed_at'])}（北京时间）。", ""]
    if report.get("error"):
        lines += [f"**中止原因：{report['error']}**", ""]
    coverage = report.get("coverage", {})
    if coverage:
        lines += ["## 每关节覆盖范围", "",
                  "| 关节 | 机械范围 ° | 计划目标范围 ° | 跨度覆盖 | 计划点数 | 实测点数 | 实测均值范围 ° | 最大绝对均值误差 ° |",
                  "|---|---|---|---:|---:|---:|---|---:|"]
        for axis, details in coverage.items():
            targets = details["planned_targets_deg"]
            measured = [r for r in results if r["axis"] == axis]
            actual_range = "—" if not measured else f"{number(min(r['actual_deg'] for r in measured))} ～ {number(max(r['actual_deg'] for r in measured))}"
            worst = None if not measured else max(abs(r["error_deg"]) for r in measured)
            lines.append(f"| {axis} | {number(details['lower_limit_deg'])} ～ {number(details['upper_limit_deg'])} | "
                         f"{number(min(targets))} ～ {number(max(targets))} | {details['planned_coverage_percent']:.1f}% | "
                         f"{len(targets)} | {len(measured)} | {actual_range} | {number(worst)} |")
    lines += ["", "## 全项目结果", "",
              "| 序号 | 关节 | 相对偏移 ° | 计划目标 ° | 实际发布参考 ° | DDS线值 ° | 实测均值 ° | 误差 ° | 绝对误差 ° | 标准差 ° | RMSE ° | 样本 | 判定 | 控制器观测 |",
              "|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|"]
    for index, case in enumerate(cases, 1):
        result = results[index - 1] if index <= len(results) else None
        if result is None:
            lines.append(f"| {index} | {case['axis']} | {number(case.get('offset_deg'))} | {number(case.get('target_deg'))} | — | — | — | — | — | — | — | 0 | 未测 | — |")
            continue
        verdict = "PASS" if result["pass"] else "FAIL"
        if result.get("no_response"):
            verdict += " 无响应"
        if result.get("sign_suspect"):
            verdict += " 方向异常"
        state_status = result.get("controller_observation_status", "unknown")
        state_text = {"independent_head": "独立头部通道", "unknown": "未知，HEFT未证实",
                      "observed_heft": "heft_pico", "other_or_transitioning": "其他控制器或切换中"}[state_status]
        observed_names = sorted({entry["current_controller"] for entry in result.get("controller_observations", [])})
        if state_status == "other_or_transitioning":
            state_text += "：" + ", ".join(observed_names)
        lines.append(f"| {index} | {case['axis']} | {number(case.get('offset_deg'))} | {number(case.get('target_deg'))} | "
                     f"{number(result.get('published_deg'))} | {number(result.get('published_wire_deg'))} | {number(result['actual_deg'])} | "
                     f"{number(result['error_deg'])} | {number(abs(result['error_deg']))} | {number(result['std_deg'])} | "
                     f"{number(result['rmse_deg'])} | {result['samples']} | {verdict} | {state_text} |")
    checks = report.get("baseline_tracking", [])
    if checks:
        lines += ["", "## 偏移前的基准跟踪", "", "| 关节 | 基准 ° | 实测 ° | 均值误差 ° | 判定 |", "|---|---:|---:|---:|---|"]
        for result in checks:
            lines.append(f"| {result['axis']} | {number(result['target_deg'])} | {number(result['actual_deg'])} | {number(result['error_deg'])} | {'PASS' if result['pass'] else 'FAIL'} |")
    lines += ["", "## 数据与边界", "",
              f"- 基准文件：`{report.get('baseline_path', '')}`",
              f"- JSON：`{report.get('json_path', '')}`",
              f"- 采样 CSV：`{report.get('csv_path', '')}`",
              f"- 每次成功发布的DDS原始消息 JSONL：`{report.get('commands_path', '')}`",
              "- 反馈CSV含全部23关节的实测值、接收时最近已成功发布的参考值、误差、seq与时间戳。该配对未补偿HEFT缓冲延迟；稳态报告使用保持阶段末段反馈。",
              "- 头部为独立 CSP 通道；手臂、腰部为 HEFT 全身参考跟踪。",
              "- 使用 URDF 与硬件限位交集，头部 yaw 额外限制为±60°，并检查反馈有效性；不因倾角变化、腿部运动、跟踪误差或初姿差异而退出。",
              "- 根朝向由 torso IMU 与腰角推导，根高度为 URDF 接触几何估计；不是根位姿的直接测量。",
              "- 本报告衡量关节编码器角度跟踪误差，不代表末端毫米精度或平衡稳定性保证。", ""]
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text("\n".join(lines))
    temporary.replace(path)


def terminal_emphasis(text, color="1"):
    if sys.stdout.isatty() and os.environ.get("TERM") != "dumb" and "NO_COLOR" not in os.environ:
        return f"\033[{color}m{text}\033[0m"
    return text


def print_test_start(number, case, start_deg):
    axis = case["axis"]
    heading = f"测试 #{number} | 关节编号 {JOINT_ID_BY_AXIS[axis]} ({axis})"
    print("\n" + "=" * 64, flush=True)
    print(terminal_emphasis(heading, "1;36"), flush=True)
    print(f"参考角：{start_deg:+.3f}° -> {case['target_deg']:+.3f}°；其余参考保持不变。", flush=True)
    print("-" * 64, flush=True)


def print_test_result(number, result):
    verdict = "PASS" if result["pass"] else "FAIL"
    print(f"\n{'-' * 20} 测试 #{number} 结果 {'-' * 20}", flush=True)
    print(f"  发布角：{result['published_deg']:+.3f}°", flush=True)
    print(f"  实际角：{result['actual_deg']:+.3f}°", flush=True)
    error = (f">>> 误差：{result['error_deg']:+.3f}°  |  "
             f"绝对误差：{abs(result['error_deg']):.3f}°  |  {verdict}")
    print(terminal_emphasis(error, "1;32" if result["pass"] else "1;31"), flush=True)
    print(f"  标准差：{result['std_deg']:.3f}°  RMSE：{result['rmse_deg']:.3f}°  样本数：{result['samples']}", flush=True)
    print("=" * 64 + "\n", flush=True)


def session_cases(command, reference, limits):
    fields = command.split(maxsplit=1)
    if len(fields) != 2:
        raise ValueError("输入格式：编号 绝对角度；编号为1..11，多个角可用空格或逗号分隔")
    axis, values = fields
    axis = JOINT_IDS.get(axis, axis)
    axis = {name: key for key, (_, name) in AXES.items()}.get(axis, axis)
    if axis not in AXES:
        raise ValueError("关节编号必须在1..11内，请按提示中的编号选择")
    targets = [float(value) for value in values.replace(",", " ").split()]
    if not targets or not all(math.isfinite(value) for value in targets):
        raise ValueError("目标角必须是有限数值，单位度")
    start = math.degrees(reference[AXES[axis][0]])
    group = next(group for group, axes in GROUPS.items() if axis in axes)
    spec = ",".join(f"{axis}:{target - start:.17g}" for target in targets)
    return parse_cases(spec, group, limits, reference)


def run_joint_session(runner, args, baseline, limits, observer, report, save, live, queued):
    """Keep one live stream; each command changes only the selected reference."""
    current = list(baseline)
    report.update(cases=[], results=[], coverage={}, profile="interactive_single_joint",
                  trajectory="hold_last", angle_convention="hardware absolute targets in degrees; offsets relative to the previous reference",
                  trajectory_description="- 从已记录的初始姿态开始，每次只改变指定关节的参考角；测完保持该目标，后续测试保留所有此前设置的参考角，不自动回位。")
    report.pop("tested_joint", None)
    save()
    while True:
        if queued:
            cases, queued = queued, []
        else:
            command = runner.read_command(current, JOINT_MENU + "\n输入：编号 绝对角度(°)（可空格/逗号分隔多个点）；show 查看参考；quit 结束。等待时持续保持当前参考：")
            if command.lower() in ("quit", "exit"):
                break
            if command.lower() == "show":
                print("当前发布参考角 [deg]（— 表示不在测试菜单内）：", flush=True)
                ids_by_name = {AXES[axis][1]: number for number, axis in JOINT_IDS.items()}
                for name, value in zip(MOTOR_NAMES, current):
                    print(f"  编号 {ids_by_name.get(name, '—'):>2}  {name:18} {math.degrees(value):+9.3f}")
                continue
            if not command:
                continue
            try:
                cases = session_cases(command, current, limits)
            except ValueError as exc:
                print(f"未执行：{exc}", flush=True)
                continue
        for requested in cases:
            axis, index = requested["axis"], requested["motor_index"]
            target = current.copy()
            target[index] = math.radians(requested["target_deg"])
            case = {**requested, "joint_number": int(JOINT_ID_BY_AXIS[axis]),
                    "offset_deg": math.degrees(target[index] - current[index]),
                    "reference_before_deg": list(map(math.degrees, current)),
                    "reference_target_deg": list(map(math.degrees, target))}
            report["cases"].append(case)
            lo, hi = limits[AXES[axis][1]]
            coverage = report["coverage"].setdefault(axis, {
                "lower_limit_deg": math.degrees(lo), "upper_limit_deg": math.degrees(hi),
                "planned_targets_deg": []})
            coverage["planned_targets_deg"].append(case["target_deg"])
            targets = coverage["planned_targets_deg"]
            width = math.degrees(hi - lo)
            coverage["planned_coverage_percent"] = 100 * (max(targets) - min(targets)) / width if width > 0 else 100.0
            report.update(status="running", current_reference_deg=list(map(math.degrees, current)))
            save()
            number = len(report["cases"])
            label = f"{number}:{axis}:{case['target_deg']:+.6f}"
            print_test_start(number, case, math.degrees(current[index]))
            runner.drive(current, target, args.move_duration, label + ":move")
            hold_started_at = time.time()
            samples = runner.hold(target, args.settle_duration, label + ":hold")
            published = math.degrees(runner.command[index])
            result = window_stats(samples, runner.command, {**case, "target_deg": published}, args)
            publication = runner.last_publication or {}
            result.update(planned_target_deg=case["target_deg"], published_deg=published,
                          published_wire_deg=-published if index == 12 else published,
                          published_topic="/rt/head_trajectory" if index >= 21 else live["topic"],
                          published_q_index=index - 21 if index >= 21 else HEFT_TO_MOTOR.index(index),
                          publication_seq=publication.get("seq"), publication_stamp_ns=publication.get("stamp_ns"),
                          abs_error_deg=abs(result["error_deg"]), trajectory="hold_last")
            result.update(controller_evidence(observer, hold_started_at, time.time(), index < 21))
            current = target
            report["results"].append(result)
            report["current_reference_deg"] = list(map(math.degrees, current))
            save()
            print_test_result(number, result)
            print("保持本次参考；可继续输入下一项。", flush=True)
    passed = all(result["pass"] for result in report["results"])
    heft_results = [result for result in report["results"] if result["motor_index"] < 21]
    observed = sum(result["controller_observation_status"] == "observed_heft" for result in heft_results)
    report.update(status="completed" if passed else "accuracy_failed", passed=passed,
                  heft_points_with_state_evidence=observed, heft_points_total=len(heft_results),
                  all_heft_points_observed=observed == len(heft_results))
    save()
    runner.prompt(current, f"报告已保存：{args.report}。保持最后参考；先按 X 并确认回到 AMP，再按 Enter 停止发布。")
    return 0 if passed else 1


def main(argv=None):
    args = parse_args(argv)
    if (args.execute or args.dry_run) and not args.baseline.is_file():
        print(f"未找到已记录的初始姿态：{args.baseline}\n"
              "请先运行测试入口且不带 --execute 记录当前位置，再加 --execute 启动；"
              "也可用 --baseline 指定已有备份。未连接 DDS、未发布指令。", file=sys.stderr)
        return 2
    report = {"started_at": time.time(), "status": "preparing", "results": [],
              "angle_convention": "hardware absolute targets in degrees; offsets relative to the saved baseline",
              "motion_validated": False, "publishing_requested": args.execute,
              "tolerance_deg": args.tolerance_deg, "rate_hz": args.rate}
    csv_file, commands_file, observer = None, None, None

    def save():
        if args.json:
            write_json(args.json, report)
        if args.execute:
            write_markdown_report(args.report, report)

    try:
        from dds_heft_pose_kinematics import estimate_root_height, root_quat_from_imu
        from dds_heft_pose_guard import LimitGuard, default_cases, range_cases, guard_command, load_combined_limits
        limits = load_combined_limits(args.urdf, args.hardware_config)
        io, snapshot = None, None
        if not args.execute and not args.dry_run:
            print("只读采集当前关节反馈，不创建指令发布器……", flush=True)
            io = TestIo("/rt/pico/heft_reference", args.max_state_age)
            snapshot = capture_baseline(io, args)
            guard_command(snapshot["motor_q_rad"], limits)
            write_json(args.snapshot, snapshot)
            saved = snapshot
            args.baseline = args.snapshot
            next_use = ("下次 --execute 默认读取该记录。" if args.snapshot.resolve() == DEFAULT_BASELINE.resolve() else
                        f"使用该记录测试时加 --baseline {args.snapshot}。")
            print(f"初始姿态已记录：{args.snapshot}，{snapshot['sample_count']} 帧 / {snapshot['span_sec']:.2f}s；{next_use}")
        else:
            saved = load_baseline(args.baseline)
        baseline = saved["motor_q_rad"]
        guard_command(baseline, limits)
        spec, skipped = args.cases, []
        if args.joint is not None:
            start = math.degrees(baseline[AXES[args.joint][0]])
            spec = ",".join(f"{args.joint}:{angle - start:.17g}" for angle in args.absolute_targets_deg)
        if spec is None and not args.single_joint:
            if args.profile == "range":
                spec, skipped = range_cases(args.groups, baseline, limits, args.points_per_joint, args.edge_margin_deg)
            else:
                spec, skipped = default_cases(args.groups, baseline, limits)
        requested = parse_cases(spec, args.groups, limits, baseline)
        if args.trajectory == "sweep" and args.joint is None and not args.single_joint:
            requested = sweep_order(requested, baseline)
        coverage = {}
        motion_seconds = 0.0
        previous = baseline.copy()
        for number, case in enumerate(requested):
            axis = case["axis"]
            if axis not in coverage:
                lo, hi = limits[AXES[axis][1]]
                coverage[axis] = {"lower_limit_deg": math.degrees(lo), "upper_limit_deg": math.degrees(hi),
                                  "planned_targets_deg": []}
            coverage[axis]["planned_targets_deg"].append(case["target_deg"])
            index = case["motor_index"]
            distance = abs(case["target_deg"] - math.degrees(previous[index]))
            motion_seconds += max(args.move_duration, 1.5 * distance / args.max_speed_deg_s) + args.settle_duration
            previous[index] = math.radians(case["target_deg"])
            return_now = not args.single_joint and (args.trajectory == "per-point" or number + 1 == len(requested)
                                                    or requested[number + 1]["axis"] != axis)
            if return_now:
                motion_seconds += max(args.move_duration, 1.5 * abs(case["offset_deg"]) / args.max_speed_deg_s) + 1.0
                previous = baseline.copy()
        for details in coverage.values():
            targets = details["planned_targets_deg"]
            width = details["upper_limit_deg"] - details["lower_limit_deg"]
            details["planned_coverage_percent"] = 100 * (max(targets) - min(targets)) / width if width > 0 else 100.0
        use_heft = args.single_joint or args.heft_for_head or any(c["axis"] not in GROUPS["head"] for c in requested)
        if args.execute and not sys.stdin.isatty():
            raise RuntimeError("执行测试需要交互终端（远程请 ssh -t）")
        live = load_config(args.config) if args.execute and use_heft else {"topic": "/rt/pico/heft_reference"}
        if args.execute:
            print("只读采集当前关节反馈，不创建指令发布器……", flush=True)
            io = TestIo(live["topic"], args.max_state_age)
            if args.execute and use_heft and hasattr(io, "participant"):
                try:
                    from dds_heft_controller_observer import ControllerStateObserver
                    observer = ControllerStateObserver(io.participant)
                    observer.start()
                except Exception as exc:
                    observer = None
                    report["controller_observer_error"] = str(exc)
            snapshot = capture_baseline(io, args)
            if args.snapshot is not None:
                write_json(args.snapshot, snapshot)
                print(f"启动时实测姿态另存：{args.snapshot}")
            print(f"启动时只读检查完成，{snapshot['sample_count']} 帧 / {snapshot['span_sec']:.2f}s；执行使用已记录的初始姿态，不覆盖记录文件。")
        cases = requested
        report.update(cases=cases, skipped_cases=skipped, baseline=saved, current_snapshot=snapshot,
                      baseline_path=str(args.baseline), heft_used=args.execute and use_heft,
                      heft_used_meaning="HEFT reference requested; active controller evidence is recorded per result",
                      controller_status_continuously_verified=False,
                      hardware_config=str(args.hardware_config), json_path=str(args.json), csv_path=str(args.csv), report_path=str(args.report),
                      commands_path=str(args.commands), coverage=coverage, estimated_motion_duration_s=motion_seconds,
                      profile="single_joint_absolute" if args.joint is not None else "custom" if args.cases is not None else args.profile,
                      requested_absolute_targets_deg=args.absolute_targets_deg,
                      trajectory=args.trajectory, max_speed_deg_s=args.max_speed_deg_s,
                      move_duration_s=args.move_duration, settle_duration_s=args.settle_duration)
        if args.single_joint:
            report["trajectory"] = "hold_last"
            report["trajectory_description"] = "- 从已记录的初始姿态开始，连续输入单关节绝对角；测后保持目标，后续测试保留此前参考角，不自动回位。"
        elif args.joint is not None:
            report["tested_joint"] = args.joint
            report["trajectory_description"] = (
                "- 按操作者指定的硬件绝对角顺序测试单个关节，每个目标后返回保存的基准。" if args.trajectory == "per-point" else
                "- 按操作者指定的硬件绝对角顺序测试单个关节，全部目标测完后返回一次保存的基准。")
        if snapshot is not None:
            report["initial_pose_error_deg"] = [math.degrees(a - b) for a, b in zip(snapshot["motor_q_rad"], baseline)]
        print(f"参考基准：{args.baseline}")
        print("初始硬件参考角 [deg]（连续单关节模式测后保持目标，不自动回基准）：" if args.single_joint else
              "保存的硬件关节角 [deg]（每关节扫描完成后回到这里，不回零）：")
        for i, name in enumerate(MOTOR_NAMES):
            print(f"  q[{i:2}] {name:18} {math.degrees(baseline[i]):+9.3f}")
        for case in cases:
            if case["target_deg"] is None:
                print(f"计划：{case['axis']} 相对实测初姿 {case['offset_deg']:+.3f}deg（尚无快照，未计算目标）")
            else:
                start = math.degrees(baseline[case["motor_index"]])
                print(f"计划：{case['axis']} {start:+.3f} -> {case['target_deg']:+.3f}deg，偏移 {case['offset_deg']:+.3f}deg")
        for entry in skipped:
            print(f"跳过限位附近用例：{entry}")
        for axis, details in coverage.items():
            targets = details["planned_targets_deg"]
            print(f"覆盖：{axis} 限位[{details['lower_limit_deg']:+.2f},{details['upper_limit_deg']:+.2f}]deg，"
                  f"目标[{min(targets):+.2f},{max(targets):+.2f}]deg，{len(targets)}点，跨度覆盖{details['planned_coverage_percent']:.1f}%")
        print(f"连续单关节会话；预设 {len(cases)} 个目标，后续从终端输入，测完保持目标。" if args.single_joint else
              f"计划 {len(cases)} 项，按参考速度配置估算 {motion_seconds/60:.1f} 分钟（不含人工等待）。")
        print(f"结果 JSON：{args.json}\n反馈 CSV：{args.csv}\n实际发布记录 JSONL：{args.commands}")
        if not args.execute:
            report["status"] = "dry_run" if args.dry_run else "captured_read_only"
            print("DRY RUN：未连接 DDS。" if args.dry_run else "只读采集完成。")
            print("没有发布控制指令。加 --execute 即按上述默认计划测试，不需要另填路径或目标。")
            return 0
        height, root_quat = None, None
        if use_heft:
            io.read()
            io.require_fresh()
            if args.root_quat_wxyz is None:
                io.require_imu()
                root_quat = root_quat_from_imu(io.last_imu.quat, io.last.q[12])
            else:
                root_quat = list(args.root_quat_wxyz)
            height_info = estimate_root_height(args.urdf, baseline, root_quat)
            height = args.root_height if args.root_height is not None else height_info["height_m"]
            if not math.isfinite(height) or height <= 0:
                raise ValueError("根高度必须是有限正数")
            report.update(root_height_m=height, root_quat_wxyz=root_quat,
                          root_height_source="explicit" if args.root_height is not None else "urdf_contact_geometry_estimate",
                          root_orientation_source="explicit" if args.root_quat_wxyz is not None else "torso_imu_and_hardware_waist_model",
                          root_pose_measured=False, root_geometry=height_info)
            print(f"根高度={height:.4f}m（{'显式指定' if args.root_height is not None else '模型几何估计，非实测高度'}）；根姿态={root_quat}")
            print(f"记录模型正常运动；不按倾角、腿部偏移或跟踪误差中止。基准误差与{len(cases)}项结果全部写入报告。")
            input("核对计划。按 Enter 开始发布基准参考，或 Ctrl+C 取消：")
        else:
            input("本次只走独立头部通道，不发 HEFT 参考、不需要按 B。审阅目标后按 Enter，或 Ctrl+C 取消：")
        # Refresh the measurement after a potentially long operator pause.
        io.read()
        io.require_fresh()
        guard = LimitGuard(limits)
        guard.check(io, baseline)
        io.enable_writers(use_heft, root_quat)
        if args.csv:
            args.csv.parent.mkdir(parents=True, exist_ok=True)
            csv_file = args.csv.open("w", newline="")
        args.commands.parent.mkdir(parents=True, exist_ok=True)
        commands_file = args.commands.open("w", buffering=1)
        runner = Runner(io, args, height, csv_file, guard=guard, commands_file=commands_file)
        if use_heft:
            runner.hold(baseline, 1.0, "warmup")
            runner.prompt(baseline, "参考流已预热。按 B 并确认 heft_pico 已激活、姿态稳定，再按 Enter；异常立即使用急停。")
            report["operator_heft_confirmed_at"] = time.time()
        initial_samples = runner.hold(baseline, args.settle_duration, "baseline_tracking_check")
        initial_checks = []
        axes_to_check = AXES if use_heft else {key: AXES[key] for key in GROUPS["head"]}
        for axis, (index, _) in axes_to_check.items():
            initial = {"axis": axis, "motor_index": index, "target_deg": math.degrees(baseline[index])}
            initial_checks.append(window_stats(initial_samples, baseline, initial, args))
        report["baseline_tracking"] = initial_checks
        save()
        if not all(check["pass"] for check in initial_checks):
            failed = [f"{r['axis']}:{r['error_deg']:+.3f}deg" for r in initial_checks if not r["pass"]]
            print("基准精度未达阈值，已记录，继续全项测试：" + ", ".join(failed), flush=True)
        else:
            print("基准跟踪检查通过，开始逐关节测试。", flush=True)
        report["status"] = "running"
        save()
        if args.single_joint:
            return run_joint_session(runner, args, baseline, limits, observer, report, save, live, cases)
        command_start = baseline.copy()
        for n, case in enumerate(cases, 1):
            target = baseline.copy()
            target[case["motor_index"]] = math.radians(case["target_deg"])
            message = f"[{n}/{len(cases)}] 仅 {case['axis']} 相对基准 {case['offset_deg']:+g}deg，目标 {case['target_deg']:+.3f}deg"
            print_test_start(n, case, math.degrees(command_start[case["motor_index"]]))
            if args.step_confirm:
                runner.prompt(command_start, message + "；按 Enter 执行本条，Ctrl+C 停止发布。")
            label = f"{n}:{case['axis']}:{case['target_deg']:+.6f}"
            runner.drive(command_start, target, args.move_duration, label + ":move")
            hold_started_at = time.time()
            samples = runner.hold(target, args.settle_duration, label + ":hold")
            published_deg = math.degrees(runner.command[case["motor_index"]])
            published_case = {**case, "target_deg": published_deg}
            result = window_stats(samples, runner.command, published_case, args)
            publication = runner.last_publication if isinstance(runner.last_publication, dict) else {}
            index = case["motor_index"]
            wire_index = index - 21 if index >= 21 else HEFT_TO_MOTOR.index(index)
            result.update(planned_target_deg=case["target_deg"], published_deg=published_deg,
                          published_wire_deg=-published_deg if index == 12 else published_deg,
                          published_topic="/rt/head_trajectory" if index >= 21 else live["topic"],
                          published_q_index=wire_index, publication_seq=publication.get("seq"),
                          publication_stamp_ns=publication.get("stamp_ns"),
                          abs_error_deg=abs(result["error_deg"]))
            result.update(controller_evidence(observer, hold_started_at, time.time(), index < 21),
                          trajectory=args.trajectory)
            start = math.degrees(baseline[case["motor_index"]])
            moved = result["actual_deg"] - start
            delta = case["offset_deg"]
            result.update(start_deg=start, no_response=abs(delta) > 0.5 and abs(moved) < 0.05 * abs(delta),
                          sign_suspect=abs(delta) > 0.5 and delta * moved < 0 and abs(moved) > 0.5 * abs(delta))
            result["pass"] = result["pass"] and not result["no_response"] and not result["sign_suspect"]
            report["results"].append(result)
            save()
            print_test_result(n, result)
            return_now = (args.trajectory == "per-point" or n == len(cases)
                          or cases[n]["axis"] != case["axis"])
            if return_now:
                runner.drive(target, baseline, args.move_duration, label + ":return")
                runner.hold(baseline, 1.0, label + ":baseline_settle")
                command_start = baseline.copy()
            else:
                command_start = target.copy()
            if not result["pass"]:
                print("本条精度未通过，已记录，继续下一项。", flush=True)
        passed = len(report["results"]) == len(cases) and all(r["pass"] for r in report["results"])
        report.update(status="completed" if passed else "accuracy_failed", passed=passed)
        heft_results = [r for r in report["results"] if r["motor_index"] < 21]
        observed = sum(r["controller_observation_status"] == "observed_heft" for r in heft_results)
        report.update(heft_points_with_state_evidence=observed, heft_points_total=len(heft_results),
                      all_heft_points_observed=observed == len(heft_results))
        save()
        print(f"汇总：{sum(r['pass'] for r in report['results'])}/{len(cases)} 通过；报告={args.report}；JSON={args.json}")
        if heft_results:
            print(f"保持阶段观测到 heft_pico：{observed}/{len(heft_results)} 项；其余项不能证实为 HEFT 下的误差，详见报告。")
        runner.prompt(baseline, "完成。先按 X 并确认回到 AMP，再按 Enter 停止参考发布。" if use_heft else "完成，保持头部初姿。按 Enter 停止发布头部参考。")
        return 0 if report["passed"] else 1
    except KeyboardInterrupt:
        if report["status"] not in ("completed", "accuracy_failed"):
            report["status"] = "interrupted"
        print("\n已停止；不追加回位动作。若已发布 HEFT，失流处理由控制器决定，中断不能代替急停。", file=sys.stderr)
        return 130
    except Exception as exc:
        report.update(status="error", error=str(exc))
        print(f"error: {exc}\n本次运行结束，已保存结果。若已切到 HEFT，请现场按 X 回到 AMP。", file=sys.stderr)
        return 2
    finally:
        if observer is not None:
            observer.stop()
            report.update(controller_observer_error=observer.error,
                          controller_last_state=observer.snapshot(),
                          controller_queries=observer.requests())
        if csv_file:
            csv_file.close()
        if commands_file:
            commands_file.close()
        report["finished_at"] = time.time()
        save()


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    raise SystemExit(main())
