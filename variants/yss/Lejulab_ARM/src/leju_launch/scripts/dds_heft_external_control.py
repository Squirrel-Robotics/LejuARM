#!/usr/bin/env python3
"""Roban S17: capture a fresh stance, warm HEFT, then accept 21 joint targets.

The array order is always waist, left leg 1..6, right leg 1..6,
left arm 1..4, right arm 1..4. Hardware coordinates are the default;
HEFT coordinates invert the waist. No head commands or mode switches are sent.
"""
from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import select
import socket
import sys
import time

HERE = Path(__file__).resolve().parent
if (HERE / "reference").is_dir():
    sys.path.insert(0, str(HERE / "reference"))
ROOT = Path(__file__).resolve().parents[3]
NAMES = (["waist_yaw_joint"]
         + [f"leg_{s}{i}_joint" for s in ("l", "r") for i in range(1, 7)]
         + [f"zarm_{s}{i}_joint" for s in ("l", "r") for i in range(1, 5)])
MOTOR_INDEX = [12] + list(range(12)) + list(range(13, 21))


def udp_only_uri(uri):
    """Keep the chosen network settings; disable SHM for this process only."""
    import xml.etree.ElementTree as ET
    from urllib.parse import urlparse, unquote
    if uri.startswith("file://"):
        location = urlparse(uri)
        if location.netloc not in ("", "localhost"):
            raise ValueError("DDS配置仅支持本机file URI")
        root = ET.parse(unquote(location.path)).getroot()
    elif uri.lstrip().startswith("<"):
        root = ET.fromstring(uri)
    else:
        raise ValueError("DDS配置必须为本机file URI或XML")
    local = lambda tag: tag.rsplit("}", 1)[-1]
    if local(root.tag) != "CycloneDDS":
        raise ValueError("DDS配置根节点必须为CycloneDDS")
    namespace = root.tag.split("}", 1)[0] + "}" if root.tag.startswith("{") else ""
    if namespace:
        ET.register_namespace("", namespace[1:-1])
    domains = [node for node in root.iter() if local(node.tag) == "Domain"]
    if not domains:
        raise ValueError("DDS配置缺少Domain")
    for domain in domains:
        shm = next((node for node in domain if local(node.tag) == "SharedMemory"), None)
        if shm is None:
            shm = ET.SubElement(domain, namespace + "SharedMemory")
        enabled = next((node for node in shm if local(node.tag) == "Enable"), None)
        if enabled is None:
            enabled = ET.SubElement(shm, namespace + "Enable")
        enabled.text = "false"
    return ET.tostring(root, encoding="unicode")


def vector(value):
    if (not isinstance(value, list) or len(value) != 21
            or any(isinstance(x, bool) or not isinstance(x, (int, float))
                   or not math.isfinite(x) for x in value)):
        raise ValueError("q 必须是恰好 21 个有限数值的数组")
    return [float(x) for x in value]


def hardware_order(motor):
    return [motor[i] for i in MOTOR_INDEX]


def wire_order(hardware):
    return [-hardware[0]] + hardware[1:]


def check_limits(q, limits, slack=0.0):
    vector(q)
    for name, value in zip(NAMES, q):
        lo, hi = limits[name]
        if not lo - slack <= value <= hi + slack:
            raise ValueError(f"{name}: {math.degrees(value):+.3f}° 超出 "
                             f"[{math.degrees(lo):+.3f}, {math.degrees(hi):+.3f}]°")


def decode_command(data, limits):
    if not isinstance(data, dict):
        raise ValueError("命令必须是 JSON object，包含 q、units、coordinates")
    q = vector(data.get("q"))
    units, coords = data.get("units"), data.get("coordinates")
    if units not in ("deg", "rad") or coords not in ("hardware", "heft"):
        raise ValueError("必须明确 units=deg/rad、coordinates=hardware/heft")
    if units == "deg":
        q = [math.radians(x) for x in q]
    if coords == "heft":
        q = wire_order(q)  # The waist conversion is its own inverse.
    check_limits(q, limits)
    return q


def slew(current, target, max_step, dt):
    distance = max(abs(b - a) for a, b in zip(current, target))
    weight = min(1.0, max_step / distance) if distance else 1.0
    q = [a + weight * (b - a) for a, b in zip(current, target)]
    return q, [(b - a) / dt for a, b in zip(current, q)]


def fresh_controller(observer, age=1.0):
    state = observer.snapshot()
    if (state is None or time.monotonic() - state["received_monotonic"] > age
            or observer.error is not None):
        return None
    return state


def active(state):
    return (state is not None and state["current_controller"] == "heft_pico"
            and not state["controller_transitioning"])


class PublicationGuard:
    """Refuse a second discovered writer on the same live reference topic."""
    def __init__(self, participant, topic):
        from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsPublication
        self.reader = BuiltinDataReader(participant, BuiltinTopicDcpsPublication)
        self.topic, self.own_guid, self.endpoints = topic, None, {}

    def check(self):
        from cyclonedds.core import InstanceState
        for endpoint in self.reader.take(N=1000):
            if endpoint.sample_info.instance_state != InstanceState.Alive:
                self.endpoints.pop(endpoint.key, None)
            elif endpoint.topic_name == self.topic:
                self.endpoints[endpoint.key] = endpoint
        others = [str(key) for key in self.endpoints if key != self.own_guid]
        if others:
            raise RuntimeError("发现另一个HEFT参考发布器，拒绝并行控制。请关闭Pico/GMR或其他外控发布器：" + ", ".join(others))


class UdpInput:
    """Loopback-only, one producer per run, timestamps + monotonic sequences."""
    def __init__(self, port, max_age):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))
        self.sock.setblocking(False)
        self.max_age, self.peer, self.seq, self.stamp = max_age, None, -1, -1
        self.last_warning = 0.0

    def warn(self, message):
        if time.monotonic() - self.last_warning > 1:
            print("拒绝 UDP 命令：" + str(message), flush=True)
            self.last_warning = time.monotonic()

    def poll(self, limits, enabled):
        latest = None
        # Bounded work per control tick. Discard commands received before B.
        for _ in range(256):
            try:
                raw, peer = self.sock.recvfrom(65536)
            except BlockingIOError:
                break
            if not enabled:
                continue
            try:
                data = json.loads(raw)
                q = decode_command(data, limits)
                seq, stamp = data.get("seq"), data.get("sent_at_ns")
                if (type(seq) is not int or seq < 0 or type(stamp) is not int
                        or not -0.05 <= time.time() - stamp / 1e9 <= self.max_age):
                    raise ValueError("seq / sent_at_ns 缺失、无效或命令过期")
                if self.peer is not None and peer != self.peer:
                    raise ValueError("本次会话已经绑定另一 UDP 发送端；重启控制脚本才能换端")
                if seq <= self.seq or stamp <= self.stamp:
                    raise ValueError("乱序或重复帧")
                self.peer, self.seq, self.stamp = peer, seq, stamp
                # Expiry is based on sender time, so queueing cannot renew it.
                latest = (q, time.monotonic() + max(0, self.max_age - (time.time() - stamp / 1e9)))
            except (ValueError, TypeError, OverflowError) as exc:
                self.warn(exc)
        return latest

    def close(self):
        self.sock.close()


class HeftOutput:
    def __init__(self, io, topic, root_pos, root_quat, log):
        from cyclonedds.pub import DataWriter
        from cyclonedds.topic import Topic
        from cyclonedds.qos import Qos, Policy
        from cyclonedds.util import duration
        # Volatile latest-frame delivery; never retain an old command for replay.
        qos = Qos(Policy.History.KeepLast(1), Policy.Durability.Volatile,
                  Policy.Reliability.Reliable(max_blocking_time=duration(milliseconds=5)))
        self.writer = DataWriter(io.participant, Topic(io.participant, topic, io.HeftReference), qos=qos)
        self.io, self.root_pos, self.root_quat, self.log = io, root_pos, root_quat, log

    def publish(self, q, qd, phase):
        ns = time.time_ns()
        sec, nsec = divmod(ns, 1_000_000_000)
        self.io.seq += 1
        fields = dict(header_sec=sec, header_nanosec=nsec, schema_version=1,
                      robot_version="roban_s17", seq=self.io.seq,
                      calibration_epoch=self.io.epoch, capture_timestamp_ms=ns // 1_000_000,
                      valid=True, calibrated=True, root_pos=self.root_pos, root_quat=self.root_quat,
                      q=wire_order(q), qd=wire_order(qd))
        self.writer.write(self.io.HeftReference(**fields))
        self.log.write(json.dumps({"phase": phase, "hardware_q_rad": q,
                                   "actual_hardware_q_rad": hardware_order(self.io.last.q),
                                   **fields}, allow_nan=False) + "\n")


def terminal(line, command, limits):
    parts = line.strip().split()
    if not parts:
        return None
    if parts[0] == "set" and len(parts) == 3:
        name = parts[1]
        if name not in NAMES:
            raise ValueError("关节名须使用 status 显示的完整名称")
        q = command.copy()
        q[NAMES.index(name)] = math.radians(float(parts[2]))
        check_limits(q, limits)
        return q
    if parts[0] in ("deg", "rad"):
        return decode_command({"q": [float(x) for x in parts[1:]],
                               "units": parts[0], "coordinates": "hardware"}, limits)
    if line.lstrip().startswith("{"):
        return decode_command(json.loads(line), limits)
    raise ValueError("命令：status | hold | set 关节名 绝对角度 | deg/rad 后跟21个值 | JSON | exit")


def display(q):
    for i, (name, value) in enumerate(zip(NAMES, q)):
        print(f"  [{i:2}] {name:18} {math.degrees(value):+9.3f}°", flush=True)


def prepare_reference_source(args, io, observer, publications):
    """AMP-only handover from the verified optional Pico frontend, before capture."""
    from dds_heft_pico_guard import stop_optional_pico

    args.run_status["stage"] = "checking_optional_pico"
    deadline = time.monotonic() + args.state_timeout
    while fresh_controller(observer, age=0.75) is None:
        if time.monotonic() >= deadline:
            raise RuntimeError("无法确认AMP状态；没有停止Pico/GMR，请检查主控并START站立")
        io.read()
        time.sleep(0.05)

    def ensure_amp():
        state = fresh_controller(observer, age=0.75)
        if state is None or state["current_controller"] != "amp" or state["controller_transitioning"]:
            raise RuntimeError("停止Pico和预热前必须为AMP稳态；请先按X回AMP并站稳，再运行脚本")
        if "heft_pico" not in state["available_controllers"]:
            raise RuntimeError("实机未提供 heft_pico 控制器")
        io.read()
        io.require_fresh()
        io.require_imu()
        return state

    ensure_amp()
    args.run_status["pico_frontend_stop"] = []
    stopped = stop_optional_pico(ROOT, ensure_amp, args.run_status["pico_frontend_stop"])
    if stopped:
        print("Pico/GMR进程已退出，等待旧DDS端点注销（最多20秒）……", flush=True)
        deadline = time.monotonic() + 20.0
        while True:
            ensure_amp()
            try:
                publications.check()
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.05)
        print("Pico/GMR已停止，已检查DDS发布冲突；重新采集当前站姿。", flush=True)
    args.run_status["stage"] = "capturing"


def capture_or_run(args):
    from dds_heft_pose_test import TestIo, capture_baseline, load_config, write_json
    from dds_heft_pose_guard import load_combined_limits
    from dds_heft_pose_kinematics import estimate_root_height, root_quat_from_imu
    from dds_heft_controller_observer import ControllerStateObserver

    if args.execute and not args.no_console and not sys.stdin.isatty():
        raise RuntimeError("交互运行需要终端，远程请 ssh -t；后台 UDP 服务可显式加 --no-console")
    limits = load_combined_limits(args.urdf, args.hardware_config)
    live = load_config(args.config)
    # One instance of this publisher per robot. Capture has no lock or writer.
    lock = None
    if args.execute:
        lock = open("/tmp/roban_heft_external_21.lock", "a")
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            lock.close()
            raise RuntimeError("已有一个21关节外控脚本运行") from None
    observer, udp = None, None
    try:
        io = TestIo(live["topic"], args.max_state_age)
        publications = PublicationGuard(io.participant, live["topic"]) if args.execute else None
        observer = ControllerStateObserver(io.participant, poll_interval=0.25).start()
        if args.execute:
            print("只读检查AMP状态和可选Pico/GMR；尚未创建运动发布器……", flush=True)
            prepare_reference_source(args, io, observer, publications)
        print("只读采集当前站姿；尚未创建运动发布器……", flush=True)
        baseline = capture_baseline(io, args)
        q = hardware_order(baseline["motor_q_rad"])
        check_limits(q, limits)
        io.read()
        io.require_fresh()
        io.require_imu()
        root_quat = root_quat_from_imu(io.last_imu.quat, io.last.q[12])
        geometry = estimate_root_height(args.urdf, baseline["motor_q_rad"], root_quat)
        height = args.root_height if args.root_height is not None else geometry["height_m"]
        if not math.isfinite(height) or height <= 0:
            raise ValueError("根高度必须为正数")
        state = fresh_controller(observer)
        if args.execute:
            if state is None or state["current_controller"] != "amp" or state["controller_transitioning"]:
                raise RuntimeError("开始预热前必须由只读 RPC 确认 amp 稳态；请先启动并 START 站立")
            if "heft_pico" not in state["available_controllers"]:
                raise RuntimeError("实机未提供 heft_pico 控制器")
            if abs(geometry["left_height_m"] - geometry["right_height_m"]) > 0.04:
                raise RuntimeError("双足估计高度相差超过4cm；不能用本脚本的双足落地根高度假设")
            if max(baseline["joint_range_deg"][:21]) > 3:
                raise RuntimeError("采集时身体关节变化超过3°，请站稳后重新运行")
            publications.check()
        args.output_dir.mkdir(parents=True, exist_ok=True)
        write_json(args.output_dir / "snapshot.json", {
            **baseline, "joint_order_21": NAMES, "hardware_q21_rad": q,
            "heft_q21_rad": wire_order(q), "root_pos": [0, 0, height],
            "root_quat_wxyz": root_quat, "root_geometry": geometry,
            "root_pose_measured": False, "controller_observation": state,
        })
        write_json(args.output_dir / "target.json", {
            "q": [math.degrees(x) for x in q], "units": "deg", "coordinates": "hardware"})
        display(q)
        print(f"记录：{args.output_dir}/snapshot.json\n目标模板：{args.output_dir}/target.json\n"
              f"根高度 {height:.4f}m（模型估计），参考根朝向 {root_quat}", flush=True)
        if not args.execute:
            print("只读完成；没有创建运动发布器。加 --execute 才会预热参考流。", flush=True)
            return 0
        udp = UdpInput(args.udp_port, args.command_timeout)
        with (args.output_dir / "commands.jsonl").open("w", buffering=1) as log:
            output = HeftOutput(io, live["topic"], [0.0, 0.0, height], root_quat, log)
            publications.own_guid = output.writer.get_guid()
            return run_loop(args, io, observer, udp, output, q, limits, live, publications)
    finally:
        if observer:
            observer.stop()
        if udp:
            udp.close()
        if lock:
            lock.close()


def run_loop(args, io, observer, udp, output, initial, limits, live, publications=None):
    diagnostics = getattr(args, "run_status", {})
    args.run_status = diagnostics
    diagnostics.update(stage="reference_loop", published_frames=0, max_cycle_gap_s=0.0,
                       reference_started_at=time.time())
    command, target = initial.copy(), initial.copy()
    tick = 1.0 / args.rate
    start = previous = time.monotonic()
    next_tick, ready_since, entered = start, None, False
    deadline, udp_control, unknown_since = None, False, None
    announced = False
    print(f"开始以 {args.rate:g}Hz 发布当前站姿；预热1秒后提示按B。", flush=True)
    while True:
        now = time.monotonic()
        gap = now - previous
        previous = now
        diagnostics["max_cycle_gap_s"] = max(diagnostics["max_cycle_gap_s"], gap)
        # Receiver-side fresh_timeout/hold_timeout/fallback remain authoritative.
        # A local scheduling gap alone must not permanently stop publication.
        io.read()
        io.require_fresh()
        if publications is not None:
            publications.check()
        check_limits(hardware_order(io.last.q), limits, slack=math.radians(0.1))
        state = fresh_controller(observer)
        diagnostics["last_controller_observation"] = state
        if state is None:
            unknown_since = now if unknown_since is None else unknown_since
            target, deadline, udp_control = command.copy(), None, False
            if now - unknown_since > 1.0:
                raise RuntimeError("控制器状态 RPC 失联；停止参考发布")
        else:
            unknown_since = None
        if entered and state is not None and not active(state):
            diagnostics["exit_reason"] = "controller_left_heft_pico_or_transitioning"
            print("已观测到离开 heft_pico，停止参考发布。", flush=True)
            return 0
        if not entered and state is not None and state["current_controller"] not in ("amp", "heft_pico"):
            raise RuntimeError("预热阶段切到了非 amp/heft_pico 控制器")
        if now - start >= 1 and not announced:
            print("参考流已预热。现在可单按手柄 B；脚本等待 RPC 确认 heft_pico。", flush=True)
            announced = True
        was_entered = entered
        if not entered and active(state) and announced:
            ready_since = now if ready_since is None else ready_since
            if now - ready_since >= 0.5:
                entered = True
                diagnostics["heft_confirmed_at"] = time.time()
                print("已确认 heft_pico：可输入21角度或发送UDP。status 显示关节；"
                      "set zarm_l1_joint 5 设置绝对角5°。先按X退出，脚本会自动停止。", flush=True)
        elif not entered:
            ready_since = None
        enabled = entered and active(state)
        # Drain everything queued before activation, including this transition tick.
        incoming = udp.poll(limits, enabled and was_entered)
        if incoming is not None:
            target, deadline = incoming
            udp_control = True
        if not args.no_console and select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if not line:
                raise RuntimeError("交互终端断开")
            line = line.strip()
            if line == "exit":
                if entered:
                    print("请先按手柄X；确认离开heft_pico后脚本自动停止。Ctrl+C可直接中断参考流。", flush=True)
                else:
                    diagnostics["exit_reason"] = "operator_exit_before_B"
                    return 0
            elif line == "status":
                print("当前控制器：", state, "\n已发送硬件参考角：", flush=True)
                display(command)
            elif line == "hold":
                target, deadline, udp_control = command.copy(), None, False
                print("保持最近已发送的参考角。", flush=True)
            elif line:
                try:
                    if not enabled or not was_entered:
                        raise ValueError("尚未确认heft_pico，切换前的目标被丢弃")
                    updated = terminal(line, command, limits)
                    if updated is not None:
                        target, deadline, udp_control = updated, None, False
                except (ValueError, TypeError, OverflowError) as exc:
                    print("拒绝终端命令：", exc, flush=True)
        if udp_control and deadline is not None and now > deadline:
            target, deadline, udp_control = command.copy(), None, False
            print("UDP输入超时：保持最近已发参考角，取消尚未完成的目标。", flush=True)
        if not enabled:
            target = command.copy()
        # Nominal tick bound also prevents catching up after a delayed scheduler.
        dt = min(tick, max(0.0001, gap))
        command, velocity = slew(command, target, math.radians(args.max_speed_deg_s) * dt, dt)
        check_limits(command, limits)
        output.publish(command, velocity, "external" if enabled else "warmup_or_hold")
        diagnostics["published_frames"] += 1
        diagnostics["last_reference_at"] = time.time()
        next_tick += tick
        if next_tick < time.monotonic():
            next_tick = time.monotonic()
        time.sleep(max(0, next_tick - time.monotonic()))


def send(args):
    """Relay a changing file or JSONL stdin over local UDP; no DDS writer."""
    sock, seq = socket.socket(socket.AF_INET, socket.SOCK_DGRAM), 0
    try:
        while True:
            if args.send_stdin:
                line = sys.stdin.readline(65537)
                if not line:
                    return
                if len(line) > 65536:
                    raise ValueError("单条命令超过64KiB")
                data = json.loads(line)
            else:
                data = json.loads(args.file.read_text())
            if not isinstance(data, dict):
                raise ValueError("命令必须为JSON object")
            vector(data.get("q"))
            if data.get("units") not in ("deg", "rad") or data.get("coordinates") not in ("hardware", "heft"):
                raise ValueError("目标文件必须显式指定 units 和 coordinates")
            seq += 1
            packet = {**data, "seq": seq, "sent_at_ns": time.time_ns()}
            sock.sendto(json.dumps(packet, allow_nan=False).encode(), ("127.0.0.1", args.udp_port))
            if seq == 1:
                print("正在转发21关节目标；Ctrl+C停止。", flush=True)
            if not args.send_stdin:
                time.sleep(1.0 / args.rate)
    finally:
        sock.close()


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--execute", action="store_true", help="AMP下停止本工程可选Pico/GMR，采集当前站姿，再预热并等待手柄B；默认仅采集")
    p.add_argument("--send", action="store_true", help="持续读取 --file 并发送本机UDP；不连接DDS")
    p.add_argument("--send-stdin", action="store_true", help="将标准输入的每行JSON转发到本机UDP，可经SSH接外部模型")
    p.add_argument("--file", type=Path, help="与 --send 同用；含q[21]、units、coordinates的JSON")
    p.add_argument("--no-console", action="store_true", help="仅用UDP输入，适于后台运行")
    p.add_argument("--udp-port", type=int, default=28721, help="仅绑定127.0.0.1")
    p.add_argument("--rate", type=float, default=100.0, help="DDS参考/UDP发送频率Hz，50..200")
    p.add_argument("--max-speed-deg-s", type=float, default=10.0, help="每关节参考变化速度上限")
    p.add_argument("--command-timeout", type=float, default=0.5, help="UDP命令有效期秒；失流保持最近已发参考")
    p.add_argument("--max-state-age", type=float, default=0.25)
    p.add_argument("--state-timeout", type=float, default=8.0)
    p.add_argument("--capture-duration", type=float, default=2.0)
    p.add_argument("--root-height", type=float, help="显式根高度m；默认由双足几何估计")
    p.add_argument("--config", type=Path, default=ROOT / "src/leju-controllers/leju-rl-controller/config/17/controllers/config_heft.yaml")
    p.add_argument("--urdf", type=Path, default=ROOT / "src/leju_assets/models/biped_s17/urdf/biped_s17.urdf")
    p.add_argument("--hardware-config", type=Path, default=ROOT / "src/leju-hardware/config/roban_v17/kuavo.json")
    p.add_argument("--output-dir", type=Path, default=None)
    a = p.parse_args(argv)
    if sum((a.execute, a.send, a.send_stdin)) > 1:
        p.error("--execute、--send、--send-stdin 不能同用")
    if a.send != (a.file is not None):
        p.error("--send 与 --file 必须同用")
    for field in ("rate", "max_speed_deg_s", "command_timeout", "max_state_age", "state_timeout", "capture_duration"):
        if not math.isfinite(getattr(a, field)) or getattr(a, field) <= 0:
            p.error(field + " 必须为有限正数")
    if not 50 <= a.rate <= 200 or not 1 <= a.udp_port <= 65535:
        p.error("rate须为50..200Hz，udp-port须为1..65535")
    if a.root_height is not None and (not math.isfinite(a.root_height) or a.root_height <= 0):
        p.error("root-height必须为有限正数")
    if a.output_dir is None:
        # Unique run directories preserve earlier poses and command logs.
        a.output_dir = ROOT / "validation/heft_external_21" / (time.strftime("%Y%m%d_%H%M%S") + f"_{os.getpid()}")
    return a


def main(argv=None):
    args = parse_args(argv)
    args.run_status = {"started_at": time.time(), "stage": "preparing", "rate_hz": args.rate,
                       "motion_reference_requested": args.execute}
    sender_only = args.send or args.send_stdin
    try:
        if sender_only:
            send(args)
            return 0
        result = capture_or_run(args)
        args.run_status.update(status="finished", exit_code=result)
        return result
    except KeyboardInterrupt:
        args.run_status.update(status="interrupted", exit_code=130, exit_reason="KeyboardInterrupt")
        print("\n已中断发送，没有追加回位动作。若仍在HEFT，请现场按X回AMP。", file=sys.stderr)
        return 130
    except Exception as exc:
        args.run_status.update(status="error", exit_code=1, exit_reason=str(exc), exception_type=type(exc).__name__)
        print(f"错误：{exc}\n参考发送已停止；若仍在HEFT，请现场按X回AMP。", file=sys.stderr)
        return 1
    finally:
        if not sender_only:
            args.run_status["ended_at"] = time.time()
            path = args.output_dir / "session_status.json"
            try:
                path.parent.mkdir(parents=True, exist_ok=True)
                temporary = path.with_suffix(".json.tmp")
                temporary.write_text(json.dumps(args.run_status, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
                temporary.replace(path)
                print(f"退出原因与发布统计：{path}", flush=True)
            except OSError as exc:
                print(f"退出记录保存失败：{exc}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
