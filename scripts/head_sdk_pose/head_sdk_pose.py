#!/usr/bin/env python3
"""Roban S17旧SDK头部控制：平滑调整yaw/pitch，省略yaw时保持当前值；默认只读。"""
import argparse
from dataclasses import dataclass
import fcntl
import json
import math
import os
from pathlib import Path
import select
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from urllib.parse import unquote, urlparse

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "reference"))
RATE = 50
MAX_AGE = .25
MAX_SPEED = math.radians(8)
# Match scripts/traj_tracking_common.hpp::specFor(Part::Head).
YAW_LIMIT_DEG = (-90., 90.)
PITCH_LIMIT_DEG = (-20., 45.)
HEAD_LOWER_RAD = tuple(map(math.radians, (YAW_LIMIT_DEG[0], PITCH_LIMIT_DEG[0])))
HEAD_UPPER_RAD = tuple(map(math.radians, (YAW_LIMIT_DEG[1], PITCH_LIMIT_DEG[1])))
POSITION_EPS = 1e-12  # Roundoff only; no additional mechanical range.


def udp_only_uri(uri):
    """Retain the selected interfaces/discovery; disable SHM in this process."""
    if uri.startswith("file://"):
        location = urlparse(uri)
        if location.netloc not in ("", "localhost"):
            raise ValueError("DDS配置仅支持本机file URI")
        root = ET.parse(unquote(location.path)).getroot()
    elif uri.lstrip().startswith("<"):
        root = ET.fromstring(uri)
    else:
        raise ValueError("DDS配置必须为file URI或XML")
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
        shm = next((n for n in domain if local(n.tag) == "SharedMemory"), None)
        if shm is None:
            shm = ET.SubElement(domain, namespace + "SharedMemory")
        enable = next((n for n in shm if local(n.tag) == "Enable"), None)
        if enable is None:
            enable = ET.SubElement(shm, namespace + "Enable")
        enable.text = "false"
    return ET.tostring(root, encoding="unicode")


@dataclass
class Feedback:
    stamp: float
    received: float
    head: list


def decode_feedback(sample, previous_stamp, now, wall):
    if not hasattr(sample, "q") or len(sample.q) != 23:
        return None
    stamp = sample.header_sec + sample.header_nanosec * 1e-9
    if (not math.isfinite(stamp) or not -.05 <= wall - stamp <= MAX_AGE
            or stamp <= previous_stamp or not all(math.isfinite(x) for x in sample.q)):
        return None
    return Feedback(stamp, now, list(sample.q[21:23]))


class HeadFeedback:
    """Only a /rt/joint_state reader; no motion writers or IMU dependency."""
    def __init__(self):
        from cyclonedds.domain import DomainParticipant
        from cyclonedds.sub import DataReader
        from cyclonedds.topic import Topic
        from dds_leju_types import JointState
        self.participant = DomainParticipant(0)
        self.reader = DataReader(self.participant, Topic(self.participant, "/rt/joint_state", JointState))
        self.last = None

    def fresh(self):
        now, wall = time.monotonic(), time.time()
        for sample in self.reader.take(N=1000):
            item = decode_feedback(sample, self.last.stamp if self.last else -math.inf, now, wall)
            if item is not None:
                self.last = item
        if (self.last is None or now - self.last.received > MAX_AGE
                or not -.05 <= wall - self.last.stamp <= MAX_AGE):
            raise RuntimeError("JOINT_FEEDBACK_STALE: 未收到新鲜23关节反馈")
        return self.last.head.copy()

    def wait(self, timeout=5):
        deadline = time.monotonic() + timeout
        while True:
            try:
                return self.fresh()
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.02)


class LegacyHeadSdk:
    def __init__(self):
        bridge = os.environ.get("HEAD_SDK_BRIDGE", str(HERE / "build" / os.uname().machine / "head_legacy_sdk_cli"))
        self.process = subprocess.Popen([bridge], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, bufsize=0)
        self.buffer = b""

    def request(self, line, timeout=.15):
        deadline = time.monotonic() + timeout
        self.process.stdin.write((line + "\n").encode("ascii"))
        while True:
            while b"\n" in self.buffer:
                output, self.buffer = self.buffer.split(b"\n", 1)
                if output.startswith(b"SDK_RESULT "):
                    return output.split()[1:] == [b"1"]
                print("[旧SDK] " + output.decode(errors="replace"), file=sys.stderr, flush=True)
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.process.stdout], [], [], remaining)[0]:
                raise RuntimeError("HEAD_SDK_REPLY_TIMEOUT")
            output = os.read(self.process.stdout.fileno(), 65536)
            if not output:
                raise RuntimeError("HEAD_SDK_PROCESS_EXITED")
            self.buffer += output
            if len(self.buffer) > 1048576:
                raise RuntimeError("HEAD_SDK_OUTPUT_OVERFLOW")

    def initialize(self):
        if not self.request("INIT", timeout=10):
            raise RuntimeError("HEAD_SDK_INITIALIZE_FAILED")

    def publish(self, q, v):
        if not self.request("PUB " + " ".join(format(x, ".17g") for x in q + v)):
            raise RuntimeError("HEAD_SDK_PUBLISH_REJECTED")

    def close(self):
        try:
            if self.process.poll() is None:
                self.request("CLOSE", timeout=.5)
                self.process.wait(timeout=1)
        except (OSError, RuntimeError, subprocess.TimeoutExpired):
            pass
        finally:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=1)
            self.process.stdin.close()
            self.process.stdout.close()


def check_pose(q):
    if (len(q) != 2 or not all(math.isfinite(x) for x in q)
            or any(not lo - POSITION_EPS <= value <= hi + POSITION_EPS
                   for value, lo, hi in zip(q, HEAD_LOWER_RAD, HEAD_UPPER_RAD))):
        raise RuntimeError("HEAD_FEEDBACK_OUTSIDE_BOUNDS")


def controller_state(observer, now, require_stable=False):
    latest = observer.snapshot()
    if (latest is None or not 0 <= now - latest["received_monotonic"] <= .75
            or observer.error is not None):
        raise RuntimeError("CONTROLLER_STATE_STALE")
    if (latest["current_controller"] not in ("amp", "heft_pico")
            or (require_stable and latest["controller_transitioning"])):
        raise RuntimeError("CONTROLLER_NOT_READY")
    return latest


def trajectory(initial, target, elapsed, duration):
    u = max(0., min(1., elapsed / duration))
    weight, derivative = u * u * (3 - 2 * u), 6 * u * (1 - u) / duration
    return ([x + weight * (y - x) for x, y in zip(initial, target)],
            [derivative * (y - x) for x, y in zip(initial, target)])


def control(args, feedback, observer, report, save, log, sdk_factory=LegacyHeadSdk, clock=time):
    initial = feedback.fresh()
    check_pose(initial)
    state = controller_state(observer, clock.monotonic(), require_stable=True)
    report.update(initial_head_deg=list(map(math.degrees, initial)),
                  controller=state["current_controller"], head_mode=state["head_mode"])
    save()
    if not args.execute:
        print("只读采集完成，未初始化运动SDK。", flush=True)
        return

    sdk = sdk_factory()
    try:
        sdk.initialize()
        initial = feedback.fresh()
        check_pose(initial)
        # Initial, feedback and SDK command bounds use the same head limits.
        controller_state(observer, clock.monotonic(), require_stable=True)
        target = [initial[0] if args.yaw_deg is None else math.radians(args.yaw_deg),
                  math.radians(args.pitch_deg)]
        duration = max(args.duration,
                       1.5 * max(abs(y - x) for x, y in zip(initial, target)) / MAX_SPEED)
        report.update(initial_head_deg=list(map(math.degrees, initial)),
                      target_yaw_deg=math.degrees(target[0]),
                      target_head_deg=list(map(math.degrees, target)),
                      duration_seconds=duration, status="prewarming")
        save()
        last_sent, next_send = None, clock.monotonic()

        def send(q, v):
            nonlocal last_sent, next_send
            now = clock.monotonic()
            if last_sent is not None and now - last_sent > .15:
                raise RuntimeError("HEAD_PUBLISH_LOOP_GAP")
            controller_state(observer, now)
            actual = feedback.fresh()
            check_pose(actual)
            if args.yaw_deg is None and abs(actual[0] - initial[0]) > math.radians(3):
                raise RuntimeError("HEAD_YAW_DEVIATED")
            sdk.publish(q, v)
            last_sent = clock.monotonic()
            report["head_frames_sent"] += 1
            report["actual_head_deg"] = list(map(math.degrees, actual))
            next_send = max(next_send + 1 / RATE, last_sent)
            clock.sleep(max(0., next_send - clock.monotonic()))
            return actual

        for _ in range(25):
            send(initial, [0., 0.])
        report["status"] = "moving"
        save()
        begin, next_log, reached = clock.monotonic(), 0., False
        yaw_description = "yaw保持当前值" if args.yaw_deg is None else f"yaw={args.yaw_deg:g}°"
        print(f"仅控制头部：pitch={args.pitch_deg:g}°，{yaw_description}，{RATE}Hz。Ctrl+C停止发布。", flush=True)
        while True:
            elapsed = clock.monotonic() - begin
            q, v = trajectory(initial, target, elapsed, duration)
            actual = send(q, v)
            if elapsed > 2 and any(
                    abs(command - start) > math.radians(3)
                    and abs(measured - start) < math.radians(.3)
                    for command, measured, start in zip(q, actual, initial)):
                raise RuntimeError("HEAD_NO_RESPONSE")
            if elapsed >= next_log:
                log.write(json.dumps({"elapsed": elapsed, "time": clock.time(),
                                      "command_deg": list(map(math.degrees, q)),
                                      "actual_deg": list(map(math.degrees, actual))}) + "\n")
                next_log = elapsed + .2
            if elapsed >= duration + 3:
                if any(abs(measured - goal) > math.radians(2)
                       for measured, goal in zip(actual, target)):
                    raise RuntimeError("HEAD_TARGET_NOT_REACHED_OR_HOLD_LOST")
                if not reached:
                    reached = True
                    report.update(status="holding", target_verified=True, verified_at=clock.time())
                    save()
                    print("已到位并持续保持，实测角度：" + str(report["actual_head_deg"]), flush=True)
            if args.hold_seconds > 0 and elapsed >= duration + args.hold_seconds:
                report.update(status="completed", exit_reason="finite_hold_completed")
                return
    finally:
        sdk.close()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="实际发布头部轨迹；不加则只读")
    parser.add_argument("--pitch-deg", type=float, default=35, help="下俯为正，范围-20..45°，默认35")
    parser.add_argument("--yaw-deg", type=float, default=None,
                        help="yaw绝对目标角，范围-90..90°；省略时保持启动yaw")
    parser.add_argument("--duration", type=float, default=7, help="运动时长，默认7秒；自动延长以限制8°/s")
    parser.add_argument("--hold-seconds", type=float, default=0, help="0=持续保持到Ctrl+C；有限保持至少3秒")
    parser.add_argument("--output-dir", type=Path, default=HERE / "validation", help="本次反馈和退出记录的父目录")
    args = parser.parse_args(argv)
    if not math.isfinite(args.pitch_deg) or not PITCH_LIMIT_DEG[0] <= args.pitch_deg <= PITCH_LIMIT_DEG[1]:
        parser.error("pitch-deg必须为-20..45°的有限数值")
    if args.yaw_deg is not None and (not math.isfinite(args.yaw_deg)
                                   or not YAW_LIMIT_DEG[0] <= args.yaw_deg <= YAW_LIMIT_DEG[1]):
        parser.error("yaw-deg必须为-90..90°的有限数值")
    if not math.isfinite(args.duration) or not 2 <= args.duration <= 120:
        parser.error("duration必须为2..120秒的有限数值")
    if not math.isfinite(args.hold_seconds) or (args.hold_seconds != 0 and not 3 <= args.hold_seconds <= 86400):
        parser.error("hold-seconds必须为0（无限保持）或3..86400秒")
    return args


def main(argv=None):
    args = parse_args(argv)
    output = args.output_dir / (time.strftime("%Y%m%d_%H%M%S") + f"_{os.getpid()}_{time.time_ns() % 1000000000}")
    output.mkdir(parents=True, exist_ok=False)
    report = dict(status="starting", body_commands_sent=0, head_frames_sent=0,
                  head_api="RobanVRAPI::publishHeadJointCmd", rate_hz=RATE,
                  target_pitch_deg=args.pitch_deg, target_yaw_deg=args.yaw_deg,
                  yaw_mode="hold_initial" if args.yaw_deg is None else "target",
                  hold_seconds=args.hold_seconds,
                  target_verified=False, execute=args.execute, started_at=time.time())

    def save():
        temporary = output / "session_status.json.tmp"
        temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        temporary.replace(output / "session_status.json")

    observer, lock = None, None
    exit_code = 0
    try:
        if args.execute:
            lock = open("/tmp/roban_head_sdk_pose.lock", "a")
            try:
                fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError("ANOTHER_HEAD_SDK_POSE_IS_RUNNING") from None
        uri = os.environ.get("CYCLONEDDS_URI", "file:///etc/cyclonedds/cyclonedds_shm.xml")
        os.environ["CYCLONEDDS_URI"] = udp_only_uri(uri)
        from dds_heft_controller_observer import ControllerStateObserver
        feedback = HeadFeedback()
        feedback.wait()
        observer = ControllerStateObserver(feedback.participant, poll_interval=.25).start()
        deadline = time.monotonic() + 2
        while True:
            try:
                controller_state(observer, time.monotonic(), require_stable=True)
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.02)
        report["status"] = "ready" if args.execute else "read_only"
        with (output / "samples.jsonl").open("w", buffering=1) as log:
            control(args, feedback, observer, report, save, log)
    except KeyboardInterrupt:
        report.update(status="stopped", exit_reason="keyboard_interrupt")
        exit_code = 130
    except Exception as exc:
        report.update(status="stopped", exit_reason=str(exc), exception_type=type(exc).__name__)
        exit_code = 1
    finally:
        if observer is not None:
            observer.stop()
        if lock is not None:
            lock.close()
        report["finished_at"] = time.time()
        save()
        print(json.dumps(report, ensure_ascii=False), flush=True)
        print("记录：" + str(output), flush=True)
        if args.execute:
            print("头部参考已停止；停发后的姿态由原控制器处理，本次实测会回到原朝向。", flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
