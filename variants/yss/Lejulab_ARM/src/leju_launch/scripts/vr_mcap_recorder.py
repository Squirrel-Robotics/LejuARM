#!/usr/bin/env python3
"""Read Pico buttons and own one native MCAP recorder per demonstration.

This process creates DDS readers only. It never publishes robot commands.
The system recorder and the existing HEFT CSV recorder remain independent.
"""
from __future__ import annotations

import argparse
from collections import deque
import ctypes
import dataclasses
from datetime import datetime
import fcntl
import json
import logging
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import statistics
import sys
import time
import uuid
from vr_workbench_support import FeedbackHealth, CommandInbox
from zoneinfo import ZoneInfo

LOG = logging.getLogger("vr_mcap")
DEFAULT_OUTPUT = Path("/home/lab/.cache/lejulab/vr_transfer")
RECORDING_TIMEZONE = ZoneInfo("Asia/Shanghai")


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")
    temporary.replace(path)


def create_session_directory(root, button_stamp_ns):
    """Reserve one directory atomically; repeated minutes never overwrite data."""
    minute = datetime.fromtimestamp(
        button_stamp_ns // 1_000_000_000, RECORDING_TIMEZONE
    ).strftime("%Y_%m_%d_%H_%M")
    index = 0
    while True:
        name = minute if index == 0 else f"{minute}_{index:02d}"
        directory = Path(root) / name
        try:
            directory.mkdir()
            break
        except FileExistsError:
            index += 1
    (directory / "vision").mkdir()
    (directory / "Action_ObservationState").mkdir()
    return directory


class ButtonEdges:
    """Require a release after startup; stop wins if X and Y overlap."""

    def __init__(self, threshold=0.5, max_age=0.5):
        self.threshold = threshold
        self.max_age = max_age
        self.previous = None
        self.last_stamp = -1

    def update(self, msg, now_ns=None):
        now_ns = time.time_ns() if now_ns is None else now_ns
        stamp = int(msg.header_sec) * 1_000_000_000 + int(msg.header_nanosec)
        age = (now_ns - stamp) / 1e9
        if not -0.05 <= age <= self.max_age or stamp <= self.last_stamp:
            return None
        self.last_stamp = stamp
        trigger = float(msg.left_trigger)
        if not math.isfinite(trigger):
            return None
        lt = trigger >= self.threshold
        x = bool(msg.left_first_button_pressed)
        y = bool(msg.left_second_button_pressed)
        current = (lt and y and not x, lt and x, y)
        previous, self.previous = self.previous, current
        if previous is None:
            return None
        if current[1] and not previous[1]:
            return ("stop", stamp)
        # Releasing X while Y remains down must not start a new segment.
        if current[0] and not previous[0] and (not previous[1] or not previous[2]):
            return ("start", stamp)
        return None


class StreamHealth:
    """Observe source timestamps only; never republishes or alters commands."""
    def __init__(self):
        self.joy_stamp = None
        self.reference_stamp = None
        self.references = deque(maxlen=300)
        self.buttons = {}
        self.reference_valid = False
        self.reference_calibrated = False
        self.feedback = FeedbackHealth()

    def observe_joy(self, msg):
        self.joy_stamp = int(msg.header_sec) * 1_000_000_000 + int(msg.header_nanosec)
        self.buttons = {'lt': float(msg.left_trigger),
                        'x': bool(msg.left_first_button_pressed),
                        'y': bool(msg.left_second_button_pressed)}

    def observe_reference(self, msg):
        t = int(msg.header_sec) * 1_000_000_000 + int(msg.header_nanosec)
        self.reference_stamp = t
        self.reference_calibrated = bool(msg.calibrated)
        self.reference_valid = bool(msg.valid and msg.calibrated and len(msg.q) == 21 and
                                    all(math.isfinite(q) for q in msg.q))
        if not self.references or t > self.references[-1]:
            self.references.append(t)

    def snapshot(self, now_ns=None):
        now = time.time_ns() if now_ns is None else now_ns
        recent = [t for t in self.references if now - 2_000_000_000 <= t <= now + 50_000_000]
        intervals = [(b-a)/1e6 for a,b in zip(recent, recent[1:])]
        return {'last_joy_ns': self.joy_stamp, 'last_reference_ns': self.reference_stamp,
                'buttons': self.buttons, 'reference_valid': self.reference_valid,
                'reference_calibrated': self.reference_calibrated,
                'reference_hz': (len(recent)-1)*1e9/(recent[-1]-recent[0]) if len(recent)>1 else None,
                'reference_interval_median_ms': statistics.median(intervals) if intervals else None,
                'reference_interval_max_ms': max(intervals) if intervals else None,
                'observed_ns': now, 'feedback': self.feedback.snapshot(now)}


class SegmentRecorder:
    def __init__(self, output_dir, executable, config_path, domain_id=0,
                 stop_timeout=10.0, ready_timeout=5.0, vision=None):
        self.output_dir = Path(output_dir)
        self.executable = Path(executable)
        self.config_path = Path(config_path)
        self.domain_id = domain_id
        self.stop_timeout = stop_timeout
        self.ready_timeout = ready_timeout
        self.vision = vision
        self.telemetry = None
        self.vision_future = None
        self.native_ok = False
        self.process = None
        self.session = None
        self.last_session = None
        self.log_stream = None
        self.log_reader = None
        self.ready = False
        self.stop_requested = None
        self.signal_sent_at = None
        self.terminate_sent = False
        self.kill_sent = False
        self.ready_tail = ""
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.logs_dir = None
        self.status_path = self.output_dir / "vr_mcap_status.json"
        self.status("idle")

    def status(self, state, error=None):
        self.current_state = state
        write_json(self.status_path, {
            "schema_version": 2,
            "state": state, "pid": os.getpid(), "domain_id": self.domain_id,
            "output_dir": str(self.output_dir), "updated_ns": time.time_ns(),
            "session": self.session, "last_session": self.last_session,
            "vision_ready": self.vision.ready if self.vision else None,
            "vision_error": self.vision.error if self.vision else None,
            "error": error,
            "telemetry": self.telemetry.snapshot() if self.telemetry else None,
            "task": getattr(self, "task", {}),
        })

    def start(self, button_stamp_ns, source="LT+Y"):
        self.start_error = ""
        if self.process is not None or self.session is not None:
            LOG.info("Already recording or closing; LT+Y ignored")
            return False
        if self.vision and not self.vision.ready:
            LOG.error("LT+Y refused: upper RGB not ready: %s", self.vision.error)
            self.status("error", "Upper RGB not ready: " + str(self.vision.error))
            return False
        if self.telemetry is not None:
            health = self.telemetry.snapshot()
            now = time.time_ns()
            feedback = health['feedback']
            if not feedback['ready']:
                self.start_error = '关节反馈检查未通过：' + feedback['reason']
            elif not (health['reference_valid'] and health['last_reference_ns'] and
                      -50_000_000 <= now-health['last_reference_ns'] < 250_000_000):
                self.start_error = '动作参考未就绪，请完成Pico标定并确认参考持续更新'
            elif not (health['last_joy_ns'] and -50_000_000 <= now-health['last_joy_ns'] < 500_000_000):
                self.start_error = '手柄数据未更新'
            if self.start_error:
                self.status('error', self.start_error)
                LOG.warning('Start refused: %s', self.start_error)
                return False
        import yaml
        try:
            config = yaml.safe_load(self.config_path.read_text())
            if not isinstance(config, dict):
                raise ValueError("Recorder configuration must be a mapping")
            output = config.setdefault("output", {})
            minimum_mb = int(output.get("min_space", 1024))
            if shutil.disk_usage(self.output_dir).free < minimum_mb * 1024 * 1024:
                raise OSError(f"Less than {minimum_mb} MiB free on recording disk")
            stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
            stem = f"vr_{stamp}_{time.time_ns() % 1_000_000_000:09d}_{uuid.uuid4().hex[:6]}"
            session_dir = create_session_directory(self.output_dir, button_stamp_ns)
            action_dir = session_dir / "Action_ObservationState"
            self.logs_dir = action_dir / ".recorder"
            self.logs_dir.mkdir()
            prefix = action_dir / stem
            config.setdefault("dds", {})["domain_id"] = self.domain_id
            # A demonstration is retained as one file, without rolling deletion.
            output.update(prefix=str(prefix), append_date=False, split=False,
                          max_splits=0, max_file_size=0, max_duration=0)
            config.setdefault("timing", {})["start_immediately"] = True
            config.setdefault("logging", {})["console"] = True
            run_config = self.logs_dir / f"{stem}.yaml"
            run_config.write_text(yaml.safe_dump(config, allow_unicode=True))
            log_path = self.logs_dir / f"{stem}.log"
            self.log_stream = log_path.open("w")
            self.log_reader = log_path.open("r")
            self.session = {
                "name": stem, "mcap": str(prefix.with_suffix(".mcap")),
                "session_id": session_dir.name,
                "session_dir": str(session_dir),
                "vision_dir": str(session_dir / "vision"),
                "action_observation_dir": str(action_dir),
                "directory_timezone": "Asia/Shanghai",
                "metadata": str(prefix.with_suffix(".json")),
                "log": str(log_path), "config": str(run_config),
                "start_button_ns": button_stamp_ns,
                "recorder_spawn_ns": time.time_ns(), "domain_id": self.domain_id,
                "state": "starting", "start_source": source,
                "task": dict(getattr(self, "task", {})),
                "feedback_preflight": self.telemetry.feedback.snapshot() if self.telemetry else None,
            }
            self.ready = False
            self.ready_tail = ""
            self.stop_requested = None
            self.signal_sent_at = None
            self.terminate_sent = self.kill_sent = False
            self.process = subprocess.Popen([
                str(self.executable), f"--config={run_config}",
                f"--domain={self.domain_id}", f"--output-name={prefix}",
            ], stdin=subprocess.DEVNULL, stdout=self.log_stream,
                stderr=subprocess.STDOUT, start_new_session=True)
            self.session["recorder_pid"] = self.process.pid
            if self.vision:
                self.vision.begin(self.session)
                self.session["vision"] = {"status": "recording", "transport": "SSH buffered RGB"}
            write_json(Path(self.session["metadata"]), self.session)
            self.status("starting")
            LOG.info("LT+Y: starting %s", self.session["mcap"])
            return True
        except Exception as exc:
            LOG.exception("Cannot start MCAP recording")
            if self.process is not None:
                # Keep ownership so poll()/close() can always reap the child.
                self.request_stop(None, "start_error")
            else:
                self._close_logs()
                self.last_session, self.session = self.session, None
                self.status("error", str(exc))
            return False

    def request_stop(self, button_stamp_ns=None, reason="LT+X"):
        if self.process is None or self.stop_requested is not None:
            return
        self.stop_requested = time.monotonic()
        self.session.update(stop_button_ns=button_stamp_ns,
                            stop_requested_ns=time.time_ns(), stop_reason=reason)
        self.session["stop_boundary_ns"] = button_stamp_ns or self.session["stop_requested_ns"]
        self.status("closing")
        LOG.info("%s: closing %s", reason, self.session["mcap"])

    def _close_logs(self):
        for stream in [self.log_stream, self.log_reader]:
            if stream:
                stream.close()
        self.log_stream = self.log_reader = None

    def poll(self):
        if self.process is None:
            if self.vision_future is not None and self.vision_future.done():
                try:
                    result = self.vision_future.result()
                    self.session.update(result.pop('mcap_update', {}))
                    self.session["vision"] = result
                    if result.get('alignment'):
                        self.session["action_frames"] = result['alignment']['action_frames_in_button_window']
                    elif result.get('processing_location') == 'workstation':
                        self.session.update(processing_location='workstation',
                                            processing_state='awaiting_transfer')
                except Exception as exc:
                    LOG.exception("Vision or alignment failed; MCAP retained")
                    self.session["vision"] = {"status": "error", "error": str(exc)}
                self.vision_future = None
                self._finalize_session()
            return
        output = self.ready_tail + self.log_reader.read()
        self.ready_tail = output[-128:]
        if not self.ready and "Recording started" in output:
            self.ready = True
            self.session.update(recorder_ready_ns=time.time_ns(), state="recording")
            self.status("closing" if self.stop_requested is not None else "recording")
            LOG.info("Recording: %s", self.session["mcap"])
        now = time.monotonic()
        code = self.process.poll()
        if code is None and self.stop_requested is not None:
            # A quick LT+Y -> LT+X must not interrupt native initialization
            # before its signal handler has been installed.
            startup_s = (time.time_ns() - self.session["recorder_spawn_ns"]) / 1e9
            if self.signal_sent_at is None and (self.ready or startup_s >= self.ready_timeout):
                try:
                    self.process.send_signal(signal.SIGINT)
                    self.signal_sent_at = now
                except ProcessLookupError:
                    pass
            elif self.signal_sent_at is not None:
                elapsed = now - self.signal_sent_at
                if elapsed > self.stop_timeout + 3 and not self.kill_sent:
                    self.kill_sent = True
                    self.process.kill()
                    LOG.error("Recorder did not shut down; MCAP may be incomplete")
                elif elapsed > self.stop_timeout and not self.terminate_sent:
                    self.terminate_sent = True
                    self.process.terminate()
                    LOG.error("SIGINT timed out; sending SIGTERM to this recorder only")
        if code is not None:
            path = Path(self.session["mcap"])
            complete = False
            if path.exists() and path.stat().st_size >= 16:
                with path.open("rb") as stream:
                    head = stream.read(8)
                    stream.seek(-8, 2)
                    complete = head == stream.read(8) == b"\x89MCAP0\r\n"
            message_count = None
            channels = []
            if complete:
                try:
                    from mcap.reader import make_reader
                    with path.open("rb") as stream:
                        summary = make_reader(stream).get_summary()
                        message_count = summary.statistics.message_count
                        channels = sorted({channel.topic for channel in summary.channels.values()})
                        self.session["mcap_start_ns"] = summary.statistics.message_start_time
                        self.session["mcap_end_ns"] = summary.statistics.message_end_time
                except Exception:
                    complete = False
                    LOG.exception("Cannot read MCAP summary")
            expected = self.stop_requested is not None
            self.native_ok = expected and code == 0 and complete and bool(message_count)
            self.session.update(exit_code=code, closed_ns=time.time_ns(),
                                mcap_complete=complete,
                                message_count=message_count, topics=channels,
                                state="finalizing_vision" if self.vision else "closing")
            write_json(Path(self.session["metadata"]), self.session)
            self._close_logs()
            self.process = None
            if self.vision:
                if not self.session.get("stop_boundary_ns"):
                    self.session["stop_boundary_ns"] = self.session["closed_ns"]
                self.vision_future = self.vision.finish(self.session)
                self.status("finalizing_vision")
                LOG.info("MCAP closed; exporting RGB and verifying the raw transfer")
            else:
                self._finalize_session()

    def _finalize_session(self):
        vision_ok = not self.vision or self.session.get("vision", {}).get("status") == "complete"
        ok = self.native_ok and vision_ok
        self.session.update(state="closed" if ok else "error", finalized_ns=time.time_ns())
        write_json(Path(self.session["metadata"]), self.session)
        if ok and self.session.get('processing_location') == 'workstation':
            try:
                from vr_raw_transfer import publish_session
                publish_session(self.session, self.vision.config)
            except Exception as exc:
                ok = False
                self.session.update(state='error', transfer_error=str(exc))
                write_json(Path(self.session['metadata']), self.session)
                LOG.exception('Raw export not published; temporary sources retained')
        self.last_session, self.session = self.session, None
        if ok:
            LOG.info("Raw MCAP closed: %s", self.last_session["mcap"])
            if self.vision:
                LOG.info("RGB saved: %s; action frames=%s video frames=%s",
                         self.last_session["vision_dir"], self.last_session.get("action_frames"),
                         self.last_session["vision"].get("frame_count"))
                if self.last_session.get('processing_location') == 'workstation':
                    LOG.info('Raw session ready for workstation transfer; local crop/alignment pending')
            self.status("idle")
        else:
            LOG.error("Recording incomplete; inspect metadata: %s", self.last_session["metadata"])
            self.status("error", "MCAP, RGB transfer or alignment verification failed")

    def close(self, reason="service_shutdown"):
        self.request_stop(None, reason)
        while self.process is not None or self.vision_future is not None:
            self.poll()
            time.sleep(0.02)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    project = Path(__file__).resolve().parents[3]
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT,
                        help="Root containing YYYY_MM_DD_HH_MM session directories")
    parser.add_argument("--recorder", type=Path,
                        default=project / "installed/bin/lejusdk_recorder")
    parser.add_argument("--config", type=Path,
                        default=project / "src/leju_launch/config/vr_mcap_recorder.yaml")
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--joy-topic", default="/rt/pico/joysticks")
    parser.add_argument("--trigger-threshold", type=float, default=0.5)
    parser.add_argument("--vision-config", type=Path,
                        default=project / "src/leju_launch/config/vr_vision_config.json")
    parser.add_argument("--no-vision", action="store_true",
                        help="Explicit MCAP-only mode, used by isolated native-recorder tests")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="[vr_mcap] %(asctime)s %(levelname)s %(message)s")
    args.output_dir = args.output_dir.expanduser().resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    lock = (args.output_dir / ".vr_mcap_recorder.lock").open("a+")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        LOG.error("A VR MCAP listener already owns %s", args.output_dir)
        return 1
    if not args.recorder.is_file() or not os.access(args.recorder, os.X_OK):
        parser.error(f"Native recorder is not executable: {args.recorder}")
    if not args.config.is_file():
        parser.error(f"Recorder config does not exist: {args.config}")
    import yaml  # Validate dependency before announcing readiness.
    import mcap.reader
    lib_dir = os.environ.get("CYCLONEDDS_LIB_DIR")
    if lib_dir:
        ctypes.CDLL(str(Path(lib_dir) / "libddsc.so"), mode=ctypes.RTLD_GLOBAL)
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    from cyclonedds.qos import Qos, Policy
    from cyclonedds.idl import IdlStruct
    from cyclonedds.idl.types import float32, int32, uint32

    # Match the deployed QuestJoysticks.idl exactly (also used by Pico).
    class WireJoystick(IdlStruct, typename="leju::msgs::QuestJoysticks"):
        __annotations__ = {
            "header_sec": int32, "header_nanosec": uint32,
            **{f"{hand}_{field}": kind for hand in ["left", "right"]
               for field, kind in [
                   ("x", float32), ("y", float32), ("trigger", float32),
                   ("grip", float32), ("first_button_pressed", bool),
                   ("second_button_pressed", bool), ("first_button_touched", bool),
                   ("second_button_touched", bool)]},
        }
    WireJoystick = dataclasses.dataclass(WireJoystick)
    participant = DomainParticipant(args.domain_id)
    reader = DataReader(participant, Topic(participant, args.joy_topic, WireJoystick),
                        qos=Qos(Policy.Reliability.BestEffort,
                                Policy.History.KeepLast(100)))
    # The existing shared module defines only wire contracts and publishers;
    # importing HeftReference creates no writer and sends no robot commands.
    sys.path.insert(0, str(project / 'src/leju_gmr_common'))
    from leju_gmr_common.dds_io import HeftReference
    reference_reader = DataReader(participant, Topic(participant, '/rt/pico/heft_reference', HeftReference),
        qos=Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(200)))
    vision = None
    if not args.no_vision:
        from vr_vision_client import VisionCoordinator
        vision = VisionCoordinator(json.loads(args.vision_config.read_text()), args.output_dir)
    from vr_workbench_support import joint_reader
    feedback_participant, feedback_reader, feedback_kind = joint_reader(args.domain_id)
    inbox = CommandInbox(args.output_dir)
    recorder = SegmentRecorder(args.output_dir, args.recorder, args.config, args.domain_id, vision=vision)
    recorder.telemetry = StreamHealth()
    detector = ButtonEdges(args.trigger_threshold)
    stopping = False

    def stop_handler(signum, frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    if vision:
        LOG.info("Waiting for upper RGB; LT+Y is accepted after the RGB+MCAP Ready message")
    else:
        LOG.info("Ready: LT+Y starts, LT+X stops; domain=%s topic=%s; output=%s",
                 args.domain_id, args.joy_topic, args.output_dir)
    last_vision_status = None
    last_heartbeat = 0
    try:
        while not stopping:
            if vision:
                vision.tick()
                if (vision.ready, vision.error) != last_vision_status:
                    last_vision_status = (vision.ready, vision.error)
                    recorder.status(recorder.current_state)
                    if vision.ready:
                        LOG.info("Ready: LT+Y starts RGB+MCAP, LT+X stops both; output=%s", args.output_dir)
            recorder.poll()
            for sample in feedback_reader.take(N=32):
                if isinstance(sample, feedback_kind):
                    recorder.telemetry.feedback.observe(sample)
            for reference in reference_reader.take(N=200):
                if isinstance(reference, HeftReference):
                    recorder.telemetry.observe_reference(reference)
            for msg in reader.take(N=100):
                if not isinstance(msg, WireJoystick):
                    continue
                recorder.telemetry.observe_joy(msg)
                event = detector.update(msg)
                if event:
                    if event[0] == "start":
                        recorder.start(event[1])
                    else:
                        recorder.request_stop(event[1])
            inbox.consume(recorder)
            if time.monotonic() - last_heartbeat >= 1:
                last_heartbeat = time.monotonic()
                recorder.status(recorder.current_state)
            time.sleep(0.01)
    finally:
        recorder.close()
        if vision:
            vision.close()
        recorder.status("stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
