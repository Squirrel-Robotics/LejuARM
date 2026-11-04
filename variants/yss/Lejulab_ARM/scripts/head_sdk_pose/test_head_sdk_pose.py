#!/usr/bin/env python3
"""Offline tests; no robot connection and no DDS/SDK required."""
import contextlib
import io
import math
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import head_sdk_pose as head


class Clock:
    def __init__(self, interrupt_after=None):
        self.now = 100.
        self.interrupt_after = interrupt_after

    def monotonic(self):
        return self.now

    def time(self):
        return self.now + 1000000

    def sleep(self, duration):
        self.now += duration
        if self.interrupt_after and self.now >= 100 + self.interrupt_after:
            raise KeyboardInterrupt


class Feedback:
    def __init__(self, clock, follows=True, stale_after=math.inf):
        self.q = [.01, .03]
        self.clock, self.follows, self.stale_after = clock, follows, stale_after

    def fresh(self):
        if self.clock.now >= self.stale_after:
            raise RuntimeError("JOINT_FEEDBACK_STALE")
        return self.q.copy()


class Observer:
    error = None

    def __init__(self, clock, fail_after=math.inf):
        self.clock, self.fail_after = clock, fail_after

    def snapshot(self):
        return dict(received_monotonic=self.clock.now,
                    current_controller="amp" if self.clock.now < self.fail_after else "fault",
                    controller_transitioning=False, head_mode=1)


class Sdk:
    def __init__(self, feedback, lag=0.):
        self.feedback = feedback
        self.lag = lag
        self.frames = []
        self.closed = False

    def initialize(self):
        pass

    def publish(self, q, v):
        if len(q) != 2 or len(v) != 2:
            raise AssertionError("head command must have two joints")
        self.frames.append((q.copy(), v.copy()))
        if self.feedback.follows:
            # Model the beta controller deadband: the head settles a fixed
            # distance behind the commanded pitch, clamped at the joint limit.
            self.feedback.q = [q[0], max(q[1] - self.lag, head.HEAD_LOWER_RAD[1])]

    def close(self):
        self.closed = True


class HeadTests(unittest.TestCase):
    def setUp(self):
        self.quiet = contextlib.redirect_stdout(io.StringIO())
        self.quiet.__enter__()

    def tearDown(self):
        self.quiet.__exit__(None, None, None)

    def harness(self, argv=None, *, clock=None, follows=True, stale_after=math.inf, fail_after=math.inf):
        args = head.parse_args(argv if argv is not None else ["--execute", "--hold-seconds", "3"])
        clock = clock or Clock()
        feedback = Feedback(clock, follows=follows, stale_after=stale_after)
        observer = Observer(clock, fail_after=fail_after)
        sdk = Sdk(feedback, lag=math.radians(args.deadband_compensation_deg))
        report = dict(head_frames_sent=0, body_commands_sent=0, target_verified=False)
        def run(factory=None):
            return head.control(args, feedback, observer, report, lambda: None,
                                io.StringIO(), factory or (lambda: sdk), clock)
        return run, clock, feedback, sdk, report

    def test_default_hold_is_unlimited(self):
        args = head.parse_args([])
        self.assertFalse(args.execute)
        self.assertEqual(args.hold_seconds, 0)

    def test_reject_invalid_angles_and_durations(self):
        with contextlib.redirect_stderr(io.StringIO()):
            for option, value in [("--pitch-deg", "nan"), ("--pitch-deg", "45.001"),
                                  ("--pitch-deg", "-21"), ("--duration", "inf"),
                                  ("--duration", "0"), ("--hold-seconds", "1"),
                                  ("--hold-seconds", "-1"),
                                  ("--deadband-compensation-deg", "-1"),
                                  ("--deadband-compensation-deg", "16"),
                                  ("--deadband-compensation-deg", "nan"),
                                  ("--yaw-deg", "91"), ("--yaw-deg", "-90.5"),
                                  ("--yaw-deg", "nan")]:
                with self.subTest(option=option, value=value), self.assertRaises(SystemExit):
                    head.parse_args([option, value])

    def test_read_only_never_creates_legacy_sdk(self):
        run, _, _, sdk, report = self.harness([])
        def forbidden():
            self.fail("read-only created SDK")
        run(forbidden)
        self.assertEqual(sdk.frames, [])
        self.assertEqual(report["body_commands_sent"], 0)

    def test_offline_startup_failure_is_saved(self):
        import json
        with tempfile.TemporaryDirectory() as folder:
            with patch.dict(os.environ, CYCLONEDDS_URI='<CycloneDDS><Domain id="0"/></CycloneDDS>'):
                with patch.object(head, "HeadFeedback", side_effect=RuntimeError("TEST_NO_DEVICE")):
                    result = head.main(["--output-dir", folder])
            statuses = list(Path(folder).glob("*/session_status.json"))
            self.assertEqual(result, 1)
            self.assertEqual(len(statuses), 1)
            saved = json.loads(statuses[0].read_text())
            self.assertEqual(saved["exit_reason"], "TEST_NO_DEVICE")
            self.assertEqual(saved["head_frames_sent"], 0)
            self.assertEqual(saved["body_commands_sent"], 0)

    def test_smooth_motion_and_finite_hold_keep_yaw(self):
        run, clock, feedback, sdk, report = self.harness()
        yaw = feedback.q[0]
        run()
        self.assertEqual(report["status"], "completed")
        self.assertTrue(report["target_verified"])
        self.assertTrue(sdk.closed)
        self.assertGreaterEqual(clock.now, 110.5)
        self.assertAlmostEqual(math.degrees(feedback.q[1]), 35)
        for q, v in sdk.frames:
            self.assertEqual(q[0], yaw)
            self.assertEqual(v[0], 0)
            self.assertLessEqual(abs(v[1]), head.MAX_SPEED + 1e-12)

    def test_long_move_stretches_duration_for_velocity_limit(self):
        run, _, feedback, sdk, report = self.harness(["--execute", "--duration", "2", "--hold-seconds", "3"])
        feedback.q[1] = math.radians(-19)
        run()
        self.assertGreater(report["duration_seconds"], 10)
        self.assertLessEqual(max(abs(v[1]) for _, v in sdk.frames), head.MAX_SPEED + 1e-12)

    def test_unlimited_hold_still_publishing_after_ninety_seconds(self):
        run, _, _, sdk, report = self.harness(["--execute"], clock=Clock(interrupt_after=91))
        with self.assertRaises(KeyboardInterrupt):
            run()
        self.assertTrue(report["target_verified"])
        self.assertGreater(len(sdk.frames), 4400)
        self.assertTrue(sdk.closed)

    def test_stale_feedback_before_start_sends_nothing(self):
        run, _, _, sdk, _ = self.harness(stale_after=100)
        with self.assertRaisesRegex(RuntimeError, "FEEDBACK_STALE"):
            run()
        self.assertEqual(sdk.frames, [])

    def test_stale_feedback_during_motion_closes_sdk(self):
        run, _, _, sdk, report = self.harness(stale_after=102)
        with self.assertRaisesRegex(RuntimeError, "FEEDBACK_STALE"):
            run()
        self.assertTrue(sdk.closed)
        self.assertFalse(report["target_verified"])

    def test_controller_change_stops_head_loop(self):
        run, _, _, sdk, _ = self.harness(fail_after=102)
        with self.assertRaisesRegex(RuntimeError, "CONTROLLER_NOT_READY"):
            run()
        self.assertTrue(sdk.closed)

    def test_no_motion_feedback_is_not_reported_as_reached(self):
        run, _, _, sdk, report = self.harness(follows=False)
        with self.assertRaisesRegex(RuntimeError, "HEAD_NO_RESPONSE"):
            run()
        self.assertTrue(sdk.closed)
        self.assertFalse(report["target_verified"])

    def test_initial_pose_out_of_official_bounds_sends_nothing(self):
        run, _, feedback, sdk, _ = self.harness()
        feedback.q[1] = math.radians(45.1)
        with self.assertRaisesRegex(RuntimeError, "HEAD_FEEDBACK_OUTSIDE_BOUNDS"):
            run()
        self.assertEqual(sdk.frames, [])

    def test_accept_official_pitch_range_including_37(self):
        for value in (-20, 35, 37, 45):
            with self.subTest(pitch=value):
                self.assertEqual(head.parse_args(["--pitch-deg", str(value)]).pitch_deg, value)

    def test_feedback_accepts_official_edges_and_rejects_overrun(self):
        for yaw, pitch in [(-90, -20), (90, 45), (0, 37)]:
            head.check_pose(list(map(math.radians, (yaw, pitch))))
        for yaw, pitch in [(-90.001, 0), (90.001, 0), (0, -20.001), (0, 45.001)]:
            with self.subTest(yaw=yaw, pitch=pitch), self.assertRaisesRegex(RuntimeError, "OUTSIDE_BOUNDS"):
                head.check_pose(list(map(math.radians, (yaw, pitch))))

    def test_motion_can_reach_official_pitch_edges_and_37(self):
        for pitch in (-20, 37, 45):
            with self.subTest(pitch=pitch):
                run, _, feedback, sdk, report = self.harness(["--execute", "--pitch-deg", str(pitch),
                                                             "--hold-seconds", "3",
                                                             "--deadband-compensation-deg", "0"])
                run()
                self.assertTrue(report["target_verified"])
                self.assertAlmostEqual(math.degrees(feedback.q[1]), pitch)
                self.assertLessEqual(max(abs(v[1]) for _, v in sdk.frames), head.MAX_SPEED + 1e-12)

    def test_deadband_compensation_commands_ahead_and_settles_on_target(self):
        run, _, feedback, sdk, report = self.harness(["--execute", "--pitch-deg", "35", "--hold-seconds", "3",
                                                      "--deadband-compensation-deg", "6.5"])
        run()
        self.assertTrue(report["target_verified"])
        self.assertEqual(report["status"], "completed")
        self.assertAlmostEqual(math.degrees(feedback.q[1]), 35)
        self.assertAlmostEqual(max(math.degrees(q[1]) for q, _ in sdk.frames), 41.5, places=6)

    def test_overshoot_is_pulled_back_past_the_hysteresis_band(self):
        run, _, feedback, sdk, report = self.harness(["--execute", "--pitch-deg", "35", "--hold-seconds", "3",
                                                      "--deadband-compensation-deg", "6.5"])
        sdk.lag = math.radians(5.18)  # smaller lag: the default-style bias overshoots
        run()
        self.assertTrue(report["target_verified"])
        self.assertLessEqual(abs(math.degrees(feedback.q[1]) - 35), 1.05)
        self.assertLess(min(math.degrees(q[1]) for q, _ in sdk.frames), 35 - 4)  # jumped past the band

    def test_deadband_compensation_clamps_command_to_pitch_limit(self):
        run, _, feedback, sdk, report = self.harness(["--execute", "--pitch-deg", "45", "--hold-seconds", "3",
                                                      "--deadband-compensation-deg", "6.5"])
        with self.assertRaisesRegex(RuntimeError, "TARGET_NOT_REACHED"):
            run()
        # The 45deg target cannot be commanded past the limit, so with the
        # modelled lag the head settles 6.5deg short and the hold check fails.
        self.assertFalse(report["target_verified"])
        self.assertAlmostEqual(max(math.degrees(q[1]) for q, _ in sdk.frames), 45, places=6)

    def test_deadband_lag_variability_is_corrected_adaptively(self):
        run, _, feedback, sdk, report = self.harness(["--execute", "--pitch-deg", "35", "--hold-seconds", "3"])
        sdk.lag = math.radians(8.7)  # larger than the default 6.5deg bias
        run()
        self.assertTrue(report["target_verified"])
        self.assertEqual(report["status"], "completed")
        self.assertLessEqual(abs(math.degrees(feedback.q[1]) - 35), 2.05)
        self.assertGreater(report["deadband_compensation_applied_deg"], 6.5)
        self.assertLessEqual(max(math.degrees(q[1]) for q, _ in sdk.frames), 45 + 1e-9)

    def test_yaw_target_is_commanded_together_with_pitch(self):
        run, _, feedback, sdk, report = self.harness(["--execute", "--pitch-deg", "35", "--yaw-deg", "30",
                                                      "--hold-seconds", "3"])
        run()
        self.assertTrue(report["target_verified"])
        self.assertAlmostEqual(math.degrees(feedback.q[0]), 30)
        self.assertAlmostEqual(math.degrees(feedback.q[1]), 35)
        self.assertTrue(all(abs(math.degrees(q[0])) <= 90 for q, _ in sdk.frames))
        self.assertEqual(report["yaw_bias_deg"], 0)

    def test_start_above_old_pitch_limit_and_keep_official_yaw(self):
        run, _, feedback, sdk, report = self.harness(["--execute", "--pitch-deg", "37", "--hold-seconds", "3"])
        feedback.q = list(map(math.radians, (89, 45)))
        run()
        self.assertTrue(report["target_verified"])
        self.assertAlmostEqual(math.degrees(feedback.q[1]), 37)
        self.assertTrue(all(abs(math.degrees(q[0])-89) < 1e-10 and v[0] == 0 for q, v in sdk.frames))

    @unittest.skipUnless(os.environ.get("HEAD_SDK_BRIDGE"), "native SDK helper is compiled on lower computer")
    def test_native_command_guard_without_sdk_initialization(self):
        sdk = head.LegacyHeadSdk()
        try:
            def check(yaw, pitch, vyaw=0, vpitch=0):
                values = [math.radians(yaw), math.radians(pitch), vyaw, vpitch]
                return sdk.request("CHECK " + " ".join(format(x, ".17g") for x in values))
            for yaw, pitch in [(-90, -20), (90, 45), (0, 37), (0, 35)]:
                with self.subTest(yaw=yaw, pitch=pitch):
                    self.assertTrue(check(yaw, pitch))
            for yaw, pitch in [(0, 45.001), (0, -20.001), (-90.001, 0), (90.001, 0), (0, math.nan), (0, math.inf)]:
                with self.subTest(yaw=yaw, pitch=pitch):
                    self.assertFalse(check(yaw, pitch))
            self.assertFalse(check(0, 37, vpitch=.181))
            # CHECK must not initialize the SDK or enable motor publications.
            self.assertFalse(sdk.request("PUB 0 0 0 0"))
        finally:
            sdk.close()

    def test_hold_drift_after_reach_keeps_publishing(self):
        # Once verified the command is frozen: drift is tolerated until Ctrl+C.
        run, clock, feedback, sdk, report = self.harness(["--execute"], clock=Clock(interrupt_after=30))
        original = sdk.publish
        def drift(q, v):
            original(q, v)
            if clock.now > 113:
                feedback.q[1] -= math.radians(4)
        sdk.publish = drift
        with self.assertRaises(KeyboardInterrupt):
            run()
        self.assertTrue(report["target_verified"])
        self.assertGreater(len(sdk.frames), 1000)
        self.assertTrue(sdk.closed)

    def test_slow_hold_creep_is_gently_pulled_back(self):
        run, clock, feedback, sdk, report = self.harness(["--execute"], clock=Clock(interrupt_after=30))
        original = sdk.publish
        offset = [0.]
        def creep(q, v):
            original(q, v)  # resets feedback to command - lag every frame
            if clock.now > 112:  # creep only after the target is reached
                offset[0] += math.radians(.01)  # ~0.5deg/s accumulated upward creep
            feedback.q[1] += offset[0]
        sdk.publish = creep
        with self.assertRaises(KeyboardInterrupt):
            run()
        self.assertTrue(report["target_verified"])
        self.assertLess(report["deadband_compensation_applied_deg"], 6.5)
        self.assertGreater(len(sdk.frames), 1000)
        self.assertTrue(sdk.closed)

    def test_loop_stall_stops_publishing(self):
        run, clock, _, sdk, _ = self.harness()
        original = clock.sleep
        def stalled_sleep(duration):
            # A 0.2s hiccup stays under the 0.45s abort threshold (and the
            # controller's own 0.5s freshness window); a 0.5s stall aborts.
            original(duration + (.5 if clock.now > 101 else 0))
        clock.sleep = stalled_sleep
        with self.assertRaisesRegex(RuntimeError, "LOOP_GAP"):
            run()
        self.assertTrue(sdk.closed)

    def test_feedback_decoder_rejects_replay_stale_and_bad_dimensions(self):
        sample = SimpleNamespace(q=[0.] * 23, header_sec=1000, header_nanosec=0)
        sample.q[21:23] = [.1, .2]
        self.assertEqual(head.decode_feedback(sample, 999, 10, 1000).head, [.1, .2])
        self.assertIsNone(head.decode_feedback(sample, 1000, 10, 1000))
        self.assertIsNone(head.decode_feedback(sample, 999, 10, 1001))
        self.assertIsNone(head.decode_feedback(sample, 999, 10, 999))
        sample.q = [0.] * 21
        self.assertIsNone(head.decode_feedback(sample, 999, 10, 1000))
        sample.q = [math.nan] * 23
        self.assertIsNone(head.decode_feedback(sample, 999, 10, 1000))

    def test_udp_config_preserves_network_and_disables_shm(self):
        original = '<CycloneDDS xmlns="https://cdds.io/config"><Domain id="0"><General><Interfaces><NetworkInterface name="eth0"/></Interfaces></General><SharedMemory><Enable>true</Enable></SharedMemory></Domain></CycloneDDS>'
        value = head.udp_only_uri(original)
        self.assertIn('name="eth0"', value)
        self.assertIn('<Enable>false</Enable>', value)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "dds config.xml"
            path.write_text(original)
            self.assertEqual(head.udp_only_uri(path.as_uri()), value)
            self.assertEqual(path.read_text(), original)

    def test_helper_protocol_timeout_and_close_without_robot(self):
        with tempfile.TemporaryDirectory() as folder:
            bridge = Path(folder) / "fake_sdk"
            bridge.write_text('#!/usr/bin/env python3\nimport sys, time\nfor line in sys.stdin:\n if line.strip() == "INIT": print("diagnostic\\nSDK_RESULT 1", flush=True)\n elif line.strip() == "CLOSE": print("SDK_RESULT 1", flush=True); break\n else: time.sleep(2)\n')
            bridge.chmod(0o755)
            with patch.dict(os.environ, HEAD_SDK_BRIDGE=str(bridge)):
                sdk = head.LegacyHeadSdk()
                sdk.initialize()
                try:
                    with self.assertRaisesRegex(RuntimeError, "REPLY_TIMEOUT"):
                        sdk.request("PUB 0 0 0 0", timeout=.03)
                finally:
                    sdk.close()
                self.assertIsNotNone(sdk.process.poll())


if __name__ == "__main__":
    unittest.main()
