"""Headless end-to-end test of calibrate_mearm_alignment.run_flow() using a
scripted fake sensor and a mocked input() -- no hardware, no viewer.

Asserts PROPERTIES of the result (the fitted map reproduces each captured
pose's target model pose; a 'no' answer flips the elbow anchors; a
too-small pose is retried; unrelated calibration keys survive and a backup
is written), not specific numeric values from any one real calibration, so
re-measuring never fights this test. The pose vectors below are only
plausible stand-ins shaped like this project's real saved calibration.
"""

import contextlib
import io
import json
import sys
import tempfile
import time
import types
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import calibrate_mearm_alignment as cma  # noqa: E402
import mearm_pathb as pb  # noqa: E402
import run_demo_live as rdl  # noqa: E402
import test_mearm_direction as fx  # noqa: E402  (sweep helpers, same 9/13-shaped vectors)

HANG = (0.9926, 0.0468, 0.2650)
FORWARD = (0.0334, -0.0110, 1.0249)
LEFT = (0.1229, 0.4518, 0.9184)
RIGHT = (0.1123, -0.4611, 0.8999)


def _mid():
    v = tuple(a + b for a, b in zip(rdl._normalize3(HANG), rdl._normalize3(FORWARD)))
    n = rdl._normalize3(v)
    return tuple(1.02 * c for c in n)


MID = _mid()
ELBOW_STRAIGHT = 0.434
ELBOW_FLEXED = 2.30


class FakeLatest:
    """A live-looking board. Readings get a 1e-11 perturbation per call: a real sensor is never bit-identical from one
    sample to the next (sensor_health calls a long identical run 'frozen'), and 1e-11 changes nothing else the tests
    compare. `frozen_forearm` reproduces the real 2026-09-27 fault (1.999939, 0, 0), bit-identical."""

    def __init__(self):
        self.raw = HANG
        self.elbow = ELBOW_STRAIGHT
        self.grip = 0.0
        self.frozen_forearm = False
        self._tick = 0

    @property
    def last_update_monotonic(self):                 # a new sample on every poll
        self._tick += 1
        return self._tick

    def is_ready(self):
        return True

    def _jitter(self, v):
        self._tick += 1
        e = 1e-11 * (1 + self._tick % 7)
        return tuple(c + e for c in v)

    def snapshot_shoulder_raw(self):
        return self._jitter(self.raw) if self.raw[0] is not None else self.raw

    def snapshot_elbow_raw(self):
        return (1.999939, 0.0, 0.0) if self.frozen_forearm else self._jitter((0.0, 0.0, 1.0))

    def snapshot(self):
        return self.grip, 0.0, 0.0, self.elbow

    stale = False
    port_error = None

    def status(self):
        return self.stale, self.port_error


class FrozenForearmPose:
    """The forearm sensor sticks (the real 9/27 fault) during this capture."""

    def __init__(self, raw, elbow):
        self.raw, self.elbow = raw, elbow


class StalePose:
    def __init__(self, raw, elbow):
        self.raw, self.elbow = raw, elbow


class UnplugPose(StalePose):
    pass


def run_scripted_flow(pose_sequence, yes_no_answers, calib_contents=None):
    """pose_sequence: list of (raw, elbow) applied at each 'press Enter'
    prompt, in order (retries consume extra entries). Returns
    (saved_json, live_fn, printed_output, calib_dir_listing)."""
    tmp = Path(tempfile.mkdtemp())
    calib = tmp / "shoulder_calibration.json"
    if calib_contents is not None:
        calib.write_text(json.dumps(calib_contents))
    fake = FakeLatest()
    state = cma.PoseState()
    poses = list(pose_sequence)
    answers = list(yes_no_answers)

    def fake_input(prompt=""):
        if "(y/n)" in prompt:
            return answers.pop(0)
        fake.stale, fake.port_error, fake.frozen_forearm = False, None, False
        item = poses.pop(0)
        if isinstance(item, FrozenForearmPose):     # the forearm sensor sticks during this capture
            fake.raw, fake.elbow = item.raw, item.elbow
            fake.frozen_forearm = True
        elif isinstance(item, UnplugPose):         # the adapter is pulled during this capture
            fake.raw, fake.elbow = item.raw, item.elbow
            fake.port_error = "device disconnected"
        elif isinstance(item, StalePose):          # the link goes quiet during this capture
            fake.raw, fake.elbow = item.raw, item.elbow
            fake.stale = True
        else:
            fake.raw, fake.elbow = item
        return ""

    real_sleep = time.sleep
    out = io.StringIO()
    with mock.patch("builtins.input", fake_input), \
            mock.patch.object(cma.time, "sleep", lambda s: real_sleep(min(s, 0.005))), \
            mock.patch.object(cma, "RECORD_SECONDS", 0.12), \
            mock.patch.object(cma, "SETTLE_TAIL_SECONDS", 0.07), \
            contextlib.redirect_stdout(out):
        cma.run_flow(state, fake, types.SimpleNamespace(calibration_file=calib))
    mode, _target, live_fn, _abort = state.snapshot()
    assert mode == "live"
    return json.loads(calib.read_text()), live_fn, fake, out.getvalue(), sorted(p.name for p in tmp.iterdir())


GOOD_POSES = [
    (HANG, ELBOW_STRAIGHT), (FORWARD, ELBOW_STRAIGHT), (LEFT, ELBOW_STRAIGHT),
    (RIGHT, ELBOW_STRAIGHT), (HANG, ELBOW_FLEXED), (MID, ELBOW_STRAIGHT),
]


class AlignmentFlowTest(unittest.TestCase):
    def _live_at(self, live_fn, fake, raw, elbow):
        fake.raw, fake.elbow = raw, elbow
        return live_fn()

    def test_fitted_map_reproduces_each_captured_poses_target(self):
        saved, live, fake, out, _ = run_scripted_flow(GOOD_POSES, ["y", "y"])
        base, shoulder, elbow, _claw = self._live_at(live, fake, HANG, ELBOW_STRAIGHT)
        self.assertAlmostEqual(base, 0.0, places=6)
        self.assertAlmostEqual(shoulder, cma.SH_LOW_ELEV, places=6)
        # elbow = the measured end of its range, fitted into the linkage window at this shoulder
        self.assertAlmostEqual(elbow, pb.elbow_window(shoulder)[0], places=6)
        self.assertAlmostEqual(self._live_at(live, fake, FORWARD, ELBOW_STRAIGHT)[1], cma.SH_HIGH_ELEV, places=6)
        self.assertAlmostEqual(self._live_at(live, fake, LEFT, ELBOW_STRAIGHT)[0], cma.BASE_SWING, places=6)
        self.assertAlmostEqual(self._live_at(live, fake, RIGHT, ELBOW_STRAIGHT)[0], -cma.BASE_SWING, places=6)
        _b, sh, el, _c = self._live_at(live, fake, HANG, ELBOW_FLEXED)
        self.assertAlmostEqual(el, pb.elbow_window(sh)[1], places=6)

    def test_held_out_mid_raise_passes_and_lands_between_the_ends(self):
        _saved, live, fake, out, _ = run_scripted_flow(GOOD_POSES, ["y", "y"])
        self.assertIn("PASS", out)
        shoulder = self._live_at(live, fake, MID, ELBOW_STRAIGHT)[1]
        self.assertLess(cma.SH_HIGH_ELEV, shoulder)
        self.assertLess(shoulder, cma.SH_LOW_ELEV)

    def test_never_touches_the_humanoid_paths_calibration_keys(self):
        # Path A (--humanoid) owns these keys; this tool must leave every one
        # of them EXACTLY as it found them (explicit requirement 2026-09-25).
        old = {"emg_threshold": 1129, "baseline_raw": [1, 0, 0], "forward_raw": [0, 0, 1],
               "left_twist_raw": [0, 1, 0], "right_twist_raw": [0, -1, 0], "zero_elbow": 0.1,
               "captured_at": "old", "emg_threshold_captured_at": "old-emg"}
        saved, _live, _f, _out, listing = run_scripted_flow(GOOD_POSES, ["y", "y"], calib_contents=old)
        for key, value in old.items():
            self.assertEqual(saved[key], value, f"{key} was modified")
        self.assertIn("mearm_alignment", saved)
        self.assertTrue(any(name.startswith("shoulder_calibration.json.bak-") for name in listing),
                        f"no backup among {listing}")

    def test_own_captures_are_stored_inside_the_alignment_key(self):
        saved, _live, _f, _out, _l = run_scripted_flow(GOOD_POSES, ["y", "y"])
        cap = saved["mearm_alignment"]["captures"]
        for key, expected in (("hang_raw", HANG), ("forward_raw", FORWARD),
                              ("left_twist_raw", LEFT), ("right_twist_raw", RIGHT)):
            for got, want in zip(cap[key], expected):
                self.assertAlmostEqual(got, want, places=9)
        self.assertAlmostEqual(cap["hang_elbow"], ELBOW_STRAIGHT, places=6)

    def test_answering_no_to_fold_flips_only_the_elbow(self):
        _y, live_y, fake_y, _, _ = run_scripted_flow(GOOD_POSES, ["y", "y"])
        _n, live_n, fake_n, _, _ = run_scripted_flow(GOOD_POSES, ["n", "y"])
        for raw in (HANG, FORWARD, LEFT, RIGHT, MID):
            with self.subTest(raw=raw):
                by, sy, _e, _c = self._live_at(live_y, fake_y, raw, ELBOW_STRAIGHT)
                bn, sn, _e, _c = self._live_at(live_n, fake_n, raw, ELBOW_STRAIGHT)
                self.assertAlmostEqual(by, bn, places=6)
                self.assertAlmostEqual(sy, sn, places=6)
        # straight -> the FAR end of the window when the answer was 'no'
        sh = self._live_at(live_n, fake_n, HANG, ELBOW_STRAIGHT)[1]
        self.assertAlmostEqual(self._live_at(live_n, fake_n, HANG, ELBOW_STRAIGHT)[2], pb.elbow_window(sh)[1], places=6)
        self.assertAlmostEqual(self._live_at(live_n, fake_n, HANG, ELBOW_FLEXED)[2], pb.elbow_window(sh)[0], places=6)

    def test_a_pure_swing_about_the_hang_axis_does_not_move_the_shoulder(self):
        # The oblique decode leaked twist into 'pitch' (right swing -> shoulder
        # ~85% raised, SESSION_LOG 2026-09-24). Path B's spherical decode does
        # not; the alignment tool must use it too. Constant-tilt sweep, tolerance
        # 15% of the shoulder's travel.
        _s, live, fake, _o, _l = run_scripted_flow(GOOD_POSES, ["y", "y"])
        span = pb.SHOULDER_REST - pb.SHOULDER_RAISED
        shoulders = [self._live_at(live, fake, fx._sweep_about_hang(FORWARD, a), ELBOW_STRAIGHT)[1]
                     for a in (-0.7, -0.35, 0.0, 0.35, 0.7)]
        self.assertLess(max(shoulders) - min(shoulders), 0.15 * span)

    def test_the_saved_alignment_says_which_decode_it_was_fitted_with(self):
        saved, _l, _f, _o, _ls = run_scripted_flow(GOOD_POSES, ["y", "y"])
        self.assertEqual(saved["mearm_alignment"]["decode"], "spherical")

    def test_a_right_twist_on_the_wrong_side_is_retried(self):
        poses = GOOD_POSES[:3] + [(LEFT, ELBOW_STRAIGHT)] + GOOD_POSES[3:]   # 2nd RIGHT attempt = LEFT again
        _s, live, fake, out, _ = run_scripted_flow(poses, ["y", "y"])
        self.assertIn("右甩", out)
        self.assertAlmostEqual(self._live_at(live, fake, RIGHT, ELBOW_STRAIGHT)[0], -cma.BASE_SWING, places=6)

    def test_claw_answer_selects_which_end_is_closed(self):
        _s, live_y, fake, _o, _l = run_scripted_flow(GOOD_POSES, ["y", "y"])     # high end = closed
        fake.grip = 1.0
        self.assertAlmostEqual(live_y()[3], cma.CLAW_HIGH, places=6)
        _s, live_n, fake, _o, _l = run_scripted_flow(GOOD_POSES, ["y", "n"])     # high end = open
        fake.grip = 1.0
        self.assertAlmostEqual(live_n()[3], cma.CLAW_LOW, places=6)
        fake.grip = 0.0
        self.assertAlmostEqual(live_n()[3], cma.CLAW_HIGH, places=6)

    def test_too_small_forward_raise_is_retried_not_accepted(self):
        poses = [(HANG, ELBOW_STRAIGHT), (HANG, ELBOW_STRAIGHT)] + GOOD_POSES[1:]   # 2nd = a FORWARD that never moved
        _s, live, fake, out, _ = run_scripted_flow(poses, ["y", "y"])
        self.assertIn("動作太小", out)
        self.assertAlmostEqual(self._live_at(live, fake, FORWARD, ELBOW_STRAIGHT)[1], cma.SH_HIGH_ELEV, places=6)

    def test_gives_up_after_repeated_bad_poses_instead_of_looping_forever(self):
        poses = [(HANG, ELBOW_STRAIGHT)] * (1 + cma.MAX_RETRIES)      # FORWARD never moves
        with self.assertRaises(RuntimeError):
            run_scripted_flow(poses, ["y", "y"])


class CaptureRobustnessTest(unittest.TestCase):
    def test_data_that_goes_stale_during_a_capture_is_never_used(self):
        # A frozen link keeps returning the LAST sample, which looks perfectly valid
        # (here: a plausible, far-from-HANG vector). Averaging it would silently save
        # a garbage calibration. The stalled attempt must be redone instead.
        poses = [GOOD_POSES[0], StalePose(LEFT, ELBOW_STRAIGHT)] + GOOD_POSES[1:]
        saved, _live, _f, out, _l = run_scripted_flow(poses, ["y", "y"])
        cap = saved["mearm_alignment"]["captures"]
        for got, want in zip(cap["forward_raw"], FORWARD):
            self.assertAlmostEqual(got, want, places=6)
        self.assertIn("中斷", out)

    def test_repeated_stalls_give_up_instead_of_looping_forever(self):
        poses = [GOOD_POSES[0]] + [StalePose(FORWARD, ELBOW_STRAIGHT)] * cma.MAX_RETRIES
        with self.assertRaises(RuntimeError):
            run_scripted_flow(poses, ["y", "y"])

    def test_unplugging_the_adapter_mid_capture_stops_with_a_clear_error(self):
        poses = [GOOD_POSES[0], UnplugPose(FORWARD, ELBOW_STRAIGHT)] + GOOD_POSES[1:]
        with self.assertRaises(RuntimeError) as cm:
            run_scripted_flow(poses, ["y", "y"])
        self.assertIn("serial port failed", str(cm.exception))
        self.assertIn("device disconnected", str(cm.exception))

    def test_nothing_is_saved_when_the_flow_aborts(self):
        tmp = Path(tempfile.mkdtemp())
        calib = tmp / "shoulder_calibration.json"
        calib.write_text(json.dumps({"emg_threshold": 1129}))
        fake, state = FakeLatest(), cma.PoseState()
        poses = [GOOD_POSES[0], UnplugPose(FORWARD, ELBOW_STRAIGHT)]

        def fake_input(prompt=""):
            fake.stale, fake.port_error = False, None
            item = poses.pop(0)
            if isinstance(item, UnplugPose):
                fake.port_error = "device disconnected"
                fake.raw, fake.elbow = item.raw, item.elbow
            else:
                fake.raw, fake.elbow = item
            return ""

        real_sleep = time.sleep
        with mock.patch("builtins.input", fake_input), \
                mock.patch.object(cma.time, "sleep", lambda s: real_sleep(min(s, 0.005))), \
                mock.patch.object(cma, "RECORD_SECONDS", 0.12), mock.patch.object(cma, "SETTLE_TAIL_SECONDS", 0.07), \
                contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            cma.flow_thread_main(state, fake, types.SimpleNamespace(calibration_file=calib))
        self.assertTrue(state.snapshot()[3], "the flow's failure must request an abort")
        self.assertEqual(json.loads(calib.read_text()), {"emg_threshold": 1129})       # untouched
        self.assertEqual([p.name for p in tmp.iterdir()], ["shoulder_calibration.json"])   # no backup either


class WaitHintTest(unittest.TestCase):
    def test_a_missing_upper_arm_vector_prints_a_hint_instead_of_waiting_silently(self):
        class Latest(FakeLatest):                     # (live-looking readings, so the health check can pass)
            def __init__(self):
                super().__init__()
                self.t0 = time.monotonic()

            def snapshot_shoulder_raw(self):
                # firmware's all-zero default for 0.3 s, then real data
                return (0.0, 0.0, 0.0) if time.monotonic() - self.t0 < 0.3 else self._jitter(HANG)

        out = io.StringIO()
        with mock.patch.object(cma, "WAIT_HINT_SECONDS", 0.05), contextlib.redirect_stdout(out):
            cma.wait_for_valid_data(Latest())
        self.assertIn("MPU6050", out.getvalue())


class SensorHealthDuringCalibrationTest(unittest.TestCase):
    """2026-09-28: the 9/27 calibration was recorded while a sensor was failing, and it silently saved garbage. A faulty
    sensor must stop a capture (redo the pose, with a clear warning), and must keep the tool from starting at all."""

    def test_a_sensor_failing_during_a_capture_is_never_saved_the_pose_is_redone(self):
        poses = [GOOD_POSES[0], FrozenForearmPose(FORWARD, ELBOW_STRAIGHT)] + GOOD_POSES[1:]
        saved, _live, _f, out, _l = run_scripted_flow(poses, ["y", "y"])
        for got, want in zip(saved["mearm_alignment"]["captures"]["forward_raw"], FORWARD):
            self.assertAlmostEqual(got, want, places=6)          # the redo, not the faulty attempt
        self.assertIn("硬體異常", out)
        self.assertIn("前臂", out)

    def test_repeated_faulty_captures_give_up_instead_of_saving(self):
        poses = [GOOD_POSES[0]] + [FrozenForearmPose(FORWARD, ELBOW_STRAIGHT)] * cma.MAX_RETRIES
        with self.assertRaises(RuntimeError):
            run_scripted_flow(poses, ["y", "y"])

    def test_the_tool_does_not_start_on_faulty_sensors(self):
        fake = FakeLatest()
        fake.frozen_forearm = True
        out = io.StringIO()
        with mock.patch.object(cma, "HEALTH_PREFLIGHT_MAX_S", 0.5), mock.patch.object(cma, "WAIT_HINT_SECONDS", 0.05), \
                contextlib.redirect_stdout(out):
            with self.assertRaises(RuntimeError) as cm:
                cma.wait_for_valid_data(fake)
        self.assertIn("前臂", str(cm.exception))

    def test_it_starts_by_itself_once_the_sensors_are_healthy(self):
        fake = FakeLatest()
        fake.frozen_forearm = True
        import threading
        threading.Timer(0.3, lambda: setattr(fake, "frozen_forearm", False)).start()
        out = io.StringIO()
        with mock.patch.object(cma, "HEALTH_PREFLIGHT_MAX_S", 10.0), mock.patch.object(cma, "HEALTH_REPEAT_WARNING_S", 0.1), \
                contextlib.redirect_stdout(out):
            cma.wait_for_valid_data(fake)                        # returns: no exception
        self.assertIn("硬體異常", out.getvalue())


class MainLoopTest(unittest.TestCase):
    """cma.main() with a fake serial port, a fake viewer and a stubbed-out interactive
    flow (run_flow itself is covered above): the sim loop, abort, live-follow and
    port-failure handling."""

    def _run_main(self, flow, serial_factory, ticks=400):
        import test_run_mearm_preview as prev
        tmp = Path(tempfile.mkdtemp())
        captured = {}

        def fake_launch(model, data):
            captured["model"], captured["data"] = model, data
            return prev.FakeViewer(ticks)

        out = io.StringIO()
        argv = ["calibrate_mearm_alignment.py", "--port", "fake",
                "--calibration-file", str(tmp / "shoulder_calibration.json")]
        with mock.patch.object(sys, "argv", argv), \
                mock.patch.object(cma.serial, "Serial", lambda *a, **k: serial_factory()), \
                mock.patch.object(cma.mujoco.viewer, "launch_passive", fake_launch), \
                mock.patch.object(cma, "flow_thread_main", flow), \
                contextlib.redirect_stdout(out):
            cma.main()
        m, d = captured["model"], captured["data"]
        return {n: float(d.ctrl[m.actuator(n).id]) for n in ("base", "shoulder", "elbow", "claw")}, out.getvalue()

    def _good_serial(self):
        import test_run_mearm_preview as prev
        return prev.FakeSerial(prev.uart_line(HANG, ELBOW_STRAIGHT))

    def test_an_aborted_flow_ends_the_loop_with_a_message(self):
        _ctrl, out = self._run_main(lambda state, latest, args: state.request_abort(), self._good_serial)
        self.assertIn("校正中止", out)

    def test_the_model_shows_the_target_pose_until_the_flow_goes_live_then_follows_it(self):
        def flow(state, latest, args):
            state.show(0.0, cma.SH_HIGH_ELEV, cma.EL_FOLDED, cma.CLAW_HIGH)
            time.sleep(0.15)
            state.go_live(lambda: (0.3, 0.5, 1.5, 0.7))

        ctrl, out = self._run_main(flow, self._good_serial, ticks=1500)
        self.assertAlmostEqual(ctrl["base"], 0.3, places=1)          # position servos settle near ctrl
        self.assertAlmostEqual(ctrl["shoulder"], 0.5, places=1)
        self.assertIn("model ctrl:", out)

    def test_an_unplugged_adapter_stops_the_tool_with_a_clear_message(self):
        import serial
        import test_run_mearm_preview as prev

        def script(n):
            return prev.uart_line(HANG, ELBOW_STRAIGHT) if n < 30 else serial.SerialException("device disconnected")

        with self.assertRaises(SystemExit) as cm:
            self._run_main(lambda state, latest, args: time.sleep(5),
                           lambda: prev.ScriptedSerial(script), ticks=5000)
        self.assertIn("serial port failed", str(cm.exception))


if __name__ == "__main__":
    unittest.main()
