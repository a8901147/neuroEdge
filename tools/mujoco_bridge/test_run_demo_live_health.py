"""The default humanoid path (run_demo_live.py) and the automatic sensor-health checks added 2026-09-28
(tools/sensor_health.py): the REAL main() runs against a fake serial port, a fake viewer and scripted Enter presses.
Only checks and warnings were added -- nothing about how the arm is computed -- so these tests are about: not starting on
faulty data, never saving a calibration capture recorded while a sensor was failing, and holding the model + warning when
a sensor fails mid-session. (The real failures behind this: SESSION_LOG 2026-09-27.)"""
import contextlib
import io
import re
import json
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_demo_live as rdl  # noqa: E402


# The golden sample: the real 2026-09-13 calibration (captured on the real board; the day the full 6-step grasp task
# first ran end to end) -- the poses below are the ones it was captured at.
GOLDEN_CALIBRATION = Path(__file__).resolve().parents[2] / "data" / "shoulder_calibration_golden_2026-09-13.json"


class fx:
    SAVED_9_13 = json.loads(GOLDEN_CALIBRATION.read_text())
    HANG = tuple(SAVED_9_13["baseline_raw"])
    FORWARD = tuple(SAVED_9_13["forward_raw"])
    LEFT = tuple(SAVED_9_13["left_twist_raw"])
    RIGHT = tuple(SAVED_9_13["right_twist_raw"])
    STRAIGHT = SAVED_9_13["zero_elbow"]


def uart_line(shoulder_raw, elbow, grip=0.0, gripping=0):
    ax, ay, az = shoulder_raw
    return (f"tick=1 grip={grip:.3f} gripping={gripping} shoulder_pitch=0.000 shoulder_roll=0.000 "
            f"elbow={elbow:.4f} shoulder_raw_ax={ax:+.4f} shoulder_raw_ay={ay:+.4f} "
            f"shoulder_raw_az={az:+.4f} shoulder_raw_gx=+0.000 shoulder_raw_gy=+0.000 "
            f"shoulder_raw_gz=+0.000 elbow_raw_ax=+0.000 elbow_raw_ay=+0.000 "
            f"elbow_raw_az=+1.000\r\n").encode()


_NOISE = __import__("random").Random(7)
_RAW_RE = re.compile(rb"(_raw_a[xyz]=)([-+]?\d+\.\d+)")


def with_noise(line):
    """Real accelerometers are never bit-for-bit constant: perturb every raw accel value a little (sensor_health treats a
    long run of identical readings as a frozen sensor -- the 2026-09-27 failure)."""
    # (an exact 0 is left alone: the firmware's all-zero "no reading yet" default is exactly zero, not noisy)
    return _RAW_RE.sub(lambda m: m.group(0) if float(m.group(2)) == 0.0 else
                       m.group(1) + f"{float(m.group(2)) + _NOISE.gauss(0, 0.003):+.4f}".encode(), line)


class FakeViewer:
    def __init__(self, ticks):
        self.ticks = ticks

    def is_running(self):
        self.ticks -= 1
        return self.ticks > 0

    def sync(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


SAVED = fx.SAVED_9_13
FROZEN_FOREARM = ("elbow_raw_ax=+0.000 elbow_raw_ay=+0.000 elbow_raw_az=+1.000",
                  "elbow_raw_ax=+1.999939 elbow_raw_ay=+0.000000 elbow_raw_az=+0.000000")


class Board:
    """A fake phase3_control_loop: streams the current pose's tick line (with sensor-like noise), accepts writes (the
    EMG threshold), and can be told to freeze the forearm like the real 2026-09-27 fault."""

    def __init__(self, raw=fx.HANG):
        self.raw, self.frozen_forearm, self.closed = raw, False, False
        self.lock = threading.Lock()
        self.reads = 0
        self.upper_nacks = 0          # drop-outs the firmware would report in its diag line (per second)
        self.upper_rewakes = 0

    def read(self, n):
        if self.closed:
            # a real port with timeout=1 BLOCKS here; returning at once would leave the finished run's reader thread
            # spinning and starve the next test
            time.sleep(0.05)
            return b""
        time.sleep(0.002)
        with self.lock:
            line = uart_line(self.raw, fx.STRAIGHT)
            self.reads += 1
            if self.reads % 400 == 0:          # ~once a second at this fake's rate, like the firmware's diag line
                self.upper_rewakes += 1 if self.upper_nacks else 0
                return (f"diag shoulder_completions=246 elbow_completions=247 shoulder_nacks={self.upper_nacks} "
                        f"shoulder_timeouts=0 elbow_nacks=0 elbow_timeouts=0 active_reader_state=0 shoulder_wake_result=0 "
                        f"elbow_wake_result=0 shoulder_required=1 elbow_required=1 bus_recovery_attempts=0 "
                        f"bus_recovery_freed=0 shoulder_asleep_rewakes={self.upper_rewakes} elbow_asleep_rewakes=0 "
                        f"shoulder_pwr_mgmt_1=1 elbow_pwr_mgmt_1=1 shoulder_power_resets=0 elbow_power_resets=0\r\n").encode()
            noisy = with_noise(line)
            if self.frozen_forearm:            # the forearm value stays bit-identical, like the real fault
                noisy = re.sub(rb"elbow_raw_ax=\S+ elbow_raw_ay=\S+ elbow_raw_az=\S+",
                               FROZEN_FOREARM[1].encode(), noisy)
                assert b"1.999939" in noisy
            return noisy

    def write(self, data):
        self.written = getattr(self, "written", b"") + bytes(data)
        return len(data)

    def close(self):
        self.closed = True


def run_main(board, argv_extra, ticks=600, inputs=None, patches=()):
    tmp = Path(tempfile.mkdtemp())
    calib = tmp / "shoulder_calibration.json"
    calib.write_text(json.dumps(SAVED))
    captured = {}

    def fake_launch(model, data):
        captured["model"], captured["data"] = model, data
        return FakeViewer(ticks)

    argv = ["run_demo_live.py", "--port", "fake", "--calibration-file", str(calib), "--skip-emg-calibration"] + argv_extra
    out = io.StringIO()
    real_sleep = time.sleep
    with contextlib.ExitStack() as st:
        st.enter_context(mock.patch.object(sys, "argv", argv))
        st.enter_context(mock.patch.object(rdl.serial, "Serial", lambda *a, **k: board))
        st.enter_context(mock.patch.object(rdl.mujoco.viewer, "launch_passive", fake_launch))
        # (the humanoid capture time, 6 s per pose, is a local in main(): left as it is -- this adds checks only)
        st.enter_context(mock.patch("builtins.input", inputs or (lambda *a: "")))
        st.enter_context(mock.patch.object(rdl.time, "sleep", lambda s: real_sleep(min(s, 0.01))))
        st.enter_context(mock.patch.object(rdl, "CAL_CONFIRM_TIMEOUT_S", 0.05))   # this fake board never confirms
        for p in patches:
            st.enter_context(p)
        st.enter_context(contextlib.redirect_stdout(out))
        rdl.main()
    m, d = captured.get("model"), captured.get("data")
    ctrl = None if m is None else {"roll": float(d.ctrl[m.actuator(rdl.SHOULDER_ROLL_ACTUATOR).id]),
                                   "pitch": float(d.ctrl[m.actuator(rdl.SHOULDER_PITCH_ACTUATOR).id])}
    return ctrl, out.getvalue(), json.loads(calib.read_text())


class StartTest(unittest.TestCase):
    def test_healthy_sensors_start_normally_and_say_so(self):
        ctrl, out, _ = run_main(Board(fx.LEFT), ["--skip-calibration"])
        self.assertIsNotNone(ctrl)
        self.assertIn("Sensors OK", out)

    def test_faulty_sensors_at_start_block_it_with_a_clear_warning(self):
        board = Board()
        board.frozen_forearm = True
        with self.assertRaises(SystemExit) as cm:
            run_main(board, ["--skip-calibration"], patches=[mock.patch.object(rdl, "HEALTH_PREFLIGHT_MAX_S", 1.0)])
        self.assertIn("forearm", str(cm.exception))
        self.assertIn("cannot be trusted", str(cm.exception))

    def test_it_continues_by_itself_once_the_sensors_are_healthy(self):
        board = Board(fx.LEFT)
        board.frozen_forearm = True
        # (kept faulty past the 1 s grace before the first warning, so the warning really has to appear)
        threading.Timer(1.5, lambda: setattr(board, "frozen_forearm", False)).start()
        ctrl, out, _ = run_main(board, ["--skip-calibration"],
                                patches=[mock.patch.object(rdl, "HEALTH_PREFLIGHT_MAX_S", 20.0),
                                         mock.patch.object(rdl, "HEALTH_REPEAT_WARNING_S", 0.1)])
        self.assertIn("HARDWARE FAULT", out)
        self.assertIsNotNone(ctrl)

    def test_a_sensor_marked_optional_is_not_counted_as_faulty(self):
        board = Board(fx.LEFT)
        board.frozen_forearm = True                   # the elbow IMU is declared absent on purpose
        ctrl, out, _ = run_main(board, ["--skip-calibration", "--optional-sensors", "elbow"],
                                patches=[mock.patch.object(rdl, "HEALTH_PREFLIGHT_MAX_S", 3.0)])
        self.assertIsNotNone(ctrl)
        self.assertNotIn("forearm MPU6050", out)


class CalibrationCaptureTest(unittest.TestCase):
    def test_a_capture_recorded_while_a_sensor_fails_is_redone_never_saved(self):
        board = Board(fx.HANG)
        poses = iter([(fx.HANG, False), (fx.FORWARD, True), (fx.LEFT, False), (fx.RIGHT, False)])

        def fake_input(*a):
            try:
                raw, freeze = next(poses)
            except StopIteration:
                return ""
            board.raw, board.frozen_forearm = raw, freeze
            if freeze:                                  # the forearm sticks for a moment, then is re-plugged
                threading.Timer(0.8, lambda: setattr(board, "frozen_forearm", False)).start()
            return ""

        _ctrl, out, saved = run_main(board, [], inputs=fake_input, ticks=50)
        self.assertIn("HARDWARE FAULT", out)
        self.assertIn("re-record", out)
        for got, want in zip(saved["forward_raw"], fx.FORWARD):
            self.assertAlmostEqual(got, want, delta=0.01)         # the redo, not the faulty recording
        self.assertNotEqual(saved["captured_at"], "2026-09-13 16:35:48")  # it did save a NEW calibration


class CalibrationToBoardTest(unittest.TestCase):
    """2026-10-03 (the author's choice "B", "the humanoid arm the same way"): after its calibration -- loaded with
    --skip-calibration or captured interactively -- the humanoid path also SENDS it to the board, so a servos-ON board
    uses the very calibration the person just made. How the humanoid arm itself is computed is unchanged."""

    def test_skip_calibration_sends_the_saved_calibration_to_the_board(self):
        import mearm_calibration_link as link
        board = Board(fx.LEFT)
        run_main(board, ["--skip-calibration"], ticks=50)
        self.assertIn(link.encode(SAVED), board.written)

    def test_a_fresh_interactive_calibration_is_sent_too(self):
        import mearm_calibration_link as link
        board = Board(fx.HANG)
        poses = iter([(fx.HANG, False), (fx.FORWARD, True), (fx.LEFT, False), (fx.RIGHT, False)])

        def fake_input(*_a):
            raw, _ = next(poses, (fx.HANG, False))
            board.raw = raw
            return ""
        _ctrl, _out, saved = run_main(board, [], inputs=fake_input, ticks=50)
        self.assertIn(link.encode(saved), board.written)


class RuntimeTest(unittest.TestCase):
    def test_a_fault_mid_session_holds_the_arm_and_warns(self):
        board = Board(fx.LEFT)

        def swing_right_with_a_frozen_forearm():
            with board.lock:
                board.raw, board.frozen_forearm = fx.RIGHT, True
        threading.Timer(1.2, swing_right_with_a_frozen_forearm).start()
        ctrl, out, _ = run_main(board, ["--skip-calibration"], ticks=1400)
        self.assertIn("HARDWARE FAULT", out)
        self.assertGreater(ctrl["roll"], 0.0)          # still on the LEFT side: the RIGHT swing was not followed

    def test_it_recovers_and_follows_again(self):
        board = Board(fx.LEFT)

        def fault():
            with board.lock:
                board.raw, board.frozen_forearm = fx.RIGHT, True

        def recover():
            with board.lock:
                board.frozen_forearm = False
        threading.Timer(1.0, fault).start()
        threading.Timer(1.6, recover).start()
        ctrl, out, _ = run_main(board, ["--skip-calibration"], ticks=2600)
        self.assertIn("back to normal", out)
        self.assertLess(ctrl["roll"], 0.0)             # now following the RIGHT arm



class HardwareWarningTest(unittest.TestCase):
    """2026-09-28: a brief drop-out the value checks cannot see (the data looks fine again a moment later) -- the firmware
    reports it (no I2C answer, re-woken) and the operator must be told, while the arm keeps following (the data is usable)."""

    def test_dropouts_reported_by_the_firmware_are_shown_and_the_arm_keeps_following(self):
        board = Board(fx.LEFT)

        def flaky():
            with board.lock:
                board.upper_nacks = 3                   # the upper arm drops out a few times each second, like 9/28

        def swing_right():
            with board.lock:
                board.raw = fx.RIGHT
        threading.Timer(1.0, flaky).start()
        threading.Timer(1.5, swing_right).start()
        ctrl, out, _ = run_main(board, ["--skip-calibration"], ticks=2600)
        self.assertIn("upper arm", out)
        self.assertIn("dropped out", out)
        self.assertLess(ctrl["roll"], 0.0)              # it FOLLOWED the swing to the right: not held


class RaceBoard(Board):
    """Follows LEFT until told the loop is running, then sends ONE faulty line (arm swung RIGHT, forearm frozen) and holds
    the next, healthy line back until the test releases it."""

    def __init__(self):
        super().__init__(fx.LEFT)
        self.loop_running, self.armed, self.bad_stored, self.released, self.good_stored = (
            threading.Event() for _ in range(5))

    def read(self, n):
        if self.loop_running.is_set() and not self.armed.is_set():
            self.armed.set()
            with self.lock:
                self.raw, self.frozen_forearm = fx.RIGHT, True
                if (self.reads + 1) % 400 == 0:
                    self.reads += 1                    # make sure this read is the tick line, not a diag line
            return super().read(n)                     # the FAULTY line
        if self.armed.is_set() and not self.released.is_set():
            self.bad_stored.set()                      # the reader came back: the faulty line is stored
            self.released.wait(2.0)
            with self.lock:
                self.raw, self.frozen_forearm = fx.LEFT, False
            return super().read(n)                     # the next, HEALTHY line
        if self.released.is_set():
            self.good_stored.set()
        return super().read(n)


class OneFramePerStepTest(unittest.TestCase):
    """2026-10-08: a step computed the arm from the line it read first but checked the forearm of a line read LATER, so a
    faulty line was applied whenever a healthy one arrived in between. Forced deterministically: the healthy line is
    handed over between the step's first read and its health check (inside latest.status(), called in between)."""

    def test_the_arm_never_uses_a_line_whose_forearm_was_not_checked(self):
        import math
        board = RaceBoard()
        used = []                                          # the upper-arm vector each step went on to use
        real_select, real_status = rdl.select_raw_smoothing_alpha, rdl.LatestSample.status

        def right_like(v):
            return v[0] is not None and math.dist(v, fx.RIGHT) < math.dist(v, fx.LEFT)

        def spy_select(gripping):                          # called once the step's health check has passed
            used.append(sys._getframe(1).f_locals["shoulder_raw"])
            if len(used) >= 50:
                board.loop_running.set()
            return real_select(gripping)

        def racing_status(latest):
            step_raw = sys._getframe(1).f_locals.get("shoulder_raw")
            if (board.bad_stored.is_set() and not board.released.is_set() and step_raw is not None
                    and right_like(step_raw)):             # this step has read the faulty line: let the healthy one in
                board.released.set()
                board.good_stored.wait(2.0)
            return real_status(latest)

        run_main(board, ["--skip-calibration"], ticks=1500,
                 patches=[mock.patch.object(rdl, "select_raw_smoothing_alpha", spy_select),
                          mock.patch.object(rdl.LatestSample, "status", racing_status)])
        self.assertTrue(board.released.is_set(), "the race was never forced -- the test proved nothing")
        self.assertFalse([v for v in used if right_like(v)], "a line with a frozen forearm was used unchecked")


if __name__ == "__main__":
    unittest.main()
