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


class fx:
    """The real 2026-09-13 calibration (captured on the real board) and the poses it was captured at."""
    SAVED_9_13 = {
        "baseline_raw": [0.9926369238095238, 0.04678664761904762, 0.26501993333333335],
        "forward_raw": [0.03340870754716981, -0.010959839622641509, 1.0248781886792453],
        "left_twist_raw": [0.12286084761904763, 0.4518007904761905, 0.9183839047619048],
        "right_twist_raw": [0.11225231428571429, -0.4611189333333333, 0.8999546761904761],
        "zero_elbow": 0.4340121238095238,
    }
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


SAVED = dict(fx.SAVED_9_13, emg_threshold=1129, captured_at="2026-09-13 16:35:48")
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
        self.assertIn("感測器狀態正常", out)

    def test_faulty_sensors_at_start_block_it_with_a_clear_warning(self):
        board = Board()
        board.frozen_forearm = True
        with self.assertRaises(SystemExit) as cm:
            run_main(board, ["--skip-calibration"], patches=[mock.patch.object(rdl, "HEALTH_PREFLIGHT_MAX_S", 1.0)])
        self.assertIn("前臂", str(cm.exception))
        self.assertIn("不可信", str(cm.exception))

    def test_it_continues_by_itself_once_the_sensors_are_healthy(self):
        board = Board(fx.LEFT)
        board.frozen_forearm = True
        # (kept faulty past the 1 s grace before the first warning, so the warning really has to appear)
        threading.Timer(1.5, lambda: setattr(board, "frozen_forearm", False)).start()
        ctrl, out, _ = run_main(board, ["--skip-calibration"],
                                patches=[mock.patch.object(rdl, "HEALTH_PREFLIGHT_MAX_S", 20.0),
                                         mock.patch.object(rdl, "HEALTH_REPEAT_WARNING_S", 0.1)])
        self.assertIn("硬體異常", out)
        self.assertIsNotNone(ctrl)

    def test_a_sensor_marked_optional_is_not_counted_as_faulty(self):
        board = Board(fx.LEFT)
        board.frozen_forearm = True                   # the elbow IMU is declared absent on purpose
        ctrl, out, _ = run_main(board, ["--skip-calibration", "--optional-sensors", "elbow"],
                                patches=[mock.patch.object(rdl, "HEALTH_PREFLIGHT_MAX_S", 3.0)])
        self.assertIsNotNone(ctrl)
        self.assertNotIn("前臂 MPU6050", out)


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
        self.assertIn("硬體異常", out)
        self.assertIn("重錄", out)
        for got, want in zip(saved["forward_raw"], fx.FORWARD):
            self.assertAlmostEqual(got, want, delta=0.01)         # the redo, not the faulty recording
        self.assertNotEqual(saved["captured_at"], "2026-09-13 16:35:48")  # it did save a NEW calibration


class RuntimeTest(unittest.TestCase):
    def test_a_fault_mid_session_holds_the_arm_and_warns(self):
        board = Board(fx.LEFT)

        def swing_right_with_a_frozen_forearm():
            with board.lock:
                board.raw, board.frozen_forearm = fx.RIGHT, True
        threading.Timer(1.2, swing_right_with_a_frozen_forearm).start()
        ctrl, out, _ = run_main(board, ["--skip-calibration"], ticks=1400)
        self.assertIn("硬體異常", out)
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
        self.assertIn("恢復", out)
        self.assertLess(ctrl["roll"], 0.0)             # now following the RIGHT arm



class HardwareWarningTest(unittest.TestCase):
    """2026-09-28: a brief drop-out the value checks cannot see (the data looks fine again a moment later) -- the firmware
    reports it (no I2C answer, re-woken) and the user must be told, while the arm keeps following (the data is usable)."""

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
        self.assertIn("上臂", out)
        self.assertIn("斷線", out)
        self.assertLess(ctrl["roll"], 0.0)              # it FOLLOWED the swing to the right: not held


if __name__ == "__main__":
    unittest.main()
