"""check_hardware_ready.py's verdicts, without hardware: the I2C scan must expect BOTH MPU6050s (on 2026-09-27 it printed
'All checks passed' with only 0x69 answering), and --sensors must judge live data with sensor_health."""
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_hardware_ready as chr_  # noqa: E402


class ScanVerdictTest(unittest.TestCase):
    def test_both_imus_answering_passes(self):
        ok, lines = chr_.evaluate_scan([0x68, 0x69], busy_before=0, scan_done=1)
        self.assertTrue(ok)

    def test_one_imu_missing_fails_and_says_which(self):
        for found, missing, where in (([0x69], "0x68", "上臂"), ([0x68], "0x69", "前臂")):
            ok, lines = chr_.evaluate_scan(found, busy_before=0, scan_done=1)
            self.assertFalse(ok, found)
            text = "\n".join(lines)
            self.assertIn(missing, text)
            self.assertIn(where, text)

    def test_an_extra_device_is_reported_but_does_not_fail(self):
        ok, lines = chr_.evaluate_scan([0x27, 0x68, 0x69], busy_before=0, scan_done=1)
        self.assertTrue(ok)
        self.assertIn("0x27", "\n".join(lines))

    def test_nothing_answering_or_a_busy_bus_or_an_unfinished_scan_fails(self):
        self.assertFalse(chr_.evaluate_scan([], busy_before=0, scan_done=1)[0])
        self.assertFalse(chr_.evaluate_scan([0x68, 0x69], busy_before=2, scan_done=1)[0])
        self.assertFalse(chr_.evaluate_scan([0x68, 0x69], busy_before=0, scan_done=0)[0])


# The REAL firmware's tick line, captured 2026-09-28 -- note the forearm (elbow_raw) comes BEFORE the upper arm
# (shoulder_raw). The first version of --sensors assumed the opposite order, passed its tests with a made-up line, and
# found "no data at all" on the real board.
REAL_TICK = ("tick=9030 grip=1.000000 gripping=1 shoulder_pitch=-0.404962 shoulder_roll=-1.478542 elbow=0.466365 "
             "emg_min=3596 emg_max=3608 elbow_raw_ax=-0.058594 elbow_raw_ay=0.541443 elbow_raw_az=-0.830444 "
             "shoulder_raw_ax=0.039124 shoulder_raw_ay=0.120117 shoulder_raw_az=-0.981812 shoulder_raw_gx=0.078207 "
             "shoulder_raw_gy=0.015188 shoulder_raw_gz=-0.045832")


def line(u, f, tick):
    """A tick line in the real firmware's exact field order (REAL_TICK)."""
    return (f"tick={tick} grip=0.000000 gripping=0 shoulder_pitch=0.000000 shoulder_roll=0.000000 elbow=0.400000 "
            f"emg_min=3596 emg_max=3608 elbow_raw_ax={f[0]:.6f} elbow_raw_ay={f[1]:.6f} elbow_raw_az={f[2]:.6f} "
            f"shoulder_raw_ax={u[0]:.6f} shoulder_raw_ay={u[1]:.6f} shoulder_raw_az={u[2]:.6f} "
            f"shoulder_raw_gx=0.000000 shoulder_raw_gy=0.000000 shoulder_raw_gz=0.000000")


def stream(upper, fore, seconds=3.0, completions=(250, 250)):
    """(read_line, clock) faking the board at 100 Hz with a diag line once a second."""
    rng = random.Random(1)
    state = {"t": 0.0, "i": 0}

    def clock():
        return state["t"]

    def read_line():
        state["t"] += 0.01
        state["i"] += 1
        if state["i"] % 100 == 0:
            return (f"diag shoulder_completions={completions[0]} elbow_completions={completions[1]} "
                    f"shoulder_nacks=0 shoulder_timeouts=0 elbow_nacks=0 elbow_timeouts=0")
        u = upper(state["i"]) if callable(upper) else tuple(c + rng.gauss(0, 0.004) for c in upper)
        f = fore(state["i"]) if callable(fore) else tuple(c + rng.gauss(0, 0.004) for c in fore)
        return line(u, f, state["i"])
    return read_line, clock


class SensorCheckTest(unittest.TestCase):
    HANG, FORE = (0.998, 0.004, 0.243), (0.95, 0.05, 0.30)

    def test_healthy_sensors_pass(self):
        ok, lines = chr_.run_sensor_check(*stream(self.HANG, self.FORE))
        self.assertTrue(ok, lines)

    def test_the_real_frozen_forearm_fails_and_names_it(self):
        ok, lines = chr_.run_sensor_check(*stream(self.HANG, lambda i: (1.999939, 0.0, 0.0), completions=(250, 246)))
        self.assertFalse(ok)
        self.assertIn("前臂", "\n".join(lines))

    def test_the_real_dropped_upper_arm_fails_and_names_it(self):
        ok, lines = chr_.run_sensor_check(*stream(lambda i: (0.179, 0.057, 0.093), self.FORE, completions=(0, 250)))
        self.assertFalse(ok)
        self.assertIn("上臂", "\n".join(lines))

    def test_dropouts_with_correct_data_pass_with_a_note_naming_the_sensor(self):
        # (the 9/28 upper arm: 353 re-wakes since boot, yet its readings looked normal between drop-outs)
        read_line, clock = stream(self.HANG, self.FORE)

        def flaky():
            text = read_line()
            if text.startswith("diag"):
                n = int(clock() // 1)
                return text + f" shoulder_asleep_rewakes={350 + 3 * n} elbow_asleep_rewakes=0 " \
                              f"shoulder_pwr_mgmt_1=1 elbow_pwr_mgmt_1=1 shoulder_power_resets=0 elbow_power_resets=0"
            return text
        ok, lines = chr_.run_sensor_check(flaky, clock)
        # option A (user, 2026-09-28): the data is right, so it passes -- but the note names the flaky sensor
        text = "\n".join(lines)
        self.assertTrue(ok, lines)
        self.assertIn("[NOTE]", text)
        self.assertIn("上臂", text)
        self.assertIn("斷線", text)
        self.assertIn("麵包板", text)                              # the user's wording: it may be the breadboard

    def test_the_real_firmware_line_is_parsed_into_both_sensors(self):
        up, fore = chr_.parse_tick_raw(REAL_TICK)
        self.assertEqual(up, (0.039124, 0.120117, -0.981812))
        self.assertEqual(fore, (-0.058594, 0.541443, -0.830444))
        self.assertEqual(chr_.parse_tick_raw("diag shoulder_completions=1"), (None, None))

    def test_a_board_sending_data_slowly_passes_but_says_so(self):
        # 2026-09-28: 57 lines in 2.5 s (instead of ~250) while the bus kept recovering from a failing upper arm
        read_line, clock = stream(self.HANG, self.FORE)
        state = {"n": 0}

        def sparse():
            text = read_line()
            state["n"] += 1
            return text if state["n"] % 5 == 0 or text.startswith("diag") else ""
        ok, lines = chr_.run_sensor_check(sparse, clock)
        # a slow rate alone does not fail the check for a prototype demo (user, 2026-09-28) -- it is only mentioned
        self.assertTrue(ok, lines)
        self.assertIn("較慢", "\n".join(lines))

    def test_with_sparse_data_it_waits_for_a_diag_line_and_names_the_flaky_sensor(self):
        # the real 9/28 case: the firmware's clock slows while the bus keeps recovering, so its "once a second" diag line
        # arrives only every few real seconds -- the check must wait for one before it judges
        real_diag = ("diag shoulder_completions=24 elbow_completions=40 shoulder_nacks=0 shoulder_timeouts=17 "
                     "elbow_nacks=0 elbow_timeouts=0 active_reader_state=7 shoulder_wake_result=0 elbow_wake_result=0 "
                     "shoulder_required=1 elbow_required=1 bus_recovery_attempts=451 bus_recovery_freed=451 "
                     "shoulder_asleep_rewakes=443 elbow_asleep_rewakes=0 shoulder_pwr_mgmt_1=1 elbow_pwr_mgmt_1=1 "
                     "shoulder_power_resets=0 elbow_power_resets=0")
        state = {"t": 0.0, "i": 0}
        clock = lambda: state["t"]

        def read_line():
            state["t"] += 0.05                                   # 20 lines a second
            state["i"] += 1
            if abs(state["t"] - 5.0) < 0.026:                    # the first diag line only after 5 s
                return real_diag
            e = 0.002 * ((state["i"] * 7) % 5 - 2)               # live sensors are noisy (identical = frozen)
            return line(tuple(c + e for c in self.HANG), tuple(c - e for c in self.FORE), state["i"])
        ok, lines = chr_.run_sensor_check(read_line, clock)
        text = "\n".join(lines)
        self.assertTrue(ok, lines)                               # data right -> pass (option A) ...
        self.assertIn("上臂", text)                              # ... with the upper arm named in the note
        self.assertIn("斷線", text)                              # the hardware's own account: upper arm time-outs
        self.assertGreater(clock(), 4.9)                         # it kept reading past the 3 s until the diag line

    def test_a_failure_lists_only_what_caused_it(self):
        # (the real 9/28 output mixed the incidental "too little data" lines into the failure: confusing)
        read_line, clock = stream(self.HANG, lambda i: (1.999939, 0.0, 0.0))
        state = {"n": 0}

        def sparse():
            state["n"] += 1
            text = read_line()
            return text if state["n"] % 3 == 0 or text.startswith("diag") else ""
        ok, lines = chr_.run_sensor_check(sparse, clock)
        text = "\n".join(lines)
        self.assertFalse(ok)
        self.assertIn("前臂", text)
        self.assertNotIn("資料太少", text)

    def test_a_silent_board_fails_with_a_hint_about_the_firmware(self):
        state = {"t": 0.0}

        def clock():
            return state["t"]

        def read_line():
            state["t"] += 0.5
            return ""
        ok, lines = chr_.run_sensor_check(read_line, clock, seconds=3.0)
        self.assertFalse(ok)
        self.assertIn("phase3_control_loop", "\n".join(lines))



class PortInUseTest(unittest.TestCase):
    def test_a_port_that_another_program_is_also_reading_gives_a_message_not_a_traceback(self):
        # 2026-09-28, real: "device reports readiness to read but returned no data (device disconnected or multiple access
        # on port?)" while MuJoCo / watch_imu_raw.py were still open in another terminal
        import io
        import contextlib
        from unittest import mock
        import serial

        class Busy:
            def __init__(self, *a, **k):
                pass

            def readline(self):
                raise serial.SerialException("device reports readiness to read but returned no data "
                                             "(device disconnected or multiple access on port?)")

            def close(self):
                pass
        out = io.StringIO()
        with mock.patch.object(serial, "Serial", Busy), contextlib.redirect_stdout(out):
            ok = chr_.check_sensors("/dev/fake")
        self.assertFalse(ok)
        self.assertIn("其他程式", out.getvalue())

if __name__ == "__main__":
    unittest.main()
