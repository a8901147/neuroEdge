"""watch_imu_raw.py: the plain raw view of both MPU6050s (no verdicts mixed into the numbers), built on the REAL firmware
output captured 2026-09-28."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import watch_imu_raw as w  # noqa: E402

REAL_TICK = ("tick=9030 grip=1.000000 gripping=1 shoulder_pitch=-0.404962 shoulder_roll=-1.478542 elbow=0.466365 "
             "emg_min=3596 emg_max=3608 elbow_raw_ax=-0.058594 elbow_raw_ay=0.541443 elbow_raw_az=-0.830444 "
             "shoulder_raw_ax=0.039124 shoulder_raw_ay=0.120117 shoulder_raw_az=-0.981812 shoulder_raw_gx=0.078207 "
             "shoulder_raw_gy=0.015188 shoulder_raw_gz=-0.045832")
REAL_DIAG = ("diag shoulder_completions=24 elbow_completions=40 shoulder_nacks=0 shoulder_timeouts=17 elbow_nacks=0 "
             "elbow_timeouts=0 active_reader_state=7 shoulder_wake_result=0 elbow_wake_result=0 shoulder_required=1 "
             "elbow_required=1 bus_recovery_attempts=451 bus_recovery_freed=451 shoulder_asleep_rewakes=443 "
             "elbow_asleep_rewakes=0 shoulder_pwr_mgmt_1=1 elbow_pwr_mgmt_1=1 shoulder_power_resets=0 "
             "elbow_power_resets=0")


class SummaryTest(unittest.TestCase):
    def summary(self, ticks, diag=None, seconds=1.0):
        s = w.Window()
        for t in ticks:
            s.add_line(t)
        if diag:
            s.add_line(diag)
        return s.summary(seconds)

    def test_the_real_tick_line_gives_both_sensors_raw_values_and_magnitude(self):
        row = self.summary([REAL_TICK])
        self.assertEqual(row["upper_arm"]["last"], (0.039124, 0.120117, -0.981812))
        self.assertEqual(row["forearm"]["last"], (-0.058594, 0.541443, -0.830444))
        self.assertAlmostEqual(row["upper_arm"]["g"], 0.9895, places=3)
        self.assertEqual(row["lines_per_s"], 1.0)

    def test_distinct_values_and_the_spread_show_whether_the_sensor_is_alive(self):
        a = REAL_TICK
        b = REAL_TICK.replace("shoulder_raw_ax=0.039124", "shoulder_raw_ax=0.041000")
        row = self.summary([a, a, b])
        self.assertEqual(row["upper_arm"]["samples"], 3)
        self.assertEqual(row["upper_arm"]["distinct"], 2)
        self.assertEqual(row["forearm"]["distinct"], 1)

    def test_the_firmwares_own_counters_are_shown_as_reported(self):
        row = self.summary([REAL_TICK], REAL_DIAG)
        self.assertEqual(row["hw"]["upper_arm"]["timeouts"], 17)
        self.assertEqual(row["hw"]["upper_arm"]["rewakes"], 443)
        self.assertEqual(row["hw"]["forearm"]["timeouts"], 0)

    def test_the_printed_line_has_the_numbers_first_and_the_verdict_last(self):
        text = w.format_row(self.summary([REAL_TICK], REAL_DIAG), verdict="upper arm: dropped out and came back")
        self.assertIn("+0.039", text)
        self.assertIn("-0.059", text)
        self.assertIn("timeouts=17", text)
        self.assertLess(text.index("+0.039"), text.index("upper arm: dropped out"))

    def test_no_data_in_a_window_says_so_instead_of_crashing(self):
        row = self.summary([])
        self.assertIsNone(row["upper_arm"]["last"])
        self.assertIn("no data", w.format_row(row, verdict=None))


if __name__ == "__main__":
    unittest.main()
