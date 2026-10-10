"""capture_arm_motion.py: the interactive recording of real arm motion (rest / slow / normal) used to choose the 1-euro
filter's parameters (include/edgeneuro/filters/vec3_one_euro.hpp). No hardware: a fake board, a fake clock and scripted
Enter presses."""
import io
import json
import math
import random
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import capture_arm_motion as cam  # noqa: E402

# a real phase3_control_loop tick line (2026-09-28): elbow_raw BEFORE shoulder_raw
REAL_TICK = ("tick=24360 grip=1.000000 gripping=1 shoulder_pitch=-1.271321 shoulder_roll=1.249607 elbow=0.919327 "
             "emg_min=3411 emg_max=3429 elbow_raw_ax=-0.144470 elbow_raw_ay=0.404846 elbow_raw_az=0.948792 "
             "shoulder_raw_ax=0.723084 shoulder_raw_ay=0.224304 shoulder_raw_az=0.701416 shoulder_raw_gx=0.106851 "
             "shoulder_raw_gy=0.025580 shoulder_raw_gz=-0.044899")


def tick(upper, fore):
    return (f"tick=1 elbow_raw_ax={fore[0]:.6f} elbow_raw_ay={fore[1]:.6f} elbow_raw_az={fore[2]:.6f} "
            f"shoulder_raw_ax={upper[0]:.6f} shoulder_raw_ay={upper[1]:.6f} shoulder_raw_az={upper[2]:.6f}\r\n")


class FakeBoard:
    """One tick line every `period` s of fake time; `motion(t)` gives (upper, fore)."""

    def __init__(self, motion, period=0.01):
        self.t, self.period, self.motion = 0.0, period, motion

    def clock(self):
        return self.t

    def read_line(self):
        self.t += self.period
        return tick(*self.motion(self.t))


def still(_t):
    return (0.99, 0.05, 0.26), (0.0, 0.6, 0.8)


def rotating(rate):
    return lambda t: ((math.sin(rate * t), 0.0, math.cos(rate * t)), (0.0, 0.6, 0.8))


class ParseTest(unittest.TestCase):
    def test_reads_both_sensors_from_a_real_firmware_line(self):
        up, fore = cam.parse_tick(REAL_TICK)
        self.assertEqual(up, (0.723084, 0.224304, 0.701416))
        self.assertEqual(fore, (-0.14447, 0.404846, 0.948792))

    def test_a_non_tick_line_gives_nothing(self):
        self.assertEqual(cam.parse_tick("diag shoulder_completions=250 elbow_completions=250\r\n"), (None, None))


class RecordTest(unittest.TestCase):
    def test_records_for_the_requested_time_with_timestamps_from_zero(self):
        board = FakeBoard(still)
        samples = cam.record_phase(board.read_line, board.clock, 2.0)
        self.assertTrue(190 <= len(samples) <= 210)
        self.assertAlmostEqual(samples[0][0], 0.01, places=6)
        self.assertLessEqual(samples[-1][0], 2.0 + 1e-9)

    def test_lines_without_sensor_data_are_skipped(self):
        lines = iter(["garbage\r\n", tick(*still(0))] * 400)
        t = [0.0]

        def clock():
            return t[0]

        def read_line():
            t[0] += 0.005
            return next(lines)

        samples = cam.record_phase(read_line, clock, 1.0)
        self.assertTrue(all(s[1] is not None for s in samples))
        self.assertTrue(90 <= len(samples) <= 110)


class SpeedTest(unittest.TestCase):
    def test_a_known_rotation_rate_is_measured_as_that_rate(self):
        # for a unit gravity vector the speed is the arm's rotation rate (rad/s) -- the 1-euro filter's input
        for rate in (0.5, 2.0):
            board = FakeBoard(rotating(rate))
            samples = cam.record_phase(board.read_line, board.clock, 5.0)
            st = cam.speed_stats(samples, "upper_arm")
            self.assertAlmostEqual(st["median"], rate, delta=0.1 * rate)

    def test_the_speed_is_that_of_the_raw_vector_like_the_filter_on_the_chip(self):
        # Vec3OneEuro works on the raw (not normalised) accel vector, so the analysis must too: a 0.5 g vector turning
        # at 2 rad/s changes at 1 g/s
        board = FakeBoard(lambda t: ((0.5 * math.sin(2.0 * t), 0.0, 0.5 * math.cos(2.0 * t)), (0.0, 0.6, 0.8)))
        samples = cam.record_phase(board.read_line, board.clock, 5.0)
        self.assertAlmostEqual(cam.speed_stats(samples, "upper_arm")["median"], 1.0, delta=0.1)

    def test_noise_is_the_sample_standard_deviation(self):
        samples = [(0.0, (1.0, 0.0, 0.0), None), (0.01, (2.0, 0.0, 0.0), None), (0.02, (3.0, 0.0, 0.0), None)]
        self.assertAlmostEqual(cam.noise_std(samples, "upper_arm"), 1.0 / 3.0)    # x: std 1, y and z: 0

    def test_a_still_arm_has_almost_no_speed_and_noise_is_reported(self):
        rng = random.Random(3)

        def noisy(_t):
            return (0.99 + rng.gauss(0, 0.005), 0.05 + rng.gauss(0, 0.005), 0.26 + rng.gauss(0, 0.005)), \
                (0.0, 0.6, 0.8)

        board = FakeBoard(noisy)
        samples = cam.record_phase(board.read_line, board.clock, 5.0)
        st = cam.speed_stats(samples, "upper_arm")
        self.assertLess(st["median"], 0.2)
        self.assertAlmostEqual(cam.noise_std(samples, "upper_arm"), 0.005, delta=0.002)


class ProtocolTest(unittest.TestCase):
    def test_it_holds_still_at_every_step_of_the_demo_task_including_while_gripping(self):
        names = [p[0] for p in cam.PHASES]
        for pose in ("hang", "forward", "left_open", "left_grip", "lifted_grip", "right_grip", "place_grip"):
            self.assertIn("hold_" + pose, names)
        self.assertTrue(any(n.startswith("task_") for n in names))

    def test_some_phases_are_kept_out_of_the_task_to_check_against_overfitting(self):
        checks = [p for p in cam.PHASES if p[0].startswith(cam.CHECK_PREFIX)]
        self.assertGreaterEqual(len(checks), 2)
        self.assertTrue(all("not in the task" in p[3] for p in checks))

    def test_resting_noise_is_reported_for_still_holds_only(self):
        board = FakeBoard(still)
        hold = cam.record_phase(board.read_line, board.clock, 1.0)
        text = "\n".join(cam.summary_lines({"hold_hang": hold, "task_1": hold}))
        hold_part, task_part = text.split("Full task")
        self.assertIn("noise", hold_part)
        self.assertNotIn("noise", task_part)


class BaseRaiseSetTest(unittest.TestCase):
    """2026-10-04: a short set to measure how much the base moves when the arm is only raised (the natural upper-arm
    twist), against an intended swing -- to set the base's "slow follow while raising" from raw data."""

    def test_raises_in_the_three_task_directions_a_swing_only_and_a_still_hold(self):
        names = [p[0] for p in cam.PHASE_SETS["base_raise"]]
        for n in ("raise_forward", "raise_left_front", "raise_right_front", "swing_only", "hold_forward"):
            self.assertIn(n, names)

    def test_a_slow_raise_is_recorded_too(self):
        # 2026-10-04 (the author's point): "raising" is decided by the direction of motion, not its speed -- checked on a
        # REAL slow raise, not only on a recording stretched in time
        slow = [p for p in cam.PHASE_SETS["base_raise"] if p[0] == "raise_slow"]
        self.assertEqual(len(slow), 1)
        self.assertIn("slowly", slow[0][3])

    def test_the_diagonal_raise_is_kept_out_as_a_check(self):
        # the author's case (lower-left -> upper-right): raising and swinging at once; checks, does not tune
        checks = [p for p in cam.PHASE_SETS["base_raise"] if p[0].startswith(cam.CHECK_PREFIX)]
        self.assertTrue(any("diagonal" in p[0] and "not in the task" in p[3] for p in checks))

    def test_the_default_set_is_the_filter_tuning_one(self):
        self.assertIs(cam.PHASE_SETS[cam.DEFAULT_SET], cam.PHASES)

    def test_the_summary_names_the_phases_of_any_set(self):
        board = FakeBoard(still)
        rec = cam.record_phase(board.read_line, board.clock, 1.0)
        text = "\n".join(cam.summary_lines({"raise_forward": rec, "hold_forward": rec}))
        self.assertIn(cam.PHASE_SETS["base_raise"][0][1], text)
        self.assertNotIn("raise_forward", text)


class SaveTest(unittest.TestCase):
    def test_the_file_round_trips_every_sample(self):
        board = FakeBoard(still)
        phases = {"rest": cam.record_phase(board.read_line, board.clock, 0.5)}
        path = Path(tempfile.mkdtemp()) / "m.json"
        cam.save(path, phases, port="/dev/fake")
        data = json.loads(path.read_text())
        self.assertEqual(data["port"], "/dev/fake")
        self.assertEqual(len(data["phases"]["rest"]), len(phases["rest"]))
        self.assertEqual(data["phases"]["rest"][0]["upper_arm"], list(phases["rest"][0][1]))

    def test_it_never_overwrites_an_existing_file(self):
        path = Path(tempfile.mkdtemp()) / "m.json"
        path.write_text("{}")
        with self.assertRaises(FileExistsError):
            cam.save(path, {}, port="x")


class InteractiveTest(unittest.TestCase):
    def run_session(self, answers, motion=still):
        board = FakeBoard(motion)
        answers = iter(answers)
        out = io.StringIO()
        with redirect_stdout(out):
            phases = cam.run_session(board.read_line, board.clock, input_fn=lambda _p="": next(answers),
                                     phases=[("rest", "still", 1.0, "do not move"), ("slow", "slow", 1.0, "turn slowly")],
                                     countdown_s=0.0)
        return phases, out.getvalue()

    def test_every_phase_is_recorded_after_its_own_enter(self):
        phases, text = self.run_session(["", "", "", ""])        # start, keep, start, keep
        self.assertEqual(list(phases), ["rest", "slow"])
        self.assertTrue(all(len(v) > 50 for v in phases.values()))
        self.assertIn("do not move", text)
        self.assertIn("turn slowly", text)

    def test_a_phase_can_be_redone(self):
        phases, text = self.run_session(["", "r", "", "", "", ""])   # rest, redo, rest again, keep, slow, keep
        self.assertEqual(list(phases), ["rest", "slow"])
        self.assertIn("Re-recording", text)

    def test_no_data_from_the_board_is_said_plainly_not_recorded_as_empty(self):
        board_lines = iter(["\r\n"] * 100000)
        t = [0.0]

        def clock():
            return t[0]

        def read_line():
            t[0] += 0.01
            return next(board_lines)

        out = io.StringIO()
        with redirect_stdout(out):
            ok = cam.wait_for_data(read_line, clock, timeout_s=2.0)
        self.assertFalse(ok)


# ---- EMG (2026-10-04): the calibration logs only hold a still arm; these record EMG while the arm moves ----

def tick_emg(emg_min, emg_max, upper=(0.99, 0.05, 0.26), fore=(0.0, 0.6, 0.8)):
    return (f"tick=1 emg_min={emg_min} emg_max={emg_max} elbow_raw_ax={fore[0]:.6f} elbow_raw_ay={fore[1]:.6f} "
            f"elbow_raw_az={fore[2]:.6f} shoulder_raw_ax={upper[0]:.6f} shoulder_raw_ay={upper[1]:.6f} "
            f"shoulder_raw_az={upper[2]:.6f}\r\n")


class EmgBoard(FakeBoard):
    """emg(t) -> (emg_min, emg_max)"""

    def __init__(self, emg, period=0.03):
        super().__init__(still, period)
        self.emg = emg

    def read_line(self):
        self.t += self.period
        return tick_emg(*self.emg(self.t))


def emg_samples(values, dt=0.03):
    """[(t, upper, fore, (v, v)), ...] -- a constant-width window around each value"""
    return [(i * dt, (0.99, 0.05, 0.26), (0.0, 0.6, 0.8), (v, v)) for i, v in enumerate(values)]


class EmgParseTest(unittest.TestCase):
    def test_the_emg_window_is_read_from_a_real_firmware_line(self):
        self.assertEqual(cam.parse_emg(REAL_TICK), (3411, 3429))

    def test_a_line_without_emg_gives_none(self):
        self.assertIsNone(cam.parse_emg(tick(*still(0))))

    def test_recorded_samples_carry_the_emg_and_it_is_saved(self):
        board = EmgBoard(lambda t: (600, 620))
        samples = cam.record_phase(board.read_line, board.clock, 0.5)
        self.assertEqual(samples[0][3], (600, 620))
        path = Path(tempfile.mkdtemp()) / "m.json"
        cam.save(path, {"relaxed_task": samples}, "fake")
        self.assertEqual(json.loads(path.read_text())["phases"]["relaxed_task"][0]["emg"], [600, 620])

    def test_the_accel_analysis_still_works_on_samples_with_emg(self):
        board = EmgBoard(lambda t: (600, 620))
        samples = cam.record_phase(board.read_line, board.clock, 2.0)
        self.assertGreater(cam.speed_stats(samples, "upper_arm")["n"], 10)


class EmgEventsTest(unittest.TestCase):
    """The grip decision replayed like the firmware's GripStateMachine: grip once the envelope stays ABOVE the grip
    threshold for 0.15 s; while gripping, release once it stays BELOW the release threshold for 0.15 s. The tick line
    carries each 10 ms window's min/max, so the compared value is approximated by their midpoint."""

    def test_a_relaxed_signal_never_grips(self):
        ev = cam.emg_events(emg_samples([600] * 100), threshold=1500, release=1000)
        self.assertEqual((ev["grips"], ev["releases"]), (0, 0))

    def test_a_short_spike_does_not_grip_but_a_sustained_one_does(self):
        short = [600] * 20 + [2000] * 3 + [600] * 20          # 0.09 s above
        long = [600] * 20 + [2000] * 10 + [600] * 20          # 0.30 s above
        self.assertEqual(cam.emg_events(emg_samples(short), 1500, 1000)["grips"], 0)
        self.assertEqual(cam.emg_events(emg_samples(long), 1500, 1000)["grips"], 1)

    def test_a_dip_between_the_two_thresholds_does_not_release(self):
        held = [2000] * 10 + [1200] * 30 + [2000] * 10        # 1200: under the grip threshold, over the release one
        ev = cam.emg_events(emg_samples(held), 1500, 1000)
        self.assertEqual((ev["grips"], ev["releases"]), (1, 0))

    def test_a_dip_below_the_release_threshold_releases(self):
        ev = cam.emg_events(emg_samples([2000] * 10 + [800] * 10), 1500, 1000)
        self.assertEqual((ev["grips"], ev["releases"]), (1, 1))

    def test_a_short_dip_below_the_release_threshold_does_not_release(self):
        ev = cam.emg_events(emg_samples([2000] * 10 + [800] * 3 + [2000] * 10), 1500, 1000)   # 0.09 s below
        self.assertEqual((ev["grips"], ev["releases"]), (1, 0))

    def test_the_window_midpoint_is_compared_not_its_peak(self):
        # a noisy relaxed window 1000..1900 (midpoint 1450) must not count as above a 1500 grip threshold
        wide = [(i * 0.03, (0.99, 0.05, 0.26), (0.0, 0.6, 0.8), (1000, 1900)) for i in range(50)]
        ev = cam.emg_events(wide, 1500, 1000)
        self.assertEqual((ev["grips"], ev["above_grip"]), (0, 0.0))

    def test_time_above_and_below_the_thresholds(self):
        ev = cam.emg_events(emg_samples([2000] * 30 + [800] * 70), 1500, 1000)
        self.assertAlmostEqual(ev["above_grip"], 0.3)
        self.assertAlmostEqual(ev["below_release"], 0.7)


class EmgProtocolTest(unittest.TestCase):
    def test_the_emg_set_records_relaxed_and_gripping_both_still_and_while_the_arm_moves(self):
        names = [p[0] for p in cam.PHASE_SETS["emg"]]
        for n in ("relaxed_still", "relaxed_task", "grip_still", "grip_task"):
            self.assertIn(n, names)
        self.assertTrue(any(n.startswith(cam.CHECK_PREFIX) for n in names))

    def test_the_summary_reports_false_grips_while_relaxed_and_false_releases_while_gripping(self):
        phases = {"relaxed_task": emg_samples([600] * 20 + [2000] * 10 + [600] * 20),
                  "grip_task": emg_samples([2000] * 10 + [800] * 10 + [2000] * 10)}
        text = "\n".join(cam.summary_lines(phases, emg_thresholds=(1500, 1000)))
        self.assertRegex(text, r"false grips[^\n]*: 1\b")
        self.assertRegex(text, r"false releases[^\n]*: 1\b")

    def test_without_thresholds_the_summary_still_shows_the_emg_levels(self):
        text = "\n".join(cam.summary_lines({"relaxed_task": emg_samples([600] * 10)}))
        self.assertIn("EMG", text)
        self.assertIn("600", text)


if __name__ == "__main__":
    unittest.main()
