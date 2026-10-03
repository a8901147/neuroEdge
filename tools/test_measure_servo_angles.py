"""measure_servo_angles.py against a fake board and a SIMULATED PERSON who reads a (made-up) real arm's link angles off
a phone inclinometer. Checks the protocol and the safety properties, not any real arm's numbers."""
import contextlib
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "mujoco_bridge"))
import gen_mearm_envelope as env  # noqa: E402
import measure_linkage_region as mlr  # noqa: E402
import measure_servo_angles as msa  # noqa: E402
import mearm_pulse_map as pm  # noqa: E402
import servo_pose_4ch as sp  # noqa: E402
from test_servo_pose_4ch import FakeBoard  # noqa: E402

ROOT = HERE.parent
TABLE = env.build_table(mlr.load_records([ROOT / "data" / n for n in env.SOURCES]))

_sleep = mock.patch.object(sp.time, "sleep", lambda s: None)


def setUpModule():
    _sleep.start()


def tearDownModule():
    _sleep.stop()


class Person:
    """Reads the arm's true link angles (a made-up arm: upper arm 55 - 0.09*(p_s-1500), forearm -35 + 0.08*(p_e-1500))
    off the fake board, like typing what a phone inclinometer shows. `noise` adds a fixed error per reading."""

    def __init__(self, board, noise=0.0, upper=(55.0, -0.09), forearm=(-35.0, 0.08)):
        self.board, self.noise, self.upper, self.forearm = board, noise, upper, forearm
        self.prompts, self.pulses_when_asked = [], []
        self.script = []                          # optional overrides, consumed first
        self.last_true = 0.0

    def __call__(self, prompt):
        self.prompts.append(prompt)
        if len(self.prompts) > 200:                 # a stuck re-prompt loop must fail the test, never hang it
            raise AssertionError("the tool kept re-prompting: " + prompt)
        if self.script:
            return self.script.pop(0)
        if "higher or lower" in prompt:
            return "h" if self.last_true >= 0 else "l"
        self.pulses_when_asked.append(tuple(self.board.pulse))
        s, e = self.board.pulse[1], self.board.pulse[2]
        link = "upper" if "Upper arm" in prompt else "forearm"
        a, k = self.upper if link == "upper" else self.forearm
        p = s if link == "upper" else e
        self.last_true = a + k * (p - 1500) + self.noise
        return f"{abs(self.last_true):.2f}"                       # what a phone app shows: no sign


def run(person, board, out_path=None, quiet=True):
    out_path = out_path or Path(tempfile.mkdtemp()) / "angles.json"
    buf = io.StringIO()
    with mock.patch("builtins.input", person), contextlib.redirect_stdout(buf):
        data = msa.measure(board, TABLE, out_path=out_path)
    return data, out_path, buf.getvalue()


class PlanTest(unittest.TestCase):
    def test_every_planned_pulse_is_inside_the_safe_envelope(self):
        for s in msa.SHOULDER_POINTS:
            self.assertEqual(env.clamp(TABLE, s, msa.SHOULDER_ELBOW_US), (s, msa.SHOULDER_ELBOW_US), s)
        for e in msa.ELBOW_POINTS:
            self.assertEqual(env.clamp(TABLE, msa.ELBOW_SHOULDER_US, e), (msa.ELBOW_SHOULDER_US, e), e)

    def test_each_part_starts_and_ends_at_rest_so_hysteresis_can_be_seen(self):
        self.assertEqual((msa.SHOULDER_POINTS[0], msa.SHOULDER_POINTS[-1]), (1500, 1500))
        self.assertEqual((msa.ELBOW_POINTS[0], msa.ELBOW_POINTS[-1]), (1500, 1500))

    def test_enough_distinct_pulses_to_fit_a_line_and_to_see_that_it_is_one(self):
        self.assertGreaterEqual(len(set(msa.SHOULDER_POINTS)), 5)
        self.assertGreaterEqual(len(set(msa.ELBOW_POINTS)), 6)

    def test_the_elbow_part_is_done_where_the_elbow_window_is_widest(self):
        i = TABLE["shoulders"].index(msa.ELBOW_SHOULDER_US)
        widths = [hi - lo for lo, hi in zip(TABLE["lo"], TABLE["hi"])]
        self.assertEqual(widths[i], max(widths))


class ProtocolTest(unittest.TestCase):
    def test_one_reading_is_asked_for_at_every_planned_point_and_saved(self):
        board = FakeBoard(start=(1500, 1500, 1500, 1300))
        data, path, _ = run(Person(board), board)
        self.assertEqual(len(data["shoulder_points"]), len(msa.SHOULDER_POINTS))
        self.assertEqual(len(data["elbow_points"]), len(msa.ELBOW_POINTS))
        saved = json.loads(path.read_text())
        self.assertEqual(saved["shoulder_points"], data["shoulder_points"])

    def test_the_recorded_pulse_is_what_the_board_reports_not_what_was_planned(self):
        board = FakeBoard(start=(1500, 1500, 1500, 1300))
        person = Person(board)
        data, _p, _o = run(person, board)
        for rec, at_ask in zip(data["shoulder_points"], [p for p in person.pulses_when_asked][:len(data["shoulder_points"])]):
            self.assertEqual(rec["pulse"], at_ask[1])

    def test_the_arm_is_never_outside_the_envelope_when_a_reading_is_taken(self):
        board = FakeBoard(start=(1500, 1500, 1500, 1300))
        person = Person(board)
        run(person, board)
        for base, s, e, claw in person.pulses_when_asked:
            self.assertEqual(env.clamp(TABLE, s, e), (s, e), (s, e))

    def test_the_arm_ends_at_rest_and_the_claw_and_base_are_left_alone(self):
        board = FakeBoard(start=(1400, 1650, 1200, 1500))
        person = Person(board)
        run(person, board)
        self.assertEqual(board.pulse, [1500, 1500, 1500, 1300])
        moved = mlr_moves(board.writes)
        first_arm = min(i for i, ch in enumerate(moved) if ch in (2, 3))
        self.assertTrue(all(ch not in (1, 4) for ch in moved[first_arm:]))

    def test_a_reading_that_is_not_a_number_is_asked_again(self):
        board = FakeBoard(start=(1500, 1500, 1500, 1300))
        person = Person(board)
        person.script = ["abc", "", "12x"]                       # three bad answers before the first good one
        data, _p, out = run(person, board)
        self.assertEqual(len(data["shoulder_points"]), len(msa.SHOULDER_POINTS))
        self.assertGreaterEqual(out.count("unrecognized"), 3)

    def test_an_impossible_angle_is_asked_again(self):
        board = FakeBoard(start=(1500, 1500, 1500, 1300))
        person = Person(board)
        person.script = ["400", "999"]                          # beyond 180: typos
        _d, _p, out = run(person, board)
        self.assertGreaterEqual(out.count("unrecognized"), 2)

    def test_a_typed_minus_sign_is_refused_with_its_own_message_and_asked_again(self):
        board = FakeBoard(start=(1500, 1500, 1500, 1300))
        person = Person(board)
        person.script = ["-40"]
        data, _p, out = run(person, board)
        self.assertIn("minus sign", out)
        self.assertEqual(len(data["shoulder_points"]), len(msa.SHOULDER_POINTS))

    def test_q_stops_saves_what_there_is_and_returns_to_rest(self):
        board = FakeBoard(start=(1500, 1500, 1500, 1300))
        person = Person(board)
        person.script = ["50", "h", "48", "h", "q"]
        data, path, _o = run(person, board)
        self.assertEqual(len(data["shoulder_points"]), 2)
        self.assertEqual(json.loads(path.read_text())["shoulder_points"], data["shoulder_points"])
        self.assertEqual(board.pulse, [1500, 1500, 1500, 1300])

    def test_results_are_saved_after_every_reading_not_only_at_the_end(self):
        board = FakeBoard(start=(1500, 1500, 1500, 1300))
        person = Person(board)
        saves = []
        real = msa.save_json

        def spy(path, data):
            saves.append(len(data["shoulder_points"]) + len(data["elbow_points"]))
            return real(path, data)

        with mock.patch.object(msa, "save_json", spy):
            run(person, board)
        self.assertEqual(saves[:4], [1, 2, 3, 4])
        self.assertEqual(saves[-1], len(msa.SHOULDER_POINTS) + len(msa.ELBOW_POINTS))

    def test_an_existing_result_file_is_never_overwritten(self):
        board = FakeBoard(start=(1500, 1500, 1500, 1300))
        p = Path(tempfile.mkdtemp()) / "keep.json"
        p.write_text("precious")
        with self.assertRaises(FileExistsError):
            run(Person(board), board, out_path=p)
        self.assertEqual(p.read_text(), "precious")
        self.assertEqual(board.writes, [])                      # and the arm was not touched

    def test_a_planned_pose_outside_the_envelope_is_refused_before_the_arm_gets_there(self):
        # the envelope allows shoulder 1800 only with the elbow at <= 1600: plan an elbow that breaks that
        board = FakeBoard(start=(1500, 1500, 1500, 1300))
        person = Person(board)
        with mock.patch.object(msa, "SHOULDER_ELBOW_US", 1700):
            with self.assertRaises(ValueError):
                run(person, board)
        for base, s, e, claw in person.pulses_when_asked:
            self.assertEqual(env.clamp(TABLE, s, e), (s, e))
        self.assertEqual(board.pulse, [1500, 1500, 1500, 1300])              # and it is back at rest afterwards

    def test_the_check_uses_where_the_other_servo_really_is_not_where_it_was_meant_to_be(self):
        # the plan believes the elbow is at rest (1500), but it is stuck at 1700 -- unsafe with the shoulder anywhere
        # near 1500 (its window there ends at 1500): the first shoulder move must be refused
        board = FakeBoard(start=(1500, 1500, 1700, 1300))
        person = Person(board)
        real_walk = mlr.walk

        def elbow_does_not_move(b, ch, target, *a, **k):
            if ch == 3:
                return mlr.Walk("reached", None, b.pulse[2])
            return real_walk(b, ch, target, *a, **k)

        with mock.patch.object(mlr, "walk", elbow_does_not_move), \
                mock.patch.object(mlr, "return_to_rest", lambda b, progress=None: None):
            with self.assertRaises(ValueError):
                run(person, board)
        self.assertEqual(person.prompts, [])                                 # it never got as far as asking for a reading

    def test_the_prompt_says_exactly_what_to_measure_and_the_sign_convention(self):
        board = FakeBoard(start=(1500, 1500, 1500, 1300))
        person = Person(board)
        run(person, board)
        first = next(p for p in person.prompts if "Upper arm" in p)
        later = next(p for p in person.prompts if "Forearm" in p)
        for text in (first, later):
            self.assertIn("horizontal", text)
            self.assertIn("no minus sign", text)                        # "no minus sign needed: the sign is asked next"


def mlr_moves(writes):
    active, out = None, []
    for c in writes:
        if c in "1234":
            active = int(c)
        elif c in "+-":
            out.append(active)
    return out


class SignTest(unittest.TestCase):
    """A phone inclinometer app shows an angle WITHOUT a sign, and the forearm crosses horizontal within the range, so the
    first real run (2026-09-27) had two readings with the wrong sign (max fit residual 23 deg). The sign now comes from what
    the person SEES -- is the far end higher or lower than the near end -- asked separately for every reading."""

    def ask(self, answers):
        it = iter(answers)
        out = []
        r = msa.ask_signed_angle("  upper arm:", "elbow", input_fn=lambda prompt: next(it), out=out.append)
        return r, out

    def test_high_means_positive_and_low_means_negative(self):
        self.assertEqual(self.ask(["55", "h"])[0], 55.0)
        self.assertEqual(self.ask(["55", "l"])[0], -55.0)
        self.assertEqual(self.ask(["55", "H"])[0], 55.0)
        self.assertEqual(self.ask(["55", " L "])[0], -55.0)

    def test_the_sign_is_never_taken_from_the_number_typed(self):
        r, out = self.ask(["-55", "55", "l"])                     # a typed minus is refused, then answered properly
        self.assertEqual(r, -55.0)
        self.assertTrue(any("minus sign" in line for line in out))

    def test_zero_is_horizontal_and_needs_no_sign(self):
        answers = iter(["0"])
        r = msa.ask_signed_angle("  ", "elbow", input_fn=lambda p: next(answers), out=lambda *_: None)
        self.assertEqual(r, 0.0)

    def test_a_bad_sign_answer_is_asked_again_not_guessed(self):
        r, out = self.ask(["55", "x", "", "up", "l"])
        self.assertEqual(r, -55.0)
        self.assertGreaterEqual(sum("unrecognized" in line for line in out), 3)

    def test_q_at_either_question_stops(self):
        self.assertIsNone(self.ask(["q"])[0])
        self.assertIsNone(self.ask(["55", "q"])[0])

    def test_the_sign_question_names_the_end_to_look_at(self):
        seen = []
        answers = iter(["55", "h"])
        msa.ask_signed_angle("  upper arm:", "elbow", input_fn=lambda p: seen.append(p) or next(answers), out=lambda *_: None)
        self.assertIn("elbow", seen[1])
        self.assertIn("higher or lower", seen[1])

    def test_a_forearm_below_horizontal_is_recorded_negative_end_to_end(self):
        board = FakeBoard(start=(1500, 1500, 1500, 1300))
        data, path, _out = run(Person(board), board)
        forearm_at_rest = next(p for p in data["elbow_points"] if p["pulse"] == 1500)
        self.assertLess(forearm_at_rest["angle_deg"], 0)                       # (the simulated arm's forearm hangs at -35)
        self.assertAlmostEqual(abs(forearm_at_rest["angle_deg"]), 35.0, places=1)

    def test_the_fit_of_a_forearm_that_crosses_horizontal_is_a_straight_line(self):
        board = FakeBoard(start=(1500, 1500, 1500, 1300))
        person = Person(board, forearm=(-20.0, 0.08))          # -100 deg at pulse 500 ... +8 deg at 1850: crosses zero
        data, _p, _o = run(person, board)
        f = msa.fit_measurements(data)["elbow"]
        self.assertLess(f["max_residual"], 0.01)
        self.assertTrue(any(p["angle_deg"] < 0 for p in data["elbow_points"]))
        self.assertTrue(any(p["angle_deg"] > 0 for p in data["elbow_points"]))     # i.e. it really crosses zero

    def test_the_typed_number_is_kept_as_it_was_given(self):
        board = FakeBoard(start=(1500, 1500, 1500, 1300))
        data, _p, _o = run(Person(board), board)
        for rec in data["elbow_points"] + data["shoulder_points"]:
            self.assertAlmostEqual(abs(rec["angle_deg"]), rec["typed_deg"], places=9)
            self.assertGreaterEqual(rec["typed_deg"], 0)


class NegativeUpperArmTest(unittest.TestCase):
    def test_an_upper_arm_below_horizontal_is_recorded_negative_and_typed_deg_stays_a_magnitude(self):
        board = FakeBoard(start=(1500, 1500, 1500, 1300))
        data, _p, _o = run(Person(board, upper=(-20.0, -0.09)), board)          # -20 deg at rest, more negative as the pulse rises
        self.assertTrue(all(p["angle_deg"] < 0 for p in data["shoulder_points"]))
        for p in data["shoulder_points"]:
            self.assertGreaterEqual(p["typed_deg"], 0)
            self.assertAlmostEqual(p["typed_deg"], -p["angle_deg"], places=9)
        f = msa.fit_measurements(data)["shoulder"]
        self.assertAlmostEqual(f["at_1500"], -20.0, places=6)


class AnalysisTest(unittest.TestCase):
    def data(self, noise=0.0, **kw):
        board = FakeBoard(start=(1500, 1500, 1500, 1300))
        person = Person(board, noise=noise, **kw)
        return run(person, board)[0]

    def test_the_true_slopes_of_the_simulated_arm_are_recovered(self):
        fits = msa.fit_measurements(self.data())
        self.assertAlmostEqual(fits["shoulder"]["slope"], -0.09, places=6)
        self.assertAlmostEqual(fits["shoulder"]["at_1500"], 55.0, places=6)
        self.assertAlmostEqual(fits["elbow"]["slope"], 0.08, places=6)
        self.assertAlmostEqual(fits["elbow"]["at_1500"], -35.0, places=6)

    def test_the_hysteresis_between_the_first_and_last_reading_at_rest_is_reported(self):
        d = self.data()
        d["shoulder_points"][-1]["angle_deg"] += 3.0
        h = msa.hysteresis(d)
        self.assertAlmostEqual(h["shoulder"], 3.0, places=6)
        self.assertAlmostEqual(h["elbow"], 0.0, places=6)

    def test_the_report_warns_about_an_implausible_slope_and_a_large_residual(self):
        d = self.data(upper=(55.0, -0.5))                       # five times too steep: a units mistake (still within +-180)
        out = []
        msa.report(d, out=out.append)
        self.assertIn("implausible", "\n".join(out))
        d = self.data()
        d["shoulder_points"][2]["angle_deg"] += 15.0            # one wildly wrong reading
        out = []
        msa.report(d, out=out.append)
        self.assertIn("exceeds", "\n".join(out))               # the WARNING (every report line mentions the residual anyway)
        clean = []
        msa.report(self.data(), out=clean.append)
        self.assertNotIn("exceeds", "\n".join(clean))             # and a clean fit does not warn
        self.assertNotIn("implausible", "\n".join(clean))

    def test_the_report_states_how_much_of_the_models_shoulder_travel_the_arm_can_reach(self):
        out = []
        msa.report(self.data(), out=out.append)
        text = "\n".join(out)
        self.assertIn("shoulder", text)
        self.assertRegex(text, r"\d+%")
        self.assertIn("1500", text)
        self.assertIn("1800", text)

    def test_fewer_readings_than_a_fit_needs_is_explained_not_a_crash(self):
        d = {"shoulder_points": [{"pulse": 1500, "angle_deg": 55.0}], "elbow_points": []}
        out = []
        msa.report(d, out=out.append)
        self.assertIn("not enough", "\n".join(out))


class OptionsTest(unittest.TestCase):
    def test_default_output_is_a_new_timestamped_file(self):
        import time
        p = msa.default_out_path(time.struct_time((2026, 9, 27, 13, 14, 15, 0, 0, -1)))
        self.assertEqual(p.name, "mearm_angles_20260927-131415.json")
        self.assertEqual(p.parent.name, "data")


if __name__ == "__main__":
    unittest.main()
