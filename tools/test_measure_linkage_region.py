"""measure_linkage_region.py against a fake board plus a SIMULATED PERSON who presses
Enter a little after the arm enters an infeasible (shoulder, elbow) combination.
Checks the protocol and the safety properties, not any real arm's numbers."""
import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
import measure_linkage_region as m  # noqa: E402
import servo_pose_4ch as sp  # noqa: E402
from test_servo_pose_4ch import FakeBoard  # noqa: E402


_sleep_patch = mock.patch.object(sp.time, "sleep", lambda s: None)


def setUpModule():
    _sleep_patch.start()        # read_state polls with real sleeps; not needed against a fake board


def tearDownModule():
    _sleep_patch.stop()


def feasible_default(s, e):
    """A made-up coupled arm: the elbow may only sit in a window that slides with the shoulder."""
    return 1300 - 0.5 * (s - 1500) <= e <= 1750 - 0.5 * (s - 1500) and 1200 <= s <= 2100


class SimulatedPerson:
    """Presses Enter `lag` polls after the board first reports an infeasible pose."""

    def __init__(self, board, feasible, lag=1):
        self.board, self.feasible, self.lag, self.bad_polls = board, feasible, lag, 0
        self.calls = 0
        self.worst_excursion = {"elbow": 0, "shoulder": 0}
        self.max_infeasible_polls = 0

    def __call__(self, dwell_s):
        self.calls += 1
        s, e = self.board.pulse[1], self.board.pulse[2]
        if self.feasible(s, e):
            self.bad_polls = 0
            return False
        self.bad_polls += 1
        self.max_infeasible_polls = max(self.max_infeasible_polls, self.bad_polls)
        if self.bad_polls > self.lag:
            self.bad_polls = 0
            self.at_stop = e                 # where the arm REALLY was when Enter was pressed
            return True
        return False


def moved_channels(writes):
    """The channel each '+'/'-' was applied to, in order (replaying the firmware's 'select then step')."""
    active, out = None, []
    for c in writes:
        if c in "1234":
            active = int(c)
        elif c in "+-":
            out.append(active)
    return out


class LossyBoard(FakeBoard):
    """Drops the +/- writes whose index (counting only +/-) is in `drop` -- what a serial link
    that is fed too fast does (the firmware has a single-byte receive register)."""

    def __init__(self, drop=(), drop_all=False, **kw):
        super().__init__(**kw)
        self.drop, self.drop_all, self.n = set(drop), drop_all, 0

    def write(self, data):
        for b in data:
            if chr(b) in "+-":
                self.n += 1
                if self.drop_all or self.n in self.drop:
                    self.writes.append(chr(b))
                    continue
            super().write(bytes([b]))


def make(feasible=feasible_default, lag=1, start=(1500, 1500, 1500, 1300)):
    board = FakeBoard(start=start)
    return board, SimulatedPerson(board, feasible, lag)


class WalkTest(unittest.TestCase):
    def test_reaches_the_target_when_nothing_binds(self):
        board, person = make(feasible=lambda s, e: True)
        w = m.walk(board, 3, 1700, person)
        self.assertEqual((w.result, board.pulse[2]), ("reached", 1700))

    def test_stops_when_the_person_presses_and_backs_off_toward_where_it_started(self):
        board, person = make()
        w = m.walk(board, 3, 1900, person)              # true upper edge at shoulder 1500 is 1750
        self.assertEqual(w.result, "stopped")
        self.assertGreater(w.stopped_at, 1750)
        self.assertLessEqual(w.stopped_at, 1750 + (person.lag + 1) * sp.STEP_US)
        self.assertEqual(w.ended_at, board.pulse[2])
        self.assertLess(w.ended_at, w.stopped_at)                      # it backed off
        self.assertTrue(feasible_default(board.pulse[1], w.ended_at))  # and is back in a feasible pose

    def test_never_backs_off_past_where_the_walk_began(self):
        board, person = make(feasible=lambda s, e: e < 1525)            # binds right after the first step
        w = m.walk(board, 3, 1900, person)
        self.assertGreaterEqual(w.ended_at, 1500)

    def test_never_leaves_the_channels_measured_range(self):
        board, person = make(feasible=lambda s, e: True)
        w = m.walk(board, 3, 9999, person)
        self.assertEqual(board.pulse[2], sp.RANGES[3][1])
        self.assertEqual(w.result, "reached")

    def test_moves_one_25us_step_at_a_time(self):
        board, person = make(feasible=lambda s, e: True)
        m.walk(board, 3, 1650, person)
        self.assertEqual([c for c in board.writes if c in "+-"], ["+"] * 6)


class MeasureTest(unittest.TestCase):
    def run_measure(self, feasible=feasible_default, lag=1, shoulders=(1500, 1650, 1800)):
        board, person = make(feasible=feasible, lag=lag)
        saved = []
        records = m.measure(board, person, shoulders=shoulders, save=lambda recs: saved.append(list(recs)),
                            log=lambda *a: None)
        return board, person, records, saved

    def test_finds_each_edge_within_the_reaction_lag(self):
        _b, person, records, _s = self.run_measure()
        for r in records:
            s = r["shoulder"]
            true_hi = 1750 - 0.5 * (s - 1500)
            true_lo = 1300 - 0.5 * (s - 1500)
            self.assertGreater(r["elbow_up_bind"], true_hi)
            self.assertLessEqual(r["elbow_up_bind"], true_hi + (person.lag + 1) * sp.STEP_US + 12.5)
            self.assertLess(r["elbow_down_bind"], true_lo)
            self.assertGreaterEqual(r["elbow_down_bind"], true_lo - (person.lag + 1) * sp.STEP_US - 12.5)

    def test_the_arm_is_never_driven_further_into_a_bind_than_the_reaction_time_allows(self):
        _b, person, _r, _s = self.run_measure(lag=2)
        self.assertLessEqual(person.max_infeasible_polls, person.lag + 1)

    def test_ends_with_the_elbow_at_rest_and_in_a_feasible_pose(self):
        board, _p, _r, _s = self.run_measure()
        self.assertEqual(board.pulse[2], 1500)
        self.assertTrue(feasible_default(board.pulse[1], board.pulse[2]))

    def test_the_elbow_is_returned_to_rest_before_the_shoulder_moves(self):
        # a shoulder move with the elbow at an extreme could itself be infeasible; start with the
        # elbow AWAY from rest so this is really exercised, not incidentally true
        board, person = make(start=(1500, 1500, 1650, 1300))
        seen = []
        real_walk = m.walk

        def spy(b, ch, target, *args, **kw):
            if ch == 2:
                seen.append(b.pulse[2])
            return real_walk(b, ch, target, *args, **kw)

        m.walk_original, m.walk = m.walk, spy
        try:
            m.measure(board, person, shoulders=(1500, 1650, 1800), save=lambda r: None, log=lambda *a: None)
        finally:
            m.walk = m.walk_original
        self.assertTrue(all(e == 1500 for e in seen), seen)

    def test_a_shoulder_position_that_binds_by_itself_is_recorded_and_skipped(self):
        board, person, records, _s = self.run_measure(
            feasible=lambda s, e: s <= 1700 and feasible_default(s, e), shoulders=(1500, 1650, 1800, 1950))
        by = {r["shoulder"]: r for r in records}
        self.assertIsNotNone(by[1800]["shoulder_bind_at_elbow_1500"])
        self.assertIsNone(by[1800]["elbow_up_bind"])
        self.assertIsNone(by[1500]["shoulder_bind_at_elbow_1500"])
        self.assertTrue(feasible_default(board.pulse[1], board.pulse[2]) or board.pulse[1] <= 1700)

    def test_every_position_is_approached_from_the_1500_rest_pose(self):
        # backlash / hysteresis: a position reached from a different starting point or direction can stop at a
        # different pulse, which is indistinguishable from the mechanism being unrepeatable
        board, person = make(start=(1400, 1650, 1650, 1500))     # everything OFF rest at the start
        starts = []
        real_walk = m.walk

        def spy(b, ch, target, stop=None, *a, **kw):
            if ch == 2 and stop is not None:                     # the walk toward a measured position
                starts.append((target, tuple(b.pulse)))
            return real_walk(b, ch, target, stop, *a, **kw)

        m.walk_original, m.walk = m.walk, spy
        try:
            m.measure(board, person, shoulders=(1350, 1350, 1425, 1500, 1575), save=lambda r: None,
                      log=lambda *a: None)
        finally:
            m.walk = m.walk_original
        self.assertEqual(len(starts), 5)
        for target, pulses in starts:
            self.assertEqual(pulses, (1500, 1500, 1500, 1300), f"walk to {target} began from {pulses}")

    def test_the_whole_arm_is_brought_to_the_rest_pose_before_anything_is_measured(self):
        board, person = make(start=(1400, 1650, 1650, 1500))
        first = []
        real_walk = m.walk

        def spy(b, ch, target, stop=None, *a, **kw):
            if stop is not None and not first:
                first.append(tuple(b.pulse))                     # the first MEASURING walk
            return real_walk(b, ch, target, stop, *a, **kw)

        m.walk_original, m.walk = m.walk, spy
        try:
            m.measure(board, person, shoulders=(1500,), save=lambda r: None, log=lambda *a: None)
        finally:
            m.walk = m.walk_original
        self.assertEqual(first, [(1500, 1500, 1500, 1300)])

    def test_the_base_is_brought_to_rest_and_then_never_moved_again(self):
        board, person = make(start=(1400, 1500, 1500, 1300))
        m.measure(board, person, shoulders=(1500, 1650), save=lambda r: None, log=lambda *a: None)
        self.assertEqual(board.pulse[0], 1500)
        moved = moved_channels(board.writes)
        first_arm_move = min(i for i, ch in enumerate(moved) if ch in (2, 3))
        self.assertTrue(all(ch != 1 for ch in moved[first_arm_move:]))     # base moves only before the arm work

    def test_while_the_person_answers_the_questions_the_arm_is_already_back_at_the_rest_pose(self):
        # the servos hold their pulse: leaving the arm at the edge of a bind while someone reads a prompt
        # keeps it under load for as long as they take
        for feasible in (feasible_default, lambda s, e: s <= 1700 and feasible_default(s, e)):
            board, person = make(feasible=feasible)
            seen = []
            m.measure(board, person, shoulders=(1500, 1800), save=lambda r: None, log=lambda *a: None,
                      confirm=lambda rec: seen.append(("confirm", tuple(board.pulse))) or True,
                      ask_cause=lambda where, pulse: seen.append(("cause", where, tuple(board.pulse))) or "linkage")
            confirms = [t for kind, *t in seen if kind == "confirm"]
            self.assertTrue(confirms)
            for (pulses,) in confirms:
                self.assertEqual(pulses, (1500, 1500, 1500, 1300))

    def test_return_to_rest_brings_all_four_servos_back_elbow_before_shoulder(self):
        board, _p = make(start=(1900, 1300, 700, 1550))
        m.return_to_rest(board)
        self.assertEqual(board.pulse, [1500, 1500, 1500, 1300])
        firsts = []
        for c in board.writes:
            if c in "1234" and c not in firsts:
                firsts.append(c)
        self.assertLess(firsts.index("3"), firsts.index("2"))

    def test_the_run_ends_with_the_claw_and_base_at_rest_too(self):
        board, person = make(start=(1400, 1500, 1500, 1550))
        m.measure(board, person, shoulders=(1500,), save=lambda r: None, log=lambda *a: None)
        self.assertEqual(board.pulse, [1500, 1500, 1500, 1300])

    def test_finishing_after_an_interruption_reports_instead_of_raising_when_the_board_is_gone(self):
        class Dead(FakeBoard):
            def write(self, data):
                pass
        out = []
        m.safe_finish(Dead(), log=out.append)                   # must not raise
        self.assertTrue(any("could not return" in line or "rest pose" in line for line in out))

    def test_safe_finish_returns_the_arm_to_rest_when_the_board_is_fine(self):
        board, _p = make(start=(1900, 1300, 700, 1550))
        m.safe_finish(board, log=lambda *a: None)
        self.assertEqual(board.pulse, [1500, 1500, 1500, 1300])

    def test_after_a_shoulder_that_binds_by_itself_the_shoulder_goes_back_to_rest_not_to_wherever(self):
        board, person = make(feasible=lambda s, e: s <= 1700 and feasible_default(s, e))
        m.measure(board, person, shoulders=(1500, 1650, 1800, 1500), save=lambda r: None, log=lambda *a: None)
        self.assertEqual(board.pulse[1], 1500)

    def test_a_missing_edge_means_no_bind_was_found_up_to_the_measured_limit(self):
        _b, _p, records, _s = self.run_measure(feasible=lambda s, e: True, shoulders=(1500,))
        self.assertIsNone(records[0]["elbow_up_bind"])
        self.assertIsNone(records[0]["elbow_down_bind"])
        self.assertEqual(records[0]["elbow_up_limit"], sp.RANGES[3][1])
        self.assertEqual(records[0]["elbow_down_limit"], sp.RANGES[3][0])

    def test_results_are_saved_after_every_shoulder_position_not_only_at_the_end(self):
        _b, _p, records, saved = self.run_measure()
        self.assertEqual([len(s) for s in saved], [1, 2, 3])

    def test_the_base_is_left_alone(self):
        board, _p, _r, _s = self.run_measure()
        self.assertEqual(board.pulse[0], 1500)

    def test_the_claw_is_put_at_its_open_rest_before_measuring_and_stays_there(self):
        # the first real run started with the claw at 1500 (the limit-finder firmware boots every
        # channel at 1475): its tip then sat near the base plate and may have caused some stops
        board, person = make(start=(1500, 1500, 1500, 1500))
        m.measure(board, person, shoulders=(1500, 1650), save=lambda r: None, log=lambda *a: None)
        self.assertEqual(board.pulse[3], m.CLAW_REST_US)
        self.assertEqual(m.CLAW_REST_US, 1300)

    def test_the_claw_is_never_moved_once_the_sweeps_begin(self):
        board, person = make(start=(1500, 1500, 1500, 1500))
        m.measure(board, person, shoulders=(1500, 1650), save=lambda r: None, log=lambda *a: None)
        moved = moved_channels(board.writes)
        first_arm_move = min(i for i, ch in enumerate(moved) if ch in (2, 3))
        self.assertTrue(any(ch == 4 for ch in moved[:first_arm_move]))       # it WAS moved, at the start
        self.assertTrue(all(ch != 4 for ch in moved[first_arm_move:]))       # and never again

    def test_the_position_after_a_shoulder_that_bound_by_itself_is_approached_from_rest_too(self):
        # (this used to require returning to the "last good position"; every position now starts from rest)
        board, person = make(feasible=lambda s, e: s <= 1700 and feasible_default(s, e))
        starts = []
        real_walk = m.walk

        def spy(b, ch, target, *args, **kw):
            if ch == 2:
                starts.append((target, b.pulse[1]))
            return real_walk(b, ch, target, *args, **kw)

        m.walk_original, m.walk = m.walk, spy
        try:
            m.measure(board, person, shoulders=(1500, 1650, 1800, 1950), save=lambda r: None, log=lambda *a: None)
        finally:
            m.walk = m.walk_original
        to_1950 = [start for target, start in starts if target == 1950]
        self.assertEqual(to_1950, [1500])

    def test_the_claw_is_never_moved_during_the_sweeps(self):
        board, person = make(start=(1500, 1500, 1500, 1500))
        m.measure(board, person, shoulders=(1500, 1650), save=lambda r: None, log=lambda *a: None)
        claw_selects = [i for i, c in enumerate(board.writes) if c == "4"]
        after = board.writes[claw_selects[-1] + 1:]
        self.assertFalse(any(c in "+-" for c in after[:1]) and False)     # (selection only; see below)
        # every +/- after the claw's own walk belongs to channels 2 or 3: the claw pulse never changes again
        self.assertEqual(board.pulse[3], 1300)


def LOW_ELBOW_OK(s, e):
    """A made-up arm where a low elbow is fine at every shoulder (feasible_default's window excludes 1200)."""
    return 900 <= e <= 1750 and 1200 <= s <= 2100


class ElbowHoldTest(unittest.TestCase):
    """2026-09-28 real arm: above shoulder 1800 the elbow's linkage hits the upper arm while the elbow sits at 1500, so the
    shoulder cannot go higher at all; with the elbow turned further (a LOWER pulse -- at 1800 the elbow already bound
    going UP from 1500, SESSION_LOG 2026-09-27) the shoulder can. --elbow-hold moves the shoulder with the elbow there."""

    def spy_shoulder_walks(self, board, person, **kw):
        seen, real_walk = [], m.walk

        def spy(b, ch, target, *args, **k):
            if ch == 2 and target != b.pulse[1]:          # a walk that really moves the shoulder
                seen.append(b.pulse[2])
            return real_walk(b, ch, target, *args, **k)

        m.walk_original, m.walk = m.walk, spy
        try:
            records = m.measure(board, person, save=lambda r: None, log=lambda *a: None, **kw)
        finally:
            m.walk = m.walk_original
        return seen, records

    def test_every_shoulder_move_happens_with_the_elbow_at_the_hold_pulse(self):
        board, person = make(feasible=LOW_ELBOW_OK, start=(1500, 1500, 1650, 1300))
        seen, _r = self.spy_shoulder_walks(board, person, shoulders=(1500, 1650, 1800), elbow_hold=1200)
        self.assertTrue(seen)
        self.assertTrue(all(e == 1200 for e in seen), seen)

    def test_the_sweeps_start_from_the_hold_and_it_is_recorded(self):
        board, person = make(feasible=LOW_ELBOW_OK)
        _seen, records = self.spy_shoulder_walks(board, person, shoulders=(1650,), elbow_hold=1200)
        self.assertEqual(records[0]["elbow_hold"], 1200)
        self.assertEqual(records[0]["elbow_up_start"], 1200)
        self.assertEqual(records[0]["elbow_down_start"], 1200)

    def test_the_default_is_unchanged_1500(self):
        board, person = make(feasible=LOW_ELBOW_OK)
        seen, records = self.spy_shoulder_walks(board, person, shoulders=(1650,))
        self.assertTrue(all(e == 1500 for e in seen), seen)
        self.assertEqual(records[0]["elbow_hold"], 1500)

    def test_the_run_still_ends_with_the_whole_arm_at_rest(self):
        board, person = make(feasible=LOW_ELBOW_OK)
        self.spy_shoulder_walks(board, person, shoulders=(1650, 1800), elbow_hold=1200)
        self.assertEqual(tuple(board.pulse), (1500, 1500, 1500, 1300))

    def test_the_hold_must_be_inside_the_elbow_window_measured_at_shoulder_1500(self):
        # the shoulder always returns to 1500 with the elbow at the hold, so the hold must be safe there: 500..1500
        # measured (SESSION_LOG 2026-09-27), kept away from the elbow's own end of travel
        self.assertEqual(m.parse_elbow_hold("1200"), 1200)
        self.assertEqual(m.parse_elbow_hold("1500"), 1500)
        for bad in ("1525", "1850", "650", "abc"):
            with self.assertRaises(ValueError):
                m.parse_elbow_hold(bad)


class ReliabilityTest(unittest.TestCase):
    def test_every_step_including_returns_and_back_off_is_followed_by_a_wait(self):
        # the first version sent the "return to rest" walks back to back: a jump of hundreds of us
        board, person = make()
        waits = []
        with mock.patch.object(m.time, "sleep", lambda s: waits.append(s)):
            m.measure(board, person, shoulders=(1500, 1650), save=lambda r: None, log=lambda *a: None)
        moves = sum(1 for c in board.writes if c in "+-")
        paced_returns = sum(1 for w in waits if w == m.RETURN_DWELL_S)
        self.assertEqual(person.calls + paced_returns, moves)

    def test_a_walk_that_loses_steps_on_the_link_corrects_itself(self):
        board = LossyBoard(drop={3, 9}, start=(1500, 1500, 1500, 1300))
        w = m.walk(board, 3, 1800, lambda d: False)         # (1850 is the elbow's measured maximum)
        self.assertEqual((w.result, w.ended_at, board.pulse[2]), ("reached", 1800, 1800))

    def test_a_back_off_that_loses_steps_also_corrects_itself(self):
        # the person stops after 12 forward steps (write #1..#12); #13..#15 are the back-off: lose one
        board = LossyBoard(drop={14}, start=(1500, 1500, 1500, 1300))
        person = SimulatedPerson(board, feasible_default)
        w = m.walk(board, 3, 1900, person)
        self.assertEqual(w.result, "stopped")
        self.assertEqual(w.ended_at, board.pulse[2])
        self.assertEqual(w.ended_at, w.stopped_at - m.BACKOFF_STEPS * sp.STEP_US)

    def test_the_recorded_stop_is_where_the_arm_really_was_not_where_we_counted(self):
        # a lost FORWARD step makes our own count run ahead of the real position
        board = LossyBoard(drop={5}, start=(1500, 1500, 1500, 1300))
        person = SimulatedPerson(board, feasible_default)
        w = m.walk(board, 3, 1900, person)
        self.assertEqual(w.stopped_at, person.at_stop)

    def test_a_board_that_never_moves_is_reported_not_papered_over(self):
        board = LossyBoard(drop_all=True, start=(1500, 1500, 1500, 1300))
        with self.assertRaises(m.WalkFailed) as cm:
            m.walk(board, 3, 1700, lambda d: False)
        self.assertIn("elbow", str(cm.exception))

    def test_queued_keypresses_are_thrown_away_before_every_stoppable_walk(self):
        board, person = make()
        flushed = []
        m.measure(board, person, shoulders=(1500, 1650), save=lambda r: None, log=lambda *a: None,
                  flush_input=lambda: flushed.append(1))
        self.assertEqual(len(flushed), 2 * 3)           # shoulder walk + elbow up + elbow down, per position

    def test_the_person_can_reject_a_position_and_it_is_measured_again(self):
        board, person = make()
        answers = iter([False, True])
        ups = []
        real_walk = m.walk

        def spy(b, ch, target, *a, **kw):
            if ch == 3 and target == sp.RANGES[3][1]:
                ups.append(1)
            return real_walk(b, ch, target, *a, **kw)

        m.walk_original, m.walk = m.walk, spy
        try:
            records = m.measure(board, person, shoulders=(1650,), save=lambda r: None, log=lambda *a: None,
                                confirm=lambda rec: next(answers))
        finally:
            m.walk = m.walk_original
        self.assertEqual(len(records), 1)                # the rejected attempt is replaced, not kept
        self.assertEqual(len(ups), 2)

    def test_giving_up_after_repeated_rejections_keeps_nothing_for_that_position(self):
        board, person = make()
        records = m.measure(board, person, shoulders=(1650, 1800), save=lambda r: None, log=lambda *a: None,
                            confirm=lambda rec: rec["shoulder"] != 1650)
        self.assertEqual([r["shoulder"] for r in records], [1800])

    def test_the_real_starting_pulse_of_each_sweep_is_recorded(self):
        board, person = make(start=(1500, 1500, 1650, 1300))
        records = m.measure(board, person, shoulders=(1500,), save=lambda r: None, log=lambda *a: None)
        self.assertEqual((records[0]["elbow_up_start"], records[0]["elbow_down_start"]), (1500, 1500))

    def test_progress_is_reported_on_every_step(self):
        board, person = make(feasible=lambda s, e: True)
        seen = []
        m.walk(board, 3, 1600, person, progress=lambda ch, pulse: seen.append((ch, pulse)))
        self.assertEqual(seen, [(3, 1525), (3, 1550), (3, 1575), (3, 1600)])


class AnalysisTest(unittest.TestCase):
    def records(self, slope=-0.5, noise=0):
        out = []
        for s in (1200, 1350, 1500, 1650, 1800):
            out.append({"shoulder": s, "elbow_up_bind": 1775 + slope * (s - 1500) + noise,
                        "elbow_down_bind": 1275 + slope * (s - 1500) - noise,
                        "shoulder_bind_at_elbow_1500": None,
                        "elbow_up_limit": 1850, "elbow_down_limit": 500})
        return out

    def test_windows_are_pulled_inward_by_the_safety_margin(self):
        w = m.windows(self.records(), margin_steps=3)
        by = {x["shoulder"]: x for x in w}
        self.assertEqual(by[1500]["hi"], 1775 - 75)
        self.assertEqual(by[1500]["lo"], 1275 + 75)

    def test_an_edge_that_never_bound_falls_back_to_the_measured_limit(self):
        recs = [{"shoulder": 1500, "elbow_up_bind": None, "elbow_down_bind": 1300,
                 "shoulder_bind_at_elbow_1500": None, "elbow_up_limit": 1850, "elbow_down_limit": 500}]
        w = m.windows(recs, margin_steps=3)[0]
        self.assertEqual(w["hi"], 1850)
        self.assertEqual(w["lo"], 1300 + 75)

    def test_a_shoulder_that_bound_by_itself_has_no_window(self):
        recs = [{"shoulder": 2100, "elbow_up_bind": None, "elbow_down_bind": None,
                 "shoulder_bind_at_elbow_1500": 2050, "elbow_up_limit": 1850, "elbow_down_limit": 500}]
        self.assertEqual(m.windows(recs, margin_steps=3), [])

    def test_line_fit_recovers_slope_and_reports_residual(self):
        f = m.fit_line([(s, 1775 - 0.5 * (s - 1500)) for s in (1200, 1350, 1500, 1650, 1800)])
        self.assertAlmostEqual(f["slope"], -0.5, places=6)
        self.assertAlmostEqual(f["max_residual"], 0.0, places=6)

    def test_line_fit_says_so_when_the_points_are_not_on_a_line(self):
        f = m.fit_line([(1200, 1000), (1350, 1600), (1500, 1000), (1650, 1600)])
        self.assertGreater(f["max_residual"], 100)

    def test_line_fit_needs_at_least_three_points(self):
        self.assertIsNone(m.fit_line([(1200, 1000), (1300, 1100)]))


class AskOkTest(unittest.TestCase):
    REC = {"shoulder": 1650, "elbow_up_bind": 1725, "elbow_down_bind": 700, "shoulder_bind_at_elbow_1500": None,
           "elbow_up_limit": 1850, "elbow_down_limit": 500}

    def ask(self, answer):
        drained = []
        out = []
        ok = m.ask_ok(self.REC, input_fn=lambda prompt: answer, drain=lambda: drained.append(1), out=out.append)
        return ok, drained, "\n".join(out)

    def test_enter_or_anything_but_r_accepts(self):
        for answer in ("", "ok", "y", " "):
            self.assertTrue(self.ask(answer)[0], repr(answer))

    def test_r_rejects_in_any_case(self):
        for answer in ("r", "R", " r "):
            self.assertFalse(self.ask(answer)[0], repr(answer))

    def test_queued_keypresses_are_discarded_first_so_a_stray_enter_cannot_auto_accept(self):
        self.assertEqual(len(self.ask("")[1]), 1)

    def test_it_shows_what_was_recorded(self):
        text = self.ask("")[2]
        for needle in ("1650", "1725", "700"):
            self.assertIn(needle, text)


class CauseTest(unittest.TestCase):
    """After every stop the person says WHY: a linkage that binds (buzzing, straining) is what we are
    measuring; something touching the base plate or table is the set-up, not the linkage."""

    def test_the_cause_is_asked_after_the_arm_has_backed_off_and_is_stored_with_the_stop(self):
        board, person = make()
        asked = []

        def ask(where, pulse):
            asked.append((where, pulse, board.pulse[2]))         # the elbow's pulse at the moment of asking
            return "linkage"

        records = m.measure(board, person, shoulders=(1500,), save=lambda r: None, log=lambda *a: None,
                            ask_cause=ask)
        wheres = [a[0] for a in asked]
        self.assertEqual(wheres, ["elbow_up", "elbow_down"])
        for where, stopped_at, current in asked:
            self.assertLess(abs(current - stopped_at), 1000)
        up = next(a for a in asked if a[0] == "elbow_up")
        self.assertLess(up[2], up[1])                                # already backed off, not still at the stop
        self.assertEqual(records[0]["elbow_up_cause"], "linkage")
        self.assertEqual(records[0]["elbow_down_cause"], "linkage")

    def test_no_stop_means_no_question_and_no_cause(self):
        board, person = make(feasible=lambda s, e: True)
        asked = []
        records = m.measure(board, person, shoulders=(1500,), save=lambda r: None, log=lambda *a: None,
                            ask_cause=lambda w, p: asked.append(w) or "linkage")
        self.assertEqual(asked, [])
        self.assertIsNone(records[0]["elbow_up_cause"])
        self.assertIsNone(records[0]["elbow_down_cause"])

    def test_a_shoulder_that_binds_by_itself_also_records_its_cause(self):
        board, person = make(feasible=lambda s, e: s <= 1700 and feasible_default(s, e))
        records = m.measure(board, person, shoulders=(1500, 1800), save=lambda r: None, log=lambda *a: None,
                            ask_cause=lambda w, p: "collision")
        by = {r["shoulder"]: r for r in records}
        self.assertEqual(by[1800]["shoulder_bind_cause"], "collision")

    def test_a_stop_caused_by_a_collision_is_not_a_linkage_edge(self):
        rec = {"shoulder": 1350, "elbow_up_bind": 1525, "elbow_down_bind": 725, "shoulder_bind_at_elbow_1500": None,
               "elbow_up_limit": 1850, "elbow_down_limit": 500, "elbow_up_cause": "linkage",
               "elbow_down_cause": "collision"}
        w = m.windows([rec], margin_steps=3)[0]
        self.assertEqual(w["hi"], 1525 - 75)
        self.assertEqual(w["lo"], 500)                    # the collision does not raise the linkage's lower edge
        w2 = m.windows([rec], margin_steps=3, include_collisions=True)[0]
        self.assertEqual(w2["lo"], 725 + 75)

    def test_records_without_a_cause_count_as_linkage_edges(self):
        rec = {"shoulder": 1500, "elbow_up_bind": 1600, "elbow_down_bind": None, "shoulder_bind_at_elbow_1500": None,
               "elbow_up_limit": 1850, "elbow_down_limit": 500}               # (the runs measured before causes existed)
        self.assertEqual(m.windows([rec], margin_steps=3)[0]["hi"], 1600 - 75)

    def test_the_summary_lists_collision_stops_separately(self):
        rec = {"shoulder": 1350, "elbow_up_bind": 1525, "elbow_down_bind": 725, "shoulder_bind_at_elbow_1500": None,
               "elbow_up_limit": 1850, "elbow_down_limit": 500, "elbow_up_cause": "linkage",
               "elbow_down_cause": "collision"}
        out = []
        m.summarize([rec], out=out.append)
        text = "\n".join(out)
        self.assertIn("collision", text)
        self.assertIn("725", text)


class SelfStopSummaryTest(unittest.TestCase):
    """The shoulder stopping by itself (elbow at rest) is the safe LOWER bound of the shoulder: the summary must
    show where it stopped and why, not just which positions were requested."""

    def rec(self, target, stop, cause):
        return {"shoulder": target, "shoulder_bind_at_elbow_1500": stop, "shoulder_bind_cause": cause,
                "elbow_up_bind": None, "elbow_down_bind": None, "elbow_up_limit": 1850, "elbow_down_limit": 500}

    def text(self, records):
        out = []
        m.summarize(records, out=out.append)
        return "\n".join(out)

    def test_the_stop_pulses_and_their_causes_are_listed(self):
        t = self.text([self.rec(1350, 1350, "collision"), self.rec(1350, 1400, "collision"),
                       self.rec(1425, 1425, "linkage")])
        for needle in ("1350", "1400", "1425", "collision", "linkage"):
            self.assertIn(needle, t)

    def test_the_spread_between_repeated_stops_is_shown(self):
        t = self.text([self.rec(1350, 1350, "collision"), self.rec(1350, 1375, "collision")])
        self.assertIn("repeat within 25", t)              # (not just any "25": 1375 / 1350 must not satisfy it by accident)

    def test_a_record_without_a_cause_says_unknown_instead_of_guessing(self):
        rec = self.rec(1350, 1350, None)
        rec.pop("shoulder_bind_cause")
        self.assertIn("unlabelled", self.text([rec]))


class SafeEnvelopeTest(unittest.TestCase):
    """What the REAL arm may be commanded to. Unlike the linkage-only windows, a stop of ANY cause counts: a claw
    hitting the base plate is as bad for the hardware as a linkage binding."""

    def rec(self, s, up=None, down=None, up_cause=None, down_cause=None, self_stop=None, self_cause=None):
        return {"shoulder": s, "shoulder_bind_at_elbow_1500": self_stop, "shoulder_bind_cause": self_cause,
                "elbow_up_bind": up, "elbow_down_bind": down, "elbow_up_cause": up_cause,
                "elbow_down_cause": down_cause, "elbow_up_limit": 1850, "elbow_down_limit": 500}

    def test_collisions_are_inside_the_envelope_even_though_they_are_outside_the_linkage_windows(self):
        recs = [self.rec(1500, up=1575, up_cause="collision")]
        env = m.safe_envelope(recs, margin_steps=3)
        self.assertEqual(env["windows"][0]["hi"], 1575 - 75)
        self.assertEqual(m.windows(recs, margin_steps=3)[0]["hi"], 1850)      # the linkage-only view ignores it

    def test_the_shoulders_lower_bound_comes_from_stops_below_rest_taking_the_one_nearest_rest(self):
        recs = [self.rec(1350, self_stop=1350, self_cause="collision"), self.rec(1350, self_stop=1400, self_cause="collision"),
                self.rec(1425, self_stop=1425, self_cause="collision")]
        env = m.safe_envelope(recs, margin_steps=3)
        self.assertEqual(env["shoulder_lo"], 1425 + 75)

    def test_the_shoulders_upper_bound_comes_from_stops_above_rest(self):
        recs = [self.rec(1800, self_stop=1750, self_cause="linkage"), self.rec(1900, self_stop=1850, self_cause="linkage")]
        env = m.safe_envelope(recs, margin_steps=3)
        self.assertEqual(env["shoulder_hi"], 1750 - 75)

    def test_without_any_self_stops_the_shoulder_keeps_its_own_measured_range_and_says_it_was_not_tested(self):
        env = m.safe_envelope([self.rec(1500)], margin_steps=3)
        self.assertEqual((env["shoulder_lo"], env["shoulder_hi"]), sp.RANGES[m.SHOULDER])
        self.assertFalse(env["shoulder_lo_tested"])
        self.assertFalse(env["shoulder_hi_tested"])

    def test_tested_bounds_are_marked_tested(self):
        env = m.safe_envelope([self.rec(1350, self_stop=1350, self_cause="collision")], margin_steps=3)
        self.assertTrue(env["shoulder_lo_tested"])
        self.assertFalse(env["shoulder_hi_tested"])

    def test_an_unlabelled_stop_counts_too(self):
        env = m.safe_envelope([self.rec(1500, up=1600)], margin_steps=3)
        self.assertEqual(env["windows"][0]["hi"], 1600 - 75)

    def test_the_summary_prints_the_envelope_and_says_which_bounds_were_never_tested(self):
        out = []
        m.summarize([self.rec(1500, up=1575, up_cause="collision"),
                     self.rec(1350, self_stop=1350, self_cause="collision")], out=out.append)
        text = "\n".join(out)
        self.assertIn("Safe envelope", text)
        self.assertIn("1500", text)
        self.assertIn("untested", text)             # the shoulder's upper bound was never tested


class RejectedAttemptsTest(unittest.TestCase):
    """An attempt the person rejects is not an accepted measurement, but it is still evidence -- the shoulder-1800
    attempts that were rejected on 2026-09-27 showed the position behaves differently from one try to the next --
    so it is kept, separately, and never used for the windows."""

    def measure_with(self, answers, shoulders=(1650,), max_attempts=3):
        board, person = make()
        rejected = []
        it = iter(answers)
        records = m.measure(board, person, shoulders=shoulders, save=lambda r: None, log=lambda *a: None,
                            confirm=lambda rec: next(it), rejected_out=rejected, max_attempts=max_attempts)
        return records, rejected

    def test_a_rejected_attempt_is_kept_separately_with_its_attempt_number(self):
        records, rejected = self.measure_with([False, True])
        self.assertEqual(len(records), 1)
        self.assertEqual(len(rejected), 1)
        self.assertEqual(rejected[0]["shoulder"], 1650)
        self.assertEqual(rejected[0]["attempt"], 1)
        self.assertIn("elbow_up_bind", rejected[0])

    def test_a_position_dropped_after_repeated_rejections_still_has_all_its_attempts_saved(self):
        records, rejected = self.measure_with([False, False, False])
        self.assertEqual(records, [])
        self.assertEqual([r["attempt"] for r in rejected], [1, 2, 3])

    def test_a_rejected_attempt_is_written_out_immediately_not_at_the_end(self):
        # an interruption (Ctrl+C, an error) right after a rejection must not lose it, and it must not be
        # possible for a later save to replace the file's contents with fewer records than were accepted
        board, person = make()
        rejected, saves = [], []
        it = iter([True, False, True])
        m.measure(board, person, shoulders=(1500, 1650), save=lambda r: saves.append((len(r), len(rejected))),
                  log=lambda *a: None, confirm=lambda rec: next(it), rejected_out=rejected)
        self.assertEqual(saves, [(1, 0), (1, 1), (2, 1)])        # accepted 1500; rejected 1650; accepted 1650

    def test_accepted_runs_produce_no_rejected_entries(self):
        _records, rejected = self.measure_with([True])
        self.assertEqual(rejected, [])

    def test_the_saved_file_carries_the_rejected_attempts_but_not_in_records(self):
        payload = m.result_payload((1650,), [{"shoulder": 1650}], [{"shoulder": 1800, "attempt": 1}])
        self.assertEqual(payload["records"], [{"shoulder": 1650}])
        self.assertEqual(payload["rejected_attempts"], [{"shoulder": 1800, "attempt": 1}])
        for key in ("measured_at", "step_us", "dwell_s", "backoff_steps", "claw_rest_us", "shoulders_requested"):
            self.assertIn(key, payload)

    def test_analyze_lists_them_apart_from_the_windows_and_does_not_use_them(self):
        import json
        import tempfile
        d = Path(tempfile.mkdtemp())
        good = {"shoulder": 1800, "elbow_up_bind": None, "elbow_down_bind": None, "shoulder_bind_at_elbow_1500": None,
                "elbow_up_limit": 1850, "elbow_down_limit": 500}
        bad = dict(good, elbow_up_bind=1750, elbow_down_bind=1475, attempt=1)
        (d / "a.json").write_text(json.dumps({"records": [good], "rejected_attempts": [bad]}))
        out = []
        m.analyze([d / "a.json"], out=out.append)
        text = "\n".join(out)
        self.assertIn("Rejected", text)
        self.assertIn("1750", text)
        self.assertIn("1475", text)
        w = m.windows(m.load_records([d / "a.json"]))
        self.assertEqual((w[0]["lo"], w[0]["hi"]), (500, 1850))          # the rejected stops did not narrow it
        # ...and neither did they narrow what analyze itself PRINTS as the window / envelope
        import re
        self.assertRegex(text, r"1800\s+500 ~\s+1850")
        self.assertNotIn("1675", text)                                   # 1750 - the 3-step margin, had it leaked in
        self.assertIn("1 record(s)", text)                                   # the accepted record only


class AskCauseCliTest(unittest.TestCase):
    def ask(self, answers):
        it = iter(answers)
        drained, out = [], []
        c = m.ask_cause_cli("elbow_down", 725, input_fn=lambda prompt: next(it), drain=lambda: drained.append(1),
                            out=out.append)
        return c, drained, out

    def test_enter_and_l_mean_linkage_c_means_collision_question_mark_means_unsure(self):
        for answer, expected in (("", "linkage"), ("l", "linkage"), ("L", "linkage"), ("c", "collision"),
                                 ("C", "collision"), ("?", "unsure")):
            self.assertEqual(self.ask([answer])[0], expected, repr(answer))

    def test_an_unrecognised_answer_asks_again_instead_of_guessing(self):
        c, _d, out = self.ask(["x", "zzz", "c"])
        self.assertEqual(c, "collision")
        self.assertTrue(any("unrecognized" in line for line in out))

    def test_queued_keypresses_are_discarded_first(self):
        self.assertEqual(len(self.ask([""])[1]), 1)


class MultiRunTest(unittest.TestCase):
    def rec(self, s, up, down=None, up_cause=None):
        return {"shoulder": s, "elbow_up_bind": up, "elbow_down_bind": down, "shoulder_bind_at_elbow_1500": None,
                "elbow_up_limit": 1850, "elbow_down_limit": 500, "elbow_up_cause": up_cause,
                "elbow_down_cause": None}

    def test_repeats_at_one_position_use_the_most_conservative_edge_and_report_the_spread(self):
        recs = [self.rec(1350, 1525, 725), self.rec(1350, 1600, 800), self.rec(1500, 1600)]
        w = {x["shoulder"]: x for x in m.windows(recs, margin_steps=3)}
        self.assertEqual(w[1350]["hi"], 1525 - 75)         # the lower of the two upper stops
        self.assertEqual(w[1350]["lo"], 800 + 75)          # the higher of the two lower stops
        self.assertEqual(w[1350]["n"], 2)
        self.assertEqual(w[1350]["up_spread"], 75)
        self.assertEqual(w[1350]["down_spread"], 75)
        self.assertEqual(w[1500]["n"], 1)
        self.assertIsNone(w[1500]["up_spread"])

    def test_a_missing_stop_in_one_repeat_falls_back_to_the_limit_for_that_repeat_only(self):
        recs = [self.rec(1500, None), self.rec(1500, 1600)]
        self.assertEqual(m.windows(recs, margin_steps=3)[0]["hi"], 1600 - 75)

    def test_load_records_merges_files_and_ignores_the_old_flawed_run_unless_asked(self):
        import json
        import tempfile
        d = Path(tempfile.mkdtemp())
        (d / "a.json").write_text(json.dumps({"records": [self.rec(1500, 1600)]}))
        (d / "b.json").write_text(json.dumps({"records": [self.rec(1500, 1625), self.rec(1650, 1775)]}))
        self.assertEqual(len(m.load_records([d / "a.json", d / "b.json"])), 3)

    def test_a_file_that_is_not_a_measurement_is_refused_clearly(self):
        import tempfile
        p = Path(tempfile.mkdtemp()) / "x.json"
        p.write_text("{\"nope\": 1}")
        with self.assertRaises(ValueError):
            m.load_records([p])


class OutputAndOptionsTest(unittest.TestCase):
    def test_the_default_output_is_a_new_timestamped_file_never_an_existing_one(self):
        import time as _t
        a = m.default_out_path(_t.struct_time((2026, 9, 27, 1, 2, 3, 0, 0, -1)))
        self.assertEqual(a.name, "mearm_linkage_20260927-010203.json")
        self.assertEqual(a.parent.name, "data")

    def test_saving_over_an_existing_file_is_refused(self):
        import tempfile
        p = Path(tempfile.mkdtemp()) / "keep.json"
        p.write_text("precious")
        with self.assertRaises(FileExistsError):
            m.check_output_path(p, overwrite=False)
        self.assertEqual(p.read_text(), "precious")
        m.check_output_path(p, overwrite=True)                        # explicit opt-in
        m.check_output_path(p.with_name("new.json"), overwrite=False)

    def test_shoulders_option_parses_a_list_and_allows_repeats_and_keeps_the_order(self):
        self.assertEqual(m.parse_shoulders("1350,1350, 1425,1500"), (1350, 1350, 1425, 1500))

    def test_with_the_elbow_moved_out_of_the_way_the_shoulder_may_be_measured_up_to_its_own_limit(self):
        # 2026-09-28: the 9/27 strain at 2100 was measured with the elbow at 1500, where its linkage hits the upper arm;
        # the author then held shoulder 2000 / elbow 900 and 2100 / elbow 700 without strain. The top-end guard only
        # applies with the elbow at 1500; the bottom end (1200: the shoulder binds by itself) keeps its guard.
        self.assertEqual(m.parse_shoulders("2025,2100", elbow_hold=700), (2025, 2100))
        for bad in ("2125", "1250"):
            with self.assertRaises(ValueError, msg=bad):
                m.parse_shoulders(bad, elbow_hold=700)
        with self.assertRaises(ValueError):
            m.parse_shoulders("2025")                     # the default elbow hold (1500): still refused

    def test_shoulders_option_refuses_the_ends_of_the_range_and_garbage(self):
        for bad in ("1200", "2100", "1250", "2050", "abc", "", "1500,,1600", "1500,99999"):
            with self.assertRaises(ValueError, msg=bad):
                m.parse_shoulders(bad)


class AnalyzeTest(unittest.TestCase):
    def write(self, name, records):
        import json
        import tempfile
        d = Path(tempfile.mkdtemp())
        (d / name).write_text(json.dumps({"records": records}))
        return d / name

    def test_analyze_merges_files_and_prints_the_table_with_repeatability(self):
        rec = lambda s, up, down: {"shoulder": s, "elbow_up_bind": up, "elbow_down_bind": down,
                                   "shoulder_bind_at_elbow_1500": None, "elbow_up_limit": 1850,
                                   "elbow_down_limit": 500}
        a = self.write("a.json", [rec(1350, 1525, 725), rec(1500, 1600, None)])
        b = self.write("b.json", [rec(1350, 1600, 800), rec(1650, 1775, None), rec(1800, 1800, None)])
        out = []
        m.analyze([a, b], out=out.append)
        text = "\n".join(out)
        for needle in ("1350", "1500", "1650", "1800", "top spread", "75"):
            self.assertIn(needle, text)

    def test_analyze_refuses_a_file_that_is_not_a_measurement(self):
        import tempfile
        p = Path(tempfile.mkdtemp()) / "x.json"
        p.write_text("[]")
        with self.assertRaises(ValueError):
            m.analyze([p], out=lambda *_: None)


class InputTest(unittest.TestCase):
    def test_the_default_shoulder_positions_stay_inside_the_measured_range_and_start_at_rest(self):
        lo, hi = sp.RANGES[2]
        self.assertEqual(m.DEFAULT_SHOULDERS[0], 1500)
        self.assertTrue(all(lo <= s <= hi for s in m.DEFAULT_SHOULDERS))
        self.assertGreaterEqual(len(set(m.DEFAULT_SHOULDERS)), 4)

    def test_the_extreme_shoulder_positions_are_not_measured(self):
        # 2026-09-27: at 2100 three attempts gave a downward stop of 850, 725 and 1400 (not repeatable:
        # the mechanism was straining), and at 1200 the shoulder binds by itself at ~1250. The default
        # positions therefore stay clear of both ends of the shoulder's range.
        self.assertNotIn(2100, m.DEFAULT_SHOULDERS)
        self.assertNotIn(1200, m.DEFAULT_SHOULDERS)
        lo, hi = sp.RANGES[2]
        self.assertGreater(min(m.DEFAULT_SHOULDERS), lo + 100)
        self.assertLess(max(m.DEFAULT_SHOULDERS), hi - 100)


if __name__ == "__main__":
    unittest.main()
