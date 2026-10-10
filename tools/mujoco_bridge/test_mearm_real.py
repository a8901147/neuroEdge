"""Sensors -> REAL arm pulses: Path B's decode, then the measured link-angle map (model command -> pulse), then the measured
safe envelope. Properties checked with the real 2026-09-24 live log and the embedded 9/13 calibration; no hardware."""
import math
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
import gen_mearm_envelope as env  # noqa: E402
import mearm_pathb as pb  # noqa: E402
import mearm_pulse_map as pm  # noqa: E402
import mearm_real as real  # noqa: E402
import test_mearm_direction as fx  # noqa: E402

CAL = pb.make_calibration(fx.SAVED_9_13)
ARM = real.load_arm()                 # the committed angle measurements + envelope


def interior_deg(amap, s_us, e_us):
    """Angle between upper arm and forearm at the elbow (180 = straight)."""
    return 180.0 - (amap.upper_arm_elevation_deg(s_us) - amap.forearm_elevation_deg(e_us))


class LoadTest(unittest.TestCase):
    def test_the_angle_map_comes_from_the_sign_corrected_measurement(self):
        self.assertTrue(real.ANGLE_FILE.name.endswith("_signed.json"))
        self.assertGreater(ARM.angles.shoulder["slope"], 0)          # measured: higher pulse = upper arm higher
        self.assertLess(ARM.angles.elbow["slope"], 0)                # measured: higher pulse = forearm lower
        self.assertTrue(pm.slope_is_plausible(ARM.angles.shoulder["slope"]))
        self.assertTrue(pm.slope_is_plausible(ARM.angles.elbow["slope"]))

    def test_the_envelope_is_the_generated_one(self):
        self.assertEqual(ARM.table, env.build_table(env.m.load_records([env.ROOT / "data" / n for n in env.SOURCES])))


class BaseTest(unittest.TestCase):
    """The base, measured on the real arm 2026-09-28: usable 500..2500 us, and 1700 us turns it LEFT (seen from behind
    the arm). Path B's base command is + for left, so a left swing must raise the pulse."""

    def test_hanging_and_invalid_readings_keep_the_base_at_rest(self):
        for raw in (fx.HANG, (0, 0, 0), None, (math.nan, 0, 1)):
            self.assertEqual(real.base_pulse(CAL, raw), real.REST_BASE_US)

    def test_left_turns_left_and_right_turns_right(self):
        self.assertGreater(real.base_pulse(CAL, fx.LEFT), real.REST_BASE_US + 200)
        self.assertLess(real.base_pulse(CAL, fx.RIGHT), real.REST_BASE_US - 200)

    def test_always_whole_microseconds_inside_the_measured_range(self):
        for raw in SafetyTest.INPUTS:
            us = real.base_pulse(CAL, raw)
            self.assertIsInstance(us, int)
            self.assertTrue(real.BASE_RANGE_US[0] <= us <= real.BASE_RANGE_US[1], (raw, us))


class BaseReachTest(unittest.TestCase):
    """2026-10-03 (the author's choice): the base keeps its whole 500..2500 us, but reaching its ends takes the arm's own
    COMFORTABLE left/right reach (measured with measure_base_reach.py) instead of the calibration's ~+-34 deg, so a
    turn of the arm moves the base less. Without a measured reach nothing changes."""

    @staticmethod
    def turned_left(deg):
        """The arm held forward, turned `deg` to the LEFT (negative = right). (A +angle rotation about the HANG axis is
        a turn to the RIGHT in this calibration's frame -- checked with cal.decode.)"""
        v = fx._rotate(fx.FORWARD, fx.HANG, -math.radians(deg))
        assert abs(math.degrees(CAL.decode(v)[1]) - deg) < 0.5
        return v

    def test_without_a_measured_reach_the_base_is_exactly_as_before(self):
        # "before" = Path B's own base command spread over 500..2500 (the 2026-09-28 mapping), computed independently
        for raw in list(fx.REAL_LOG_2026_09_24) + [fx.HANG, fx.FORWARD, fx.LEFT, fx.RIGHT]:
            base_ctrl = pb.ctrl_from_sensors(CAL, raw, math.nan)[0]
            before = int(round(min(max(real.REST_BASE_US + base_ctrl / pb.BASE_LIMIT * 1000.0, 500), 2500)))
            self.assertAlmostEqual(real.base_pulse(CAL, raw), before, delta=1, msg=raw)
            self.assertEqual(real.base_pulse(CAL, raw, real.default_base_reach(CAL)), real.base_pulse(CAL, raw))

    def test_nearly_hanging_the_base_stays_at_rest_whatever_the_reach(self):
        # the arm only 5 deg off HANG, towards the left: azimuth is meaningless that close to hanging (Path B's fade)
        v = fx._slerp(fx.HANG, self.turned_left(80.0), 5.0 / 73.0)
        for reach in (None, (math.radians(30.0), math.radians(-30.0))):
            self.assertEqual(real.base_pulse(CAL, v, reach), real.REST_BASE_US)

    def test_a_saved_reach_on_one_side_only_is_refused(self):
        saved = dict(fx.SAVED_9_13, base_reach_left_raw=list(self.turned_left(40.0)),
                     base_reach_right_raw=list(self.turned_left(20.0)))
        with self.assertRaises(ValueError):
            real.saved_base_reach(CAL, saved)

    def test_the_measured_reach_is_where_the_base_hits_its_ends_and_halfway_is_halfway(self):
        reach = (math.radians(60.0), math.radians(-50.0))
        at = lambda deg: real.base_pulse(CAL, self.turned_left(deg), reach)
        self.assertEqual(at(0.0), real.REST_BASE_US)
        self.assertAlmostEqual(at(30.0), 2000, delta=3)                  # half the left reach -> half the left range
        self.assertAlmostEqual(at(-25.0), 1000, delta=3)                 # half the right reach -> half the right range
        self.assertEqual(at(65.0), 2500)                                 # beyond the reach: the end, never past it
        self.assertEqual(at(-55.0), 500)

    def test_a_wider_reach_makes_the_base_less_sensitive(self):
        turn = lambda reach: real.base_pulse(CAL, self.turned_left(10.0), reach)
        narrow, wide = real.default_base_reach(CAL), (math.radians(60.0), math.radians(-60.0))
        self.assertLess(turn(wide) - real.REST_BASE_US, (turn(narrow) - real.REST_BASE_US) * 0.7)

    def test_the_hanging_and_behind_the_body_fades_still_apply(self):
        reach = (math.radians(60.0), math.radians(-60.0))
        self.assertEqual(real.base_pulse(CAL, fx.HANG, reach), real.REST_BASE_US)

    def test_the_reach_is_read_from_the_saved_calibration_raw_vectors(self):
        saved = dict(fx.SAVED_9_13)
        self.assertIsNone(real.saved_base_reach(CAL, saved))
        saved["base_reach_left_raw"] = list(self.turned_left(55.0))
        saved["base_reach_right_raw"] = list(self.turned_left(-45.0))
        left, right = real.saved_base_reach(CAL, saved)
        self.assertAlmostEqual(math.degrees(left), 55.0, delta=0.5)
        self.assertAlmostEqual(math.degrees(right), -45.0, delta=0.5)


class SafetyTest(unittest.TestCase):
    INPUTS = list(fx.REAL_LOG_2026_09_24) + [fx.HANG, fx.FORWARD, fx.LEFT, fx.RIGHT, (0, 0, 0), None,
                                             (math.nan, 0, 1), (math.inf, 0, 1), tuple(-c for c in fx.HANG)]
    BENDS = (fx.STRAIGHT, fx.STRAIGHT + 0.7, fx.FLEXED, 0.0, 3.1, math.nan, math.inf)

    def test_every_output_is_inside_the_measured_envelope_whole_microseconds(self):
        for mode in real.MODES:
            for raw in self.INPUTS:
                for bend in self.BENDS:
                    s, e = real.pulses(ARM, CAL, raw, bend, mode=mode)
                    self.assertIsInstance(s, int)
                    self.assertIsInstance(e, int)
                    self.assertEqual(env.clamp(ARM.table, s, e), (s, e), (mode, raw, bend, s, e))

    def test_an_invalid_reading_gives_the_rest_pose(self):
        for mode in real.MODES:
            for raw in ((0, 0, 0), None, (math.nan, 0, 1)):
                s, _e = real.pulses(ARM, CAL, raw, fx.STRAIGHT, mode=mode)
                self.assertEqual(s, real.REST_SHOULDER_US)


class DirectionTest(unittest.TestCase):
    """The joint-for-joint modes (2026-09-27/28): the person's shoulder drives the shoulder servo, the elbow the elbow
    servo. Kept as options; the default since 2026-10-03 is height_reach (HeightReachTest)."""

    def test_raising_the_arm_raises_the_real_upper_arm(self):
        for mode in ("geometric", "stretch"):
            elev = []
            for t in (0.0, 0.25, 0.5, 0.75, 1.0):
                s, _e = real.pulses(ARM, CAL, fx._slerp(fx.HANG, fx.FORWARD, t), fx.STRAIGHT, mode=mode)
                elev.append(ARM.angles.upper_arm_elevation_deg(s))
            self.assertEqual(elev, sorted(elev), mode)
            self.assertGreater(elev[-1] - elev[0], 15.0, mode)          # a real, visible lift (safe range is ~20 deg)

    def test_bending_the_elbow_drives_the_real_elbow_servo_the_way_verified_on_the_arm(self):
        # 2026-09-28 real MEArm: with the first mapping (bend -> higher elbow pulse) the real elbow moved OPPOSITE to the
        # user's own elbow (MuJoCo matched the hand). The hand-checked direction is: bending -> LOWER elbow pulse.
        for mode in ("geometric", "stretch"):
            for raw in (fx.HANG, fx._slerp(fx.HANG, fx.FORWARD, 0.6), fx.FORWARD):
                pulses = [real.pulses(ARM, CAL, raw, bend, mode=mode)[1]
                          for bend in (fx.STRAIGHT, fx.STRAIGHT + 0.5, fx.STRAIGHT + 1.0, fx.FLEXED)]
                self.assertEqual(pulses, sorted(pulses, reverse=True), (mode, pulses))   # never turns back
                self.assertGreater(pulses[0] - pulses[-1], 300, (mode, pulses))          # a real, visible bend

    def test_the_real_elbow_follows_from_the_first_degrees_of_a_bend_no_dead_zone(self):
        # 2026-09-28 real arm (elbow-only test): the first ~45 deg of the author's bend did not move the real elbow at all --
        # the mirrored straight end lay outside the envelope and was clamped flat. Now every few degrees must move it.
        for mode in ("geometric", "stretch"):
            for raw in (fx.HANG, fx._slerp(fx.HANG, fx.FORWARD, 0.5), fx.FORWARD):
                steps = [fx.STRAIGHT + i * pb.ELBOW_SWING_RAD / 20 for i in range(21)]
                pulses = [real.pulses(ARM, CAL, raw, b, mode=mode)[1] for b in steps]
                for a, b in zip(pulses, pulses[1:]):
                    self.assertLess(b, a, (mode, raw, pulses))                     # every ~6 deg moves it

    def test_straight_is_the_top_of_the_elbow_window_and_fully_bent_the_bottom(self):
        for raw in (fx.HANG, fx._slerp(fx.HANG, fx.FORWARD, 0.5), fx.FORWARD):
            s, e_straight = real.pulses(ARM, CAL, raw, fx.STRAIGHT, mode="stretch")
            _s, e_bent = real.pulses(ARM, CAL, raw, fx.STRAIGHT + pb.ELBOW_SWING_RAD, mode="stretch")
            self.assertEqual(e_straight, env.clamp(ARM.table, s, 10 ** 6)[1])
            self.assertEqual(e_bent, env.clamp(ARM.table, s, -10 ** 6)[1])

    def test_elbow_only_motion_does_not_move_the_shoulder(self):
        for mode in ("geometric", "stretch"):
            for raw in (fx.HANG, fx.FORWARD, fx.LEFT):
                shoulders = {real.pulses(ARM, CAL, raw, b, mode=mode)[0] for b in (fx.STRAIGHT, fx.FLEXED)}
                self.assertEqual(len(shoulders), 1, (mode, raw))


class HeightReachTest(unittest.TestCase):
    """2026-10-03 (the author's design, checked on the real arm): the MEArm's forearm servo sets the claw's HEIGHT and its
    upper-arm servo its REACH, so the person's arm drives them crosswise -- raising the arm lowers the ELBOW servo's
    pulse (claw up), bending the elbow raises the SHOULDER servo's pulse. Both still inside the measured envelope."""

    def raise_steps(self):
        return [fx._slerp(fx.HANG, fx.FORWARD, t) for t in (0.0, 0.25, 0.5, 0.75, 1.0)]

    def test_raising_the_arm_lowers_the_elbow_servo_and_leaves_the_shoulder_servo(self):
        for bend in (fx.STRAIGHT, fx.STRAIGHT + 1.0):
            pairs = [real.pulses(ARM, CAL, raw, bend) for raw in self.raise_steps()]
            elbows = [e for _s, e in pairs]
            self.assertEqual(elbows, sorted(elbows, reverse=True), (bend, pairs))
            self.assertGreater(elbows[0] - elbows[-1], 300, (bend, pairs))           # a real, visible move
            self.assertEqual(len({s for s, _e in pairs}), 1, (bend, pairs))          # shoulder servo untouched

    def test_bending_the_elbow_raises_the_shoulder_servo(self):
        for raw in (fx.HANG, fx._slerp(fx.HANG, fx.FORWARD, 0.5), fx.FORWARD):
            steps = [fx.STRAIGHT + i * pb.ELBOW_SWING_RAD / 20 for i in range(21)]
            shoulders = [real.pulses(ARM, CAL, raw, b)[0] for b in steps]
            for a, b in zip(shoulders, shoulders[1:]):
                self.assertGreater(b, a, (raw, shoulders))                       # every ~6 deg moves it, no dead zone

    def test_straight_and_hanging_is_the_start_pose_and_the_ends_are_the_envelope_ends(self):
        lo, hi = env.shoulder_range(ARM.table)
        self.assertEqual(real.pulses(ARM, CAL, fx.HANG, fx.STRAIGHT), (lo, env.clamp(ARM.table, lo, 10 ** 6)[1]))
        self.assertEqual(real.pulses(ARM, CAL, fx.HANG, fx.STRAIGHT), (1500, 1500))   # = the R start pose
        s, e = real.pulses(ARM, CAL, fx.FORWARD, fx.STRAIGHT + pb.ELBOW_SWING_RAD)
        self.assertEqual(s, hi)                                                   # fully bent: the reach end
        self.assertEqual(e, env.clamp(ARM.table, s, -10 ** 6)[1])                # raised: the elbow window's low end

    def test_hanging_with_the_elbow_fully_bent_keeps_the_elbow_servo_at_the_top_of_ITS_window(self):
        # the window depends on the shoulder pulse: at the top of the reach (shoulder 2100) the elbow may only go to 1000
        s, e = real.pulses(ARM, CAL, fx.HANG, fx.STRAIGHT + pb.ELBOW_SWING_RAD)
        self.assertEqual(s, env.shoulder_range(ARM.table)[1])
        self.assertEqual(e, env.clamp(ARM.table, s, 10 ** 6)[1])
        self.assertLess(e, 1500)
        # half raised: strictly inside THAT window (computing the window at the rest shoulder would pin it to the top)
        s2, e2 = real.pulses(ARM, CAL, fx._slerp(fx.HANG, fx.FORWARD, 0.5), fx.STRAIGHT + pb.ELBOW_SWING_RAD)
        self.assertEqual(s2, s)
        self.assertLess(env.clamp(ARM.table, s2, -10 ** 6)[1], e2)
        self.assertLess(e2, env.clamp(ARM.table, s2, 10 ** 6)[1])

    def test_a_one_degree_change_never_makes_the_elbow_servo_jump(self):
        # 2026-10-03 real arm: "the elbow moves steeply, a little motion and it shakes up and down". The elbow window's
        # top jumps where the measured shoulder positions change (1500 -> 1850 at shoulder 1575, -> 1600 past 1650 ...),
        # so a bend of 16 -> 17 deg (shoulder 1574 -> 1579) moved the elbow servo 1200 -> 1445. Scanning every 0.5 deg
        # of bend and of raise: no step may move it more than a smooth slope would.
        for t in (0.0, 0.3, 0.6, 1.0):
            raw = fx._slerp(fx.HANG, fx.FORWARD, t)
            prev = None
            for i in range(0, 260):
                e = real.pulses(ARM, CAL, raw, fx.STRAIGHT + math.radians(i * 0.5))[1]
                if prev is not None:
                    self.assertLessEqual(abs(e - prev), 25, (t, i * 0.5, prev, e))
                prev = e
        for bend_deg in (0, 16.5, 40, 90):
            prev = None
            for i in range(0, 101):
                e = real.pulses(ARM, CAL, fx._slerp(fx.HANG, fx.FORWARD, i / 100), fx.STRAIGHT + math.radians(bend_deg))[1]
                if prev is not None:
                    self.assertLessEqual(abs(e - prev), 25, (bend_deg, i, prev, e))
                prev = e

    def test_the_smoothed_elbow_window_never_leaves_the_measured_one(self):
        lo_s, hi_s = env.shoulder_range(ARM.table)
        for s in range(lo_s, hi_s + 1):
            top, bottom = real.smooth_elbow_window(ARM.table, s)
            self.assertLessEqual(top, env.clamp(ARM.table, s, 10 ** 6)[1], s)
            self.assertGreaterEqual(bottom, env.clamp(ARM.table, s, -10 ** 6)[1], s)
            self.assertLessEqual(bottom, top, s)

    def test_the_smoothed_window_also_stays_inside_a_varying_bottom_edge(self):
        # every measured bottom is 500 today, so a made-up table with a varying bottom checks that side too
        table = {"shoulders": [1500, 1600, 1700], "lo": [500, 800, 500], "hi": [1500, 1500, 1500]}
        for s in range(1500, 1701):
            top, bottom = real.smooth_elbow_window(table, s)
            self.assertGreaterEqual(bottom, env.clamp(table, s, -10 ** 6)[1], s)
            self.assertLessEqual(top, env.clamp(table, s, 10 ** 6)[1], s)

    def test_an_invalid_upper_arm_reading_gives_the_rest_pose_whatever_the_elbow(self):
        for raw in ((0, 0, 0), None, (math.nan, 0, 1)):
            self.assertEqual(real.pulses(ARM, CAL, raw, fx.FLEXED), (1500, 1500))


class ModeTest(unittest.TestCase):
    def test_geometric_mode_converts_the_model_shoulder_exactly_when_inside_the_range(self):
        # geometric = the real upper arm at the MuJoCo MeArm's own angle, unaltered while inside the measured shoulder
        # range. (The shoulder only: since 2026-09-28 the elbow is spread over the window at that shoulder in both modes.)
        lo, hi = env.shoulder_range(ARM.table)
        inside = 0
        for i in range(41):
            raw = fx._slerp(fx.HANG, fx.FORWARD, i / 40)
            _b, sh, _el = pb.ctrl_from_sensors(CAL, raw, fx.STRAIGHT)
            ps, _pe = ARM.angles.pulses_from_model_ctrl(sh, pb.ELBOW_EXTENDED)
            if lo <= ps <= hi:
                inside += 1
                self.assertEqual(real.pulses(ARM, CAL, raw, fx.STRAIGHT, mode="geometric")[0], int(math.floor(ps + 0.5)))
        self.assertGreaterEqual(inside, 5)

    def test_geometric_mode_saturates_most_of_a_raise_stretch_mode_does_not(self):
        # the real arm reaches only part of the model's shoulder travel: geometric keeps the angles faithful but part
        # of a human raise hits the ends; stretch spreads the whole raise over the safe range. (How MUCH geometric
        # saturates depends on the measured range -- <60% moving at 1500..1800, 70% at 1500..2100 -- so only the
        # relation is pinned.)
        def moving_fraction(mode):
            ts = [i / 40 for i in range(41)]
            ss = [real.pulses(ARM, CAL, fx._slerp(fx.HANG, fx.FORWARD, t), fx.STRAIGHT, mode=mode)[0] for t in ts]
            return sum(1 for a, b in zip(ss, ss[1:]) if b != a) / (len(ss) - 1)
        self.assertLess(moving_fraction("geometric"), moving_fraction("stretch"))
        self.assertLess(moving_fraction("geometric"), 1.0)            # it does saturate somewhere
        self.assertGreater(moving_fraction("stretch"), 0.9)

    def test_stretch_reaches_both_ends_of_the_safe_shoulder_range(self):
        lo, hi = env.shoulder_range(ARM.table)
        self.assertEqual(real.pulses(ARM, CAL, fx.HANG, fx.STRAIGHT, mode="stretch")[0], lo)
        self.assertEqual(real.pulses(ARM, CAL, fx.FORWARD, fx.STRAIGHT, mode="stretch")[0], hi)

    def test_the_default_mode_is_height_reach_the_users_choice(self):
        # 2026-09-27: stretch; 2026-10-03: the author chose height_reach (see HeightReachTest)
        self.assertEqual(real.DEFAULT_MODE, "height_reach")
        for raw in (fx.HANG, fx._slerp(fx.HANG, fx.FORWARD, 0.3), fx.FORWARD):
            self.assertEqual(real.pulses(ARM, CAL, raw, fx.STRAIGHT),
                             real.pulses(ARM, CAL, raw, fx.STRAIGHT, mode="height_reach"))

    def test_unknown_mode_is_refused(self):
        with self.assertRaises(ValueError):
            real.pulses(ARM, CAL, fx.HANG, fx.STRAIGHT, mode="banana")


if __name__ == "__main__":
    unittest.main()
