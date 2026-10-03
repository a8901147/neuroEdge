"""Path B direction-level tests, fully offline: the SAVED 9/13 calibration
vectors -> mearm_pathb -> mearm_scene.xml in MuJoCo, checking that the model
moves the way the arm does. No hardware, no serial.

Scope is deliberately COARSE (explicit, 2026-09-25): the big directions --
arm left => model left, arm up => model up, elbow bend => model folds -- plus
the few properties that keep a real servo from being slammed around
(continuity, no jitter near the hanging pose, no jump behind the body).
Tolerances are generous and physically motivated, not tuned to one run.
Exact anchors and the elbow's visual up/down convention are left to the
interactive calibration at the hardware.

The vectors below are the real 9/13 calibration (16:35, the one that a full
real 6-step task ran on), embedded as a fixture because
shoulder_calibration.json is personal, gitignored data. A future
recalibration should NOT require touching this file: only the properties are
asserted, never these numbers' derived outputs.

Model-frame fact used throughout: mearm_scene.xml has +x forward and +y to the
LEFT of forward, so "arm left" == tcp y > 0.
"""

import json
import math
import random
import sys
import unittest
from pathlib import Path

import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mearm_pathb as pb  # noqa: E402

SCENE_XML = Path(__file__).resolve().parent / "mearm_scene.xml"

# The committed golden sample (data/shoulder_calibration_golden_2026-09-13.json, v1.1.0) -- only the pose fields
SAVED_9_13 = {k: v for k, v in json.loads(
    (Path(__file__).resolve().parents[2] / "data" / "shoulder_calibration_golden_2026-09-13.json").read_text()).items()
    if k in ("baseline_raw", "forward_raw", "left_twist_raw", "right_twist_raw", "zero_elbow")}
HANG = tuple(SAVED_9_13["baseline_raw"])
FORWARD = tuple(SAVED_9_13["forward_raw"])
LEFT = tuple(SAVED_9_13["left_twist_raw"])
RIGHT = tuple(SAVED_9_13["right_twist_raw"])
STRAIGHT = SAVED_9_13["zero_elbow"]
FLEXED = STRAIGHT + pb.ELBOW_SWING_RAD

CAL = pb.make_calibration(SAVED_9_13)


# ---- test-only geometry helpers (deliberately NOT mearm_pathb's own code) ----

def _unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _angle(a, b):
    return math.acos(max(-1.0, min(1.0, _dot(_unit(a), _unit(b)))))


def _rotate(v, axis, angle):
    """Rodrigues rotation of v about unit `axis`."""
    a = _unit(axis)
    c, s = math.cos(angle), math.sin(angle)
    cross = (a[1] * v[2] - a[2] * v[1], a[2] * v[0] - a[0] * v[2], a[0] * v[1] - a[1] * v[0])
    d = _dot(a, v)
    return tuple(v[i] * c + cross[i] * s + a[i] * d * (1 - c) for i in range(3))


def _slerp(a, b, t):
    a, b = _unit(a), _unit(b)
    th = _angle(a, b)
    s = math.sin(th)
    return tuple((math.sin((1 - t) * th) * x + math.sin(t * th) * y) / s for x, y in zip(a, b))


def _sweep_about_hang(start, angle):
    """`start` rotated about the HANG axis by `angle`, sign chosen so that a
    positive angle moves toward LEFT (checked geometrically against the LEFT
    vector, not by using mearm_pathb's decode)."""
    h = _unit(HANG)
    probe = _rotate(_unit(FORWARD), h, 0.05)
    sign = 1.0 if _angle(probe, LEFT) < _angle(FORWARD, LEFT) else -1.0
    return _rotate(_unit(start), h, sign * angle)


# ---- MuJoCo side ----

_MODEL = mujoco.MjModel.from_xml_path(str(SCENE_XML))


def _elevation_deg(upper_arm_vec, base_angle):
    """Signed elevation of the upper arm above horizontal, measured along the
    base's heading. NOT atan2(z, hypot(x, y)): that is capped at 90deg, so an
    arm leaning back past vertical (true 98deg) wrongly reads as 82deg and looks
    like the model went DOWN when it kept rising (seen for real while checking
    the linkage fix -- the joint angle was tracking perfectly)."""
    heading = upper_arm_vec[0] * math.cos(base_angle) + upper_arm_vec[1] * math.sin(base_angle)
    return math.degrees(math.atan2(upper_arm_vec[2], heading))


def settle(base, shoulder, elbow, claw=1.0, steps=2500):
    d = mujoco.MjData(_MODEL)
    for name, value in (("base", base), ("shoulder", shoulder), ("elbow", elbow), ("claw", claw)):
        d.ctrl[_MODEL.actuator(name).id] = value
    for _ in range(steps):
        mujoco.mj_step(_MODEL, d)
    p = lambda body: d.xpos[_MODEL.body(body).id].copy()
    shoulder_p, elbow_p, wrist_p = p("upper_arm_link"), p("forearm_link"), p("tool_link")
    upper = elbow_p - shoulder_p
    interior = math.degrees(_angle(tuple(shoulder_p - elbow_p), tuple(wrist_p - elbow_p)))
    return {
        "tcp": d.site_xpos[_MODEL.site("tcp").id].copy(),
        "elevation_deg": _elevation_deg(upper, d.qpos[_MODEL.jnt_qposadr[_MODEL.joint("base").id]]),
        "elbow_interior_deg": interior,
    }


def pose(raw, elbow_bend=STRAIGHT):
    return pb.ctrl_from_sensors(CAL, raw, elbow_bend)


class DecodeSanityTest(unittest.TestCase):
    def test_left_and_right_are_symmetric_in_the_spherical_view(self):
        # The reason Path B uses (tilt, azimuth): the SAME real vectors that
        # the oblique decode leaks into pitch come out symmetric here.
        self.assertGreater(CAL.az_left, 0.1)
        self.assertLess(CAL.az_right, -0.1)
        self.assertLess(abs(CAL.az_left + CAL.az_right), 0.2 * abs(CAL.az_left))

    def test_left_right_forward_share_a_similar_tilt(self):
        tilts = [CAL.decode(v)[0] for v in (FORWARD, LEFT, RIGHT)]
        self.assertLess(max(tilts) - min(tilts), math.radians(15))

    def test_bad_calibrations_are_rejected_not_silently_used(self):
        with self.assertRaises(ValueError):
            pb.Calibration(HANG, FORWARD, LEFT, LEFT, STRAIGHT)          # right on the left side
        with self.assertRaises(ValueError):
            pb.Calibration(HANG, HANG, LEFT, RIGHT, STRAIGHT)            # FORWARD == HANG
        with self.assertRaises(KeyError):
            pb.make_calibration({k: v for k, v in SAVED_9_13.items() if k != "zero_elbow"})


class ConstantsMatchTheSceneTest(unittest.TestCase):
    def test_targets_equal_the_actuator_ranges_in_mearm_scene_xml(self):
        rng = lambda n: _MODEL.actuator_ctrlrange[_MODEL.actuator(n).id]
        self.assertAlmostEqual(pb.SHOULDER_RAISED, rng("shoulder")[0], places=6)
        self.assertAlmostEqual(pb.SHOULDER_REST, rng("shoulder")[1], places=6)
        self.assertAlmostEqual(pb.ELBOW_EXTENDED, rng("elbow")[0], places=6)
        self.assertAlmostEqual(pb.ELBOW_FOLDED, rng("elbow")[1], places=6)
        self.assertAlmostEqual(pb.BASE_LIMIT, rng("base")[1], places=6)
        self.assertLess(pb.BASE_SWING, pb.BASE_LIMIT)

    def test_linkage_constants_equal_the_scenes_tool_lock_and_limits(self):
        lo, hi = _MODEL.jnt_range[_MODEL.joint("tool").id]
        self.assertAlmostEqual(pb.TOOL_LIMIT[0], lo, places=6)
        self.assertAlmostEqual(pb.TOOL_LIMIT[1], hi, places=6)
        self.assertAlmostEqual(pb.TOOL_LOCK_SUM, _MODEL.eq_data[0][0], places=6)   # shoulder+elbow+tool == this


class BigDirectionsTest(unittest.TestCase):
    def test_arm_left_is_model_left_and_arm_right_is_model_right(self):
        left_y = settle(*pose(LEFT))["tcp"][1]
        right_y = settle(*pose(RIGHT))["tcp"][1]
        fwd_y = settle(*pose(FORWARD))["tcp"][1]
        self.assertGreater(left_y, 0.03, "arm swung left must put the model's tip on its left (+y)")
        self.assertLess(right_y, -0.03, "arm swung right must put the model's tip on its right (-y)")
        self.assertLess(abs(fwd_y), 0.02, "arm straight ahead must keep the model centered")

    def test_raising_the_arm_raises_the_model(self):
        rest = settle(*pose(HANG))["elevation_deg"]
        raised = settle(*pose(FORWARD))["elevation_deg"]
        self.assertGreater(raised - rest, 30.0)

    def test_elevation_rises_monotonically_from_hang_to_forward(self):
        elevations = [settle(*pose(_slerp(HANG, FORWARD, i / 8)))["elevation_deg"] for i in range(9)]
        for a, b in zip(elevations, elevations[1:]):
            self.assertGreaterEqual(b, a - 1.5)
        self.assertGreater(elevations[-1] - elevations[0], 30.0)

    def test_base_moves_the_right_way_from_forward_to_left_and_right(self):
        to_left = [pose(_slerp(FORWARD, LEFT, i / 6))[0] for i in range(7)]
        to_right = [pose(_slerp(FORWARD, RIGHT, i / 6))[0] for i in range(7)]
        self.assertTrue(all(b >= a - 1e-9 for a, b in zip(to_left, to_left[1:])))
        self.assertTrue(all(b <= a + 1e-9 for a, b in zip(to_right, to_right[1:])))
        self.assertGreater(to_left[-1], 0.3)
        self.assertLess(to_right[-1], -0.3)

    def test_bending_the_elbow_folds_the_model_at_any_shoulder_height(self):
        # Direction only, amplitude is window-limited: at a fixed shoulder the
        # linkage lets the elbow travel just ~32deg, so a 129deg human flexion
        # shows up as a small (but unmistakable, same-direction) fold.
        for name, raw in (("HANG", HANG), ("FORWARD", FORWARD)):
            straight = settle(*pose(raw, STRAIGHT))["elbow_interior_deg"]
            flexed = settle(*pose(raw, FLEXED))["elbow_interior_deg"]
            self.assertGreater(straight - flexed, 12.0,
                               f"{name}: flexing must reduce the model's elbow interior angle")

    def test_the_elbow_REQUEST_from_a_real_full_flexion_asks_for_nearly_a_full_fold_but_half_does_not(self):
        # INDEPENDENT of mearm_pathb.ELBOW_SWING_RAD (using it here would
        # just check the constant against itself -- a mutation test showed a
        # 20deg swing sailed through): 129deg is the real full-flexion swing
        # measured on hardware 9/05 (test_imu_to_mujoco.py ELBOW_FLEXION vs
        # REST, dot-product angle 17.7deg -> 146.7deg).
        real_full = math.radians(129.0)
        span = pb.ELBOW_FOLDED - pb.ELBOW_EXTENDED
        full = pb.elbow_request(CAL, STRAIGHT + real_full)
        half = pb.elbow_request(CAL, STRAIGHT + real_full / 2)
        self.assertGreaterEqual(full, pb.ELBOW_EXTENDED + 0.9 * span,
                                "a real full elbow flexion must fold the model nearly all the way")
        self.assertGreater(half, pb.ELBOW_EXTENDED + 0.25 * span)
        self.assertLess(half, pb.ELBOW_EXTENDED + 0.75 * span,
                        "half a real flexion must not already be saturated")

    def test_elbow_ctrl_rises_monotonically_with_flexion(self):
        ctrls = [pose(HANG, STRAIGHT + i * pb.ELBOW_SWING_RAD / 10)[2] for i in range(11)]
        self.assertTrue(all(b >= a for a, b in zip(ctrls, ctrls[1:])))

    def test_hanging_with_a_straight_arm_is_exactly_the_rest_pose(self):
        base, shoulder, elbow = pose(HANG, STRAIGHT)
        # places=6, not 9: tilt at HANG is acos(~1), which amplifies float
        # rounding to ~1e-8 rad (seen for real: 1.7e-8) -- harmless.
        self.assertAlmostEqual(base, 0.0, places=6)
        self.assertAlmostEqual(shoulder, pb.SHOULDER_REST, places=6)
        # the linkage only allows elbow >= its window's lower edge here (a hair
        # above the actuator's own EXTENDED end)
        self.assertAlmostEqual(elbow, pb.elbow_window(pb.SHOULDER_REST)[0], places=6)
        self.assertLess(abs(elbow - pb.ELBOW_EXTENDED), 0.05)


class FadeIntentTest(unittest.TestCase):
    """The two fades have a documented purpose, so they are checked as properties rather than as pinned window numbers:
    the MeArm cannot reach behind the body, and the base output must not be attenuated once the arm is clearly raised."""

    def test_well_behind_the_body_the_arm_is_at_rest_not_half_way(self):
        # (found by a mutation run: widening the fade to 180deg left the arm 1/3 raised at az 150deg, and nothing noticed)
        for sign in (+1, -1):
            for az_deg in (150.0, 165.0, 179.0):
                raw = _sweep_about_hang(FORWARD, sign * math.radians(az_deg))
                tilt, az = CAL.decode(raw)
                self.assertGreater(abs(math.degrees(az)), 145.0)
                base, shoulder, _e = pose(raw)
                self.assertEqual(base, 0.0, (sign, az_deg))
                self.assertAlmostEqual(shoulder, pb.SHOULDER_REST, places=9)

    def test_above_the_calibrations_own_minimum_tilt_the_base_output_is_not_attenuated(self):
        # the calibration requires FORWARD >= 20deg from HANG, so from 20deg on the azimuth is trustworthy and the fade
        # must be over: at exactly the calibrated LEFT azimuth the base must give the full calibrated swing
        tilt_left = _angle(_unit(HANG), _unit(LEFT))
        for tilt_deg in (22.0, 30.0, 45.0):
            raw = _slerp(HANG, LEFT, math.radians(tilt_deg) / tilt_left)
            tilt, az = CAL.decode(raw)
            self.assertAlmostEqual(math.degrees(tilt), tilt_deg, delta=0.5)
            self.assertAlmostEqual(az, CAL.az_left, places=6)
            self.assertAlmostEqual(pose(raw)[0], pb.BASE_SWING, places=6, msg=f"tilt {tilt_deg}")


class DecouplingTest(unittest.TestCase):
    def test_a_left_or_right_swing_does_not_also_lift_the_shoulder(self):
        # The defect the oblique decode has on this same data (right swing
        # decodes to pitch +1.08 -> shoulder ~85% raised). Tolerance is 15%
        # of the shoulder's whole travel.
        span = pb.SHOULDER_REST - pb.SHOULDER_RAISED
        fwd = pose(FORWARD)[1]
        for name, raw in (("LEFT", LEFT), ("RIGHT", RIGHT)):
            self.assertLess(abs(pose(raw)[1] - fwd), 0.15 * span, f"{name} swing moved the shoulder")

    def test_elbow_only_motion_cannot_move_base_or_shoulder(self):
        for raw in (HANG, FORWARD, LEFT, RIGHT):
            a = pose(raw, STRAIGHT)
            b = pose(raw, FLEXED)
            self.assertEqual(a[:2], b[:2])

    def test_the_elbows_position_inside_its_window_does_not_depend_on_the_shoulder(self):
        # The shoulder may shift the WINDOW (that is the linkage coupling), but a
        # given human bend must land at the same fraction of it at every height.
        for bend in (STRAIGHT, STRAIGHT + 0.6, STRAIGHT + 1.4, FLEXED):
            fractions = []
            for raw in (HANG, _slerp(HANG, FORWARD, 0.5), FORWARD):
                _b, shoulder, elbow = pose(raw, bend)
                lo, hi = pb.elbow_window(shoulder)
                fractions.append((elbow - lo) / (hi - lo))
            self.assertLess(max(fractions) - min(fractions), 1e-9)


class LinkageCouplingTest(unittest.TestCase):
    """Path B respects the MeArm's shoulder/elbow linkage coupling."""


    def test_nan_or_infinite_inputs_still_give_a_finite_elbow_inside_the_scene_range(self):
        # mirrors the C++ port's guarantee (tests/test_mearm_linkage.cpp); a NaN
        # reaching data.ctrl would poison the whole MuJoCo state.
        nan, inf = float("nan"), float("inf")
        for s in (0.5, nan, inf, -inf):
            for r in (1.5, nan, inf, -inf):
                with self.subTest(shoulder=s, request=r):
                    e = pb.project_elbow(s, r)
                    self.assertTrue(math.isfinite(e))
                    self.assertGreaterEqual(e, pb.ELBOW_EXTENDED - 1e-9)
                    self.assertLessEqual(e, pb.ELBOW_FOLDED + 1e-9)
    def test_shoulder_plus_elbow_is_always_inside_the_linkage_band(self):
        band_lo, band_hi = pb.linkage_band()
        rng = random.Random(11)
        for _ in range(4000):
            raw = tuple(rng.uniform(-1.5, 1.5) for _ in range(3))
            if math.hypot(*raw) < 1e-3:
                continue
            _b, shoulder, elbow = pose(raw, rng.uniform(0.0, 3.2))
            self.assertGreaterEqual(shoulder + elbow, band_lo - 1e-9)
            self.assertLessEqual(shoulder + elbow, band_hi + 1e-9)

    def test_the_band_kept_is_strictly_inside_what_the_scene_allows(self):
        # margin: the passive tool link must never sit exactly on its limit
        lo, hi = _MODEL.jnt_range[_MODEL.joint("tool").id]
        scene_lo, scene_hi = math.pi / 2 - hi, math.pi / 2 - lo
        band_lo, band_hi = pb.linkage_band()
        self.assertGreater(band_lo, scene_lo)
        self.assertLess(band_hi, scene_hi)

    def test_the_window_is_never_empty_at_any_shoulder(self):
        for i in range(101):
            shoulder = pb.SHOULDER_RAISED + (pb.SHOULDER_REST - pb.SHOULDER_RAISED) * i / 100
            lo, hi = pb.elbow_window(shoulder)
            self.assertGreater(hi - lo, 0.3)
            self.assertGreaterEqual(lo, pb.ELBOW_EXTENDED - 1e-12)
            self.assertLessEqual(hi, pb.ELBOW_FOLDED + 1e-12)

    def test_a_straight_arm_raised_high_folds_the_forearm_as_the_mechanism_requires(self):
        # Direction of the (unavoidable) compromise: same straight human elbow,
        # higher shoulder -> the model's forearm sits at a MORE folded setting.
        low = pose(HANG, STRAIGHT)[2]
        high = pose(FORWARD, STRAIGHT)[2]
        self.assertGreater(high, low + 0.5)

    def test_raising_the_arm_with_a_fixed_elbow_changes_the_elbow_smoothly(self):
        prev = None
        for i in range(101):
            elbow = pose(_slerp(HANG, FORWARD, i / 100), STRAIGHT + 0.7)[2]
            if prev is not None:
                self.assertLess(abs(elbow - prev), 0.05)
            prev = elbow

    def test_elbow_flexion_still_moves_the_elbow_the_same_way_at_every_height(self):
        for raw in (HANG, _slerp(HANG, FORWARD, 0.5), FORWARD):
            vals = [pose(raw, STRAIGHT + i * pb.ELBOW_SWING_RAD / 8)[2] for i in range(9)]
            self.assertTrue(all(b >= a for a, b in zip(vals, vals[1:])))
            self.assertGreater(vals[-1] - vals[0], 0.4)


class SafetyForRealServosTest(unittest.TestCase):
    def test_no_jitter_near_hanging(self):
        # Azimuth is undefined at tilt 0. Below the fade-in window the base
        # output must be EXACTLY zero whatever the azimuth noise does.
        h = _unit(HANG)
        for az_deg in range(0, 360, 15):
            v = _rotate(_rotate(h, (0.0, 1.0, 0.0), math.radians(5.0)), h, math.radians(az_deg))
            self.assertEqual(pose(v)[0], 0.0, f"base moved at 5deg tilt, azimuth {az_deg}")

    def test_sensor_noise_near_rest_keeps_base_quiet(self):
        rng = random.Random(1)
        bases = [pose(tuple(c + rng.gauss(0, 0.02) for c in HANG))[0] for _ in range(400)]
        mean = sum(bases) / len(bases)
        std = math.sqrt(sum((b - mean) ** 2 for b in bases) / len(bases))
        self.assertLess(std, 0.05)

    def test_left_to_right_sweep_is_continuous(self):
        prev = pose(_slerp(LEFT, RIGHT, 0.0))
        for i in range(1, 201):
            cur = pose(_slerp(LEFT, RIGHT, i / 200))
            self.assertLess(abs(cur[0] - prev[0]), 0.1)
            self.assertLess(abs(cur[1] - prev[1]), 0.05)
            prev = cur

    def test_a_full_circle_around_the_hang_axis_never_jumps(self):
        # Includes passing directly BEHIND the body, where azimuth flips
        # sign at +-180deg and a hard switch would slam the base servo.
        prev = None
        for i in range(721):
            v = _sweep_about_hang(FORWARD, math.radians(i * 0.5))
            cur = pose(v)
            if prev is not None:
                self.assertLess(abs(cur[0] - prev[0]), 0.1, f"base jumped at {i * 0.5} deg")
                self.assertLess(abs(cur[1] - prev[1]), 0.1, f"shoulder jumped at {i * 0.5} deg")
            prev = cur

    def test_directly_behind_the_body_is_treated_as_rest(self):
        base, shoulder, _ = pose(_sweep_about_hang(FORWARD, math.pi))
        self.assertAlmostEqual(base, 0.0, places=9)
        self.assertAlmostEqual(shoulder, pb.SHOULDER_REST, places=9)

    def test_no_reading_means_rest_never_a_guess(self):
        for raw in (None, (None, None, None), (0.0, 0.0, 0.0)):
            self.assertEqual(pose(raw, STRAIGHT), pb.REST_CTRL)

    def test_outputs_are_always_finite_and_inside_the_actuator_ranges(self):
        rng = random.Random(7)
        for _ in range(3000):
            raw = tuple(rng.uniform(-1.5, 1.5) for _ in range(3))
            if math.hypot(*raw) < 1e-3:
                continue
            base, shoulder, elbow = pose(raw, rng.uniform(0.0, 3.2))
            for v in (base, shoulder, elbow):
                self.assertTrue(math.isfinite(v))
            self.assertLessEqual(abs(base), pb.BASE_LIMIT)
            self.assertLessEqual(pb.SHOULDER_RAISED, shoulder)
            self.assertLessEqual(shoulder, pb.SHOULDER_REST)
            self.assertLessEqual(pb.ELBOW_EXTENDED, elbow)
            self.assertLessEqual(elbow, pb.ELBOW_FOLDED)


# ---------------------------------------------------------------------------
# REAL live-session replay. The user's own `run_demo_live.py --mearm` terminal
# output of 2026-09-24 (9/13 calibration loaded), 86 consecutive lines of
# (printed shoulder_pitch, printed shoulder_roll, printed elbow_ctrl) while
# they lifted the arm toward the left-front and lowered it again. They
# reported "left/right works, lifting doesn't". That log has no raw
# accelerometer vector, but Path A's decode keeps the DIRECTION exactly
# (tilt = hypot(pitch_equiv, roll_equiv); tangent direction proportional to
# pitch_equiv*pf + roll_equiv*pa), so each row is inverted back to a raw
# direction and fed through Path B. (Printed pitch = clamp(-pitch_equiv) and
# roll = clamp(roll_equiv); no row in this log is clamped.)
# ---------------------------------------------------------------------------
REAL_LOG_2026_09_24 = [
    (-0.447, +0.495, +1.226), (-0.444, +0.498, +1.224), (-0.444, +0.500, +1.227), (-0.437, +0.503, +1.223),
    (-0.430, +0.508, +1.219), (-0.427, +0.514, +1.221), (-0.433, +0.502, +1.221), (-0.511, +0.456, +1.240),
    (+0.022, +0.747, +1.238), (+0.106, +0.783, +1.192), (+0.172, +0.870, +1.227), (+0.196, +0.945, +1.196),
    (+0.265, +0.917, +1.194), (+0.254, +0.949, +1.118), (+0.020, +0.951, +1.115), (+0.075, +0.999, +1.189),
    (+0.106, +1.071, +1.173), (+0.136, +1.143, +1.220), (+0.167, +1.212, +1.197), (+0.173, +1.276, +1.197),
    (+0.200, +1.340, +1.190), (+0.161, +1.400, +1.178), (+0.199, +1.424, +1.174), (+0.235, +1.477, +1.149),
    (+0.227, +1.506, +1.137), (+0.263, +1.560, +1.120), (+0.254, +1.613, +1.137), (+0.306, +1.631, +1.100),
    (+0.302, +1.673, +1.104), (+0.310, +1.723, +1.085), (+0.343, +1.752, +1.081), (+0.387, +1.780, +1.081),
    (+0.364, +1.841, +1.080), (+0.409, +1.862, +1.051), (+0.498, +1.860, +1.070), (+0.481, +1.878, +1.057),
    (+0.480, +1.887, +1.053), (+0.502, +1.888, +1.048), (+0.513, +1.908, +1.059), (+0.542, +1.905, +1.055),
    (+0.553, +1.918, +1.042), (+0.536, +1.917, +1.048), (+0.502, +1.907, +1.044), (+0.502, +1.894, +1.058),
    (+0.516, +1.910, +1.042), (+0.588, +1.895, +1.052), (+0.534, +1.888, +1.046), (+0.483, +1.886, +1.050),
    (+0.561, +1.877, +1.048), (+0.611, +1.901, +1.066), (+0.599, +1.936, +1.045), (+0.624, +1.939, +1.067),
    (+0.625, +1.947, +1.045), (+0.640, +1.956, +1.040), (+0.666, +1.948, +1.045), (+0.642, +1.933, +1.044),
    (+0.582, +1.926, +1.040), (+0.450, +1.893, +1.038), (+0.321, +1.838, +1.045), (+0.255, +1.758, +1.069),
    (+0.156, +1.687, +1.071), (+0.112, +1.571, +1.113), (+0.059, +1.524, +1.142), (+0.030, +1.399, +1.140),
    (+0.024, +1.255, +1.154), (+0.014, +1.164, +1.152), (-0.045, +1.127, +1.186), (-0.083, +1.105, +1.169),
    (-0.104, +1.101, +1.121), (+0.030, +0.987, +1.080), (+0.057, +0.959, +1.100), (+0.082, +0.948, +1.091),
    (+0.086, +0.916, +1.097), (+0.105, +0.901, +1.092), (+0.109, +0.886, +1.077), (+0.150, +0.867, +1.072),
    (+0.157, +0.859, +1.059), (+0.136, +0.895, +1.075), (+0.197, +1.011, +1.179), (+0.204, +1.143, +1.178),
    (+0.228, +1.168, +1.159), (+0.207, +1.108, +1.197), (+0.046, +0.989, +1.137), (-0.074, +0.904, +1.182),
    (+0.016, +0.895, +1.061), (+0.017, +0.866, +1.115),
]


def _reconstruct_direction(pitch_printed, roll_printed):
    """Inverse of run_demo_live.oblique_decompose_scaled (direction only --
    magnitude of a raw accel vector is irrelevant to every decode here)."""
    import run_demo_live as rdl
    basis = rdl.make_oblique_basis(HANG, FORWARD, LEFT)
    pitch_equiv, roll_equiv = -pitch_printed, roll_printed
    tilt = math.hypot(pitch_equiv, roll_equiv)
    t = tuple(pitch_equiv * a + roll_equiv * b for a, b in zip(basis["pf"], basis["pa"]))
    n = math.sqrt(sum(c * c for c in t))
    return tuple(math.cos(tilt) * r + math.sin(tilt) * c / n for r, c in zip(basis["ref"], t)), basis


class RealLiveLogReplayTest(unittest.TestCase):
    """Path B against real logged motion, not synthetic poses."""

    @classmethod
    def setUpClass(cls):
        import run_demo_live as rdl
        cls.rdl = rdl
        cls.rows = []
        for pitch, roll, elbow_ctrl in REAL_LOG_2026_09_24:
            v, basis = _reconstruct_direction(pitch, roll)
            tilt, az = CAL.decode(v)
            elbow_bend = STRAIGHT + 1.28 - elbow_ctrl        # invert run_demo_live's elbow_ctrl formula
            base, shoulder, elbow = pose(v, elbow_bend)
            legacy_shoulder = rdl.rescale(max(rdl.SHOULDER_PITCH_RANGE[0], min(rdl.SHOULDER_PITCH_RANGE[1], pitch)),
                                          *rdl.SHOULDER_PITCH_RANGE, *rdl.MEARM_SHOULDER_CTRL_RANGE)
            cls.rows.append({"v": v, "basis": basis, "pitch": pitch, "roll": roll, "tilt": tilt, "az": az,
                              "base": base, "shoulder": shoulder, "legacy_shoulder": legacy_shoulder})

    def test_the_inverse_reproduces_the_logged_numbers(self):
        # Guards the replay itself: decoding each reconstructed vector with
        # Path A's own decode must give back what the log printed.
        for row in self.rows:
            pe, re_ = self.rdl.oblique_decompose_scaled(row["basis"], row["v"])
            self.assertAlmostEqual(-pe, row["pitch"], places=2)
            self.assertAlmostEqual(re_, row["roll"], places=2)

    def test_the_real_motion_really_was_a_big_raise_to_the_left_front(self):
        # Documents what the user actually did (they said they lifted high
        # enough; the data agrees): above horizontal, and always left of forward.
        tilts = [math.degrees(r["tilt"]) for r in self.rows]
        azimuths = [math.degrees(r["az"]) for r in self.rows]
        self.assertGreater(max(tilts), 100.0)
        self.assertGreater(min(azimuths), 0.0)

    def test_path_b_lifts_the_shoulder_for_a_real_raise(self):
        # smaller ctrl == higher elevation
        low_tilt = min(self.rows, key=lambda r: r["tilt"])
        high_tilt = max(self.rows, key=lambda r: r["tilt"])
        self.assertGreater(low_tilt["shoulder"], 0.2)
        self.assertLess(high_tilt["shoulder"], pb.SHOULDER_RAISED + 0.05)

    def test_shoulder_ctrl_never_rises_as_the_arm_rises(self):
        ordered = sorted(self.rows, key=lambda r: r["tilt"])
        for a, b in zip(ordered, ordered[1:]):
            self.assertLessEqual(b["shoulder"], a["shoulder"] + 1e-9)

    def test_path_b_uses_clearly_more_of_the_shoulders_travel_than_path_a_did(self):
        swing = lambda key: max(r[key] for r in self.rows) - min(r[key] for r in self.rows)
        self.assertGreater(swing("shoulder"), 1.4 * swing("legacy_shoulder"),
                           "the same real raise must move the shoulder clearly more than the old mapping did")

    def test_base_stays_on_the_left_while_the_arm_stays_left_of_forward(self):
        for r in self.rows:
            self.assertGreater(r["base"], 0.3)

    def test_no_single_step_of_the_real_motion_slams_the_shoulder(self):
        for a, b in zip(self.rows, self.rows[1:]):
            self.assertLess(abs(b["shoulder"] - a["shoulder"]), 0.3)


def _average_ranks(values):
    """Tie-aware ranks (the raise saturates at the model's highest elevation,
    so ties are real and an arbitrary tie-break would skew the correlation)."""
    import numpy as np
    values = np.asarray(values, dtype=float)
    ranks = np.argsort(np.argsort(values)).astype(float)
    for u in np.unique(values):
        tied = values == u
        ranks[tied] = ranks[tied].mean()
    return ranks


def spearman(a, b):
    import numpy as np
    return float(np.corrcoef(_average_ranks(a), _average_ranks(b))[0, 1])


STEPS_PER_LOG_ROW = 200    # one log row == 0.2 s: the preview prints every 200 sim steps at 1 ms


def simulate_log(controller):
    """Runs the REAL logged motion through MuJoCo as one continuous
    simulation (one ctrl update per 0.2 s log row), not settle-per-pose.
    `controller(v, elbow_bend, pitch, roll, elbow_ctrl)` -> (base, shoulder, elbow) ctrl."""
    import numpy as np
    d = mujoco.MjData(_MODEL)
    d.ctrl[_MODEL.actuator("shoulder").id] = pb.SHOULDER_REST
    d.ctrl[_MODEL.actuator("elbow").id] = pb.ELBOW_EXTENDED
    d.ctrl[_MODEL.actuator("claw").id] = 1.0
    for _ in range(2000):
        mujoco.mj_step(_MODEL, d)
    q = lambda name: d.qpos[_MODEL.jnt_qposadr[_MODEL.joint(name).id]]
    out = {"tilt": [], "az": [], "cmd": [], "q_shoulder": [], "elev": [], "tcp": [], "tcp_az": [], "ncon": []}
    for pitch, roll, elbow_ctrl in REAL_LOG_2026_09_24:
        v, _ = _reconstruct_direction(pitch, roll)
        tilt, az = CAL.decode(v)
        cmd = controller(v, STRAIGHT + 1.28 - elbow_ctrl, pitch, roll, elbow_ctrl)
        for name, value in zip(("base", "shoulder", "elbow"), cmd):
            d.ctrl[_MODEL.actuator(name).id] = value
        for _ in range(STEPS_PER_LOG_ROW):
            mujoco.mj_step(_MODEL, d)
        upper = d.xpos[_MODEL.body("forearm_link").id] - d.xpos[_MODEL.body("upper_arm_link").id]
        tcp = d.site_xpos[_MODEL.site("tcp").id].copy()
        out["tilt"].append(math.degrees(tilt))
        out["az"].append(math.degrees(az))
        out["cmd"].append(cmd)
        out["q_shoulder"].append(q("shoulder"))
        out["elev"].append(_elevation_deg(upper, q("base")))
        out["tcp"].append(tcp)
        out["tcp_az"].append(math.degrees(math.atan2(tcp[1], tcp[0])))
        out["ncon"].append(d.ncon)
    return out


def _legacy_controller(v, elbow_bend, pitch, roll, elbow_ctrl):
    """What --mearm sent BEFORE Path B existed (full-anatomical-ROM rescale of
    Path A's oblique decode) -- reproduced only as the comparison baseline."""
    import run_demo_live as rdl
    sp = max(rdl.SHOULDER_PITCH_RANGE[0], min(rdl.SHOULDER_PITCH_RANGE[1], pitch))
    sr = max(rdl.SHOULDER_ROLL_RANGE[0], min(rdl.SHOULDER_ROLL_RANGE[1], roll))
    return (rdl.rescale(sr, *rdl.SHOULDER_ROLL_RANGE, *rdl.MEARM_BASE_CTRL_RANGE),
            rdl.rescale(sp, *rdl.SHOULDER_PITCH_RANGE, *rdl.MEARM_SHOULDER_CTRL_RANGE),
            rdl.rescale(elbow_ctrl, *rdl.ELBOW_RANGE, rdl.MEARM_ELBOW_CTRL_RANGE[1], rdl.MEARM_ELBOW_CTRL_RANGE[0]))


class SpearmanHelperTest(unittest.TestCase):
    def test_monotone_reversed_and_tied_sequences(self):
        self.assertAlmostEqual(spearman([1, 2, 3, 4], [10, 20, 30, 40]), 1.0)
        self.assertAlmostEqual(spearman([1, 2, 3, 4], [40, 30, 20, 10]), -1.0)
        # ties (as when the raise saturates) are averaged, not arbitrarily ordered
        # by hand: ranks [0,1,2,3,4] vs [0,1,3,3,3] -> cov 8, var 10 and 8 -> 8/sqrt(80)
        self.assertAlmostEqual(spearman([1, 2, 3, 4, 5], [1, 2, 3, 3, 3]), 8 / 80 ** 0.5, places=9)
        self.assertAlmostEqual(spearman([1, 2, 3, 4, 5], [5, 5, 4, 2, 1]), -0.9746794344808963, places=9)


class RealLogTrajectoryInMuJoCoTest(unittest.TestCase):
    """The real logged motion driven through MuJoCo IN TIME: is the trajectory
    the MODEL actually executes the expected one? (Steady-state pose checks
    elsewhere in this file cannot see lag or a constraint fighting the servo.)"""

    @classmethod
    def setUpClass(cls):
        cls.b = simulate_log(lambda v, bend, *_: pose(v, bend))
        cls.a = simulate_log(_legacy_controller)

    def test_model_elevation_follows_how_high_the_arm_is_raised_below_the_calibrated_forward_pose(self):
        # Split at the calibrated FORWARD tilt on purpose: beyond it the shoulder
        # command is DESIGNED to saturate at its highest (nearly half of this real
        # log is above it), and a saturated stretch is all ties -- one overall
        # rank correlation would just measure how the ties happen to be ordered
        # (0.86 overall vs 0.99 below, on the same run).
        import numpy as np
        tilt, elev = np.array(self.b["tilt"]), np.array(self.b["elev"])
        below = tilt < math.degrees(CAL.tilt_forward)
        self.assertGreater(below.sum(), 20)
        self.assertGreater(spearman(tilt[below], elev[below]), 0.95)

    def test_model_stays_at_its_highest_elevation_while_the_arm_is_raised_beyond_forward(self):
        import numpy as np
        tilt, elev = np.array(self.b["tilt"]), np.array(self.b["elev"])
        above = tilt >= math.degrees(CAL.tilt_forward)
        self.assertGreater(above.sum(), 20)
        self.assertLess(elev[above].max() - elev[above].min(), 4.0, "must hold at the top, not fall back")
        self.assertGreater(elev[above].min(), elev[~above].max() - 1.0, "must be at least as high as anywhere below")

    def test_model_tip_direction_follows_the_arms_direction(self):
        self.assertGreater(spearman(self.b["az"], self.b["tcp_az"]), 0.85)

    def test_model_tip_stays_on_the_left_the_whole_time(self):
        self.assertGreater(min(t[1] for t in self.b["tcp"]), 0.03)

    def test_no_collisions_and_no_teleporting_between_frames(self):
        import numpy as np
        self.assertEqual(max(self.b["ncon"]), 0)
        steps = [np.linalg.norm(b - a) for a, b in zip(self.b["tcp"], self.b["tcp"][1:])]
        self.assertLess(max(steps), 0.15)     # metres per 0.2 s

    def test_path_b_fixes_the_direction_the_old_mapping_got_backwards(self):
        # Before Path B, on this SAME real motion, the model's elevation moved
        # AGAINST how high the arm was raised (rank correlation ~ -0.87) and its
        # tip even crossed to the right. Path B must be the opposite on both.
        self.assertLess(spearman(self.a["tilt"], self.a["elev"]), 0.0)
        self.assertGreater(spearman(self.b["tilt"], self.b["elev"]), 0.0)
        self.assertLess(min(t[1] for t in self.a["tcp"]), min(t[1] for t in self.b["tcp"]))

    def test_path_b_swings_the_model_more_than_the_old_mapping(self):
        swing = lambda r: max(r["elev"]) - min(r["elev"])
        self.assertGreater(swing(self.b), 1.5 * swing(self.a))

    # --- known limitation, kept as executable documentation ----------------
    # MeArmPilot's measured passive "tool" link only allows a band of
    # (shoulder + elbow) joint-angle sums (mearm_scene.xml: tool = pi/2 -
    # shoulder - elbow, tool limited to its joint range). Path B maps shoulder
    # and elbow INDEPENDENTLY, so on this real motion every commanded pose
    # violates the band; the constraint solver then pins the tool link and the
    # shoulder cannot reach its command (RMS ~0.31 rad, vs ~0.01 with the limit
    # removed -- shown with servo gain x10 making it WORSE, i.e. not a servo
    # problem). On a real MEArm the same coupling would show up as a binding
    # linkage, not just a lagging sim. expectedFailure: this flips to an
    # "unexpected success" (reported as a failure) once the mapping respects the
    # coupling, which is the cue to delete the decorator.

    @staticmethod
    def _feasible_sum_band():
        lo, hi = _MODEL.jnt_range[_MODEL.joint("tool").id]
        return math.pi / 2 - hi, math.pi / 2 - lo

    def test_commanded_shoulder_plus_elbow_stays_inside_the_linkage_band(self):
        band_lo, band_hi = self._feasible_sum_band()
        for _base, shoulder, elbow in self.b["cmd"]:
            self.assertGreaterEqual(shoulder + elbow, band_lo)
            self.assertLessEqual(shoulder + elbow, band_hi)

    def test_model_reaches_the_shoulder_position_it_was_commanded(self):
        import numpy as np
        errors = np.array(self.b["q_shoulder"]) - np.array([c[1] for c in self.b["cmd"]])
        self.assertLess(float(np.sqrt(np.mean(errors ** 2))), 0.1)


class InterpAnchorsTest(unittest.TestCase):
    def test_a_nan_x_is_rejected_clearly_not_with_an_unbound_local(self):
        with self.assertRaises(ValueError):
            pb.interp_anchors([(0.0, 0.0), (1.0, 1.0)], float("nan"), -1, 1)

    def test_ctrl_from_sensors_survives_non_finite_sensor_values(self):
        cal = pb.make_calibration(SAVED_9_13)
        inf, nan = float("inf"), float("nan")
        for raw in (LEFT, (inf, 0.0, 1.0), (nan, 0.0, 1.0), (0.0, 0.0, 0.0), None):
            for bend in (STRAIGHT, inf, -inf, nan):
                with self.subTest(raw=raw, bend=bend):
                    out = pb.ctrl_from_sensors(cal, raw, bend)
                    self.assertTrue(all(math.isfinite(v) for v in out), out)

    def test_exact_at_anchors_clamped_outside_and_rejects_bad_input(self):
        anchors = [(-0.5, -0.9), (0.0, 0.0), (0.5, 0.9)]
        for x, y in anchors:
            self.assertAlmostEqual(pb.interp_anchors(anchors, x, -2, 2), y, places=9)
        self.assertEqual(pb.interp_anchors(anchors, 99.0, -1.0, 1.0), 1.0)
        with self.assertRaises(ValueError):
            pb.interp_anchors([(0.0, 0.0)], 0.0, -1, 1)
        with self.assertRaises(ValueError):
            pb.interp_anchors([(1.0, 0.0), (1.0, 1.0)], 1.0, -1, 1)


if __name__ == "__main__":
    unittest.main()
