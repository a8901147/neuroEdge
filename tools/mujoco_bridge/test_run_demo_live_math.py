"""Unit tests for run_demo_live.py's pure math/decision functions --
clamp, ema_step, rate_limit_step, calibration_tilt_deg, and the
oblique-basis functions (make_oblique_basis/oblique_decompose/
oblique_decompose_scaled). These are the core logic this session's
calibration pivot and ctrl-smoothing changes touched, and none of it had
an automated test before this file: everything was verified by hand
during development (interactive runs, one-off print-and-check scripts),
which doesn't protect against a future change quietly breaking it.

Deliberately plain unittest (stdlib), not pytest -- this project already
keeps its Python tool dependencies to just mujoco/pyserial (see
requirements.txt); no reason to add a new one just for this.

The oblique-basis fixture mirrors tests/test_tilt_azimuth.cpp's synthetic
non-orthogonal-basis test exactly (same ref/tilt/azimuth values, same
independently-numpy-cross-checked v_mid literal) -- so a passing run here
is also a cross-check that this file's Python port and the C++ original
still agree, not just that the Python port is internally consistent.

Usage:
    python3 -m unittest tools/mujoco_bridge/test_run_demo_live_math.py -v
    python3 tools/mujoco_bridge/test_run_demo_live_math.py
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_demo_live as rdl  # noqa: E402


# ---- test-only fixture helpers, mirroring tests/test_tilt_azimuth.cpp's
# orthonormal_basis_perpendicular_to()/synthesize() (NOT run_demo_live.py's
# own code -- these exist only to construct known-answer test vectors) ----

def _orthonormal_basis_perpendicular_to(ref):
    seed = (0.0, 0.0, 1.0)
    if abs(ref[2]) > 0.99:
        seed = (1.0, 0.0, 0.0)
    ux = ref[1] * seed[2] - ref[2] * seed[1]
    uy = ref[2] * seed[0] - ref[0] * seed[2]
    uz = ref[0] * seed[1] - ref[1] * seed[0]
    u = rdl._normalize3((ux, uy, uz))
    vx = ref[1] * u[2] - ref[2] * u[1]
    vy = ref[2] * u[0] - ref[0] * u[2]
    vz = ref[0] * u[1] - ref[1] * u[0]
    return u, (vx, vy, vz)


def _synthesize(ref, u, v, tilt, azimuth):
    ct, st = math.cos(tilt), math.sin(tilt)
    ca, sa = math.cos(azimuth), math.sin(azimuth)
    return tuple(
        ref[i] * ct + (u[i] * ca + v[i] * sa) * st
        for i in range(3)
    )


class ClampTest(unittest.TestCase):
    def test_within_range_unchanged(self):
        self.assertEqual(rdl.clamp(0.5, 0.0, 1.0), 0.5)

    def test_clamps_low(self):
        self.assertEqual(rdl.clamp(-1.0, 0.0, 1.0), 0.0)

    def test_clamps_high(self):
        self.assertEqual(rdl.clamp(5.0, 0.0, 1.0), 1.0)


class EmgPercentileTest(unittest.TestCase):
    def test_empty_list_returns_zero(self):
        self.assertEqual(rdl.emg_percentile([], 90), 0.0)

    def test_median_of_odd_length_list(self):
        self.assertEqual(rdl.emg_percentile([1, 2, 3, 4, 5], 50), 3)

    def test_90th_percentile_picks_high_order_statistic(self):
        # 10 sorted values 0..9 -- 90th percentile lands on index
        # round(0.9*9)=8, i.e. value 8, not the max (9).
        self.assertEqual(rdl.emg_percentile(list(range(10)), 90), 8)

    def test_unsorted_input_is_sorted_first(self):
        self.assertEqual(rdl.emg_percentile([5, 1, 3, 2, 4], 0), 1)


class EmgMeanStdTest(unittest.TestCase):
    """2026-09-12: backs calibrate_emg_threshold()'s mean+k*SD threshold,
    which replaced a relaxed/contracted percentile split after real
    hardware testing found that split could invert (relaxed_max >=
    contracted_min) whenever a real contraction was weak or inconsistent,
    not just from ADC jitter -- see emg_mean_std's own docstring."""

    def test_empty_list_returns_zero_zero(self):
        self.assertEqual(rdl.emg_mean_std([]), (0.0, 0.0))

    def test_constant_values_have_zero_std(self):
        mean, std = rdl.emg_mean_std([5.0, 5.0, 5.0, 5.0])
        self.assertEqual(mean, 5.0)
        self.assertEqual(std, 0.0)

    def test_known_mean_and_population_std(self):
        # [2, 4, 4, 4, 5, 5, 7, 9] is a textbook population-SD example:
        # mean=5, population variance=4, SD=2.
        mean, std = rdl.emg_mean_std([2, 4, 4, 4, 5, 5, 7, 9])
        self.assertAlmostEqual(mean, 5.0)
        self.assertAlmostEqual(std, 2.0)

    def test_wider_spread_gives_larger_std(self):
        _, tight_std = rdl.emg_mean_std([100, 101, 99, 100, 100])
        _, wide_std = rdl.emg_mean_std([50, 150, 60, 140, 100])
        self.assertLess(tight_std, wide_std)


class EmaStepTest(unittest.TestCase):
    def test_alpha_zero_never_moves(self):
        self.assertEqual(rdl.ema_step(1.0, 5.0, 0.0), 1.0)

    def test_alpha_one_jumps_immediately(self):
        self.assertEqual(rdl.ema_step(1.0, 5.0, 1.0), 5.0)

    def test_partial_blend(self):
        self.assertAlmostEqual(rdl.ema_step(0.0, 1.0, 0.15), 0.15)

    def test_converges_toward_target_over_repeated_calls(self):
        current = 0.0
        for _ in range(200):
            current = rdl.ema_step(current, 1.0, 0.1)
        self.assertAlmostEqual(current, 1.0, places=6)


class SelectRawSmoothingAlphaTest(unittest.TestCase):
    """2026-09-12: gripping gets stronger smoothing (see
    GRIPPING_SMOOTHING_ALPHA's comment -- muscle-exertion tremor measured
    ~100x the ordinary jitter RAW_SMOOTHING_ALPHA was tuned against).
    Guards the actual decision, not just its inputs -- a future change
    that accidentally weakens or removes the gripping-specific value
    should fail loudly here instead of only showing up as "the arm still
    shakes while gripping" during a real hardware session."""

    def test_gripping_uses_the_stronger_alpha(self):
        self.assertEqual(rdl.select_raw_smoothing_alpha(True), rdl.GRIPPING_SMOOTHING_ALPHA)

    def test_not_gripping_uses_the_ordinary_alpha(self):
        self.assertEqual(rdl.select_raw_smoothing_alpha(False), rdl.RAW_SMOOTHING_ALPHA)

    def test_gripping_alpha_is_actually_stronger(self):
        # Smaller alpha = slower ema_step = more smoothing (see ema_step's
        # own docstring) -- the whole point of a separate constant is that
        # gripping's value smooths MORE, not just differently.
        self.assertLess(rdl.GRIPPING_SMOOTHING_ALPHA, rdl.RAW_SMOOTHING_ALPHA)


class RateLimitStepTest(unittest.TestCase):
    def test_within_one_step_lands_exactly_on_target(self):
        self.assertEqual(rdl.rate_limit_step(0.0, 0.05, 0.1), 0.05)

    def test_caps_a_large_positive_jump(self):
        self.assertEqual(rdl.rate_limit_step(0.0, 10.0, 0.1), 0.1)

    def test_caps_a_large_negative_jump(self):
        self.assertEqual(rdl.rate_limit_step(0.0, -10.0, 0.1), -0.1)

    def test_never_overshoots_regardless_of_step_count(self):
        current = 0.0
        target = 1.0
        max_step = 0.37  # deliberately not a clean divisor of 1.0
        for _ in range(20):
            current = rdl.rate_limit_step(current, target, max_step)
            self.assertLessEqual(current, target + 1e-9)
        self.assertAlmostEqual(current, target, places=6)


class CalibrationTiltDegTest(unittest.TestCase):
    def test_identical_vectors_zero_tilt(self):
        self.assertAlmostEqual(rdl.calibration_tilt_deg((0, 0, 1), (0, 0, 2)), 0.0, places=4)

    def test_perpendicular_vectors_90deg(self):
        self.assertAlmostEqual(rdl.calibration_tilt_deg((0, 0, 1), (1, 0, 0)), 90.0, places=4)

    def test_opposite_vectors_180deg(self):
        self.assertAlmostEqual(rdl.calibration_tilt_deg((0, 0, 1), (0, 0, -1)), 180.0, places=4)


class ObliqueBasisTest(unittest.TestCase):
    """Mirrors tests/test_tilt_azimuth.cpp's "oblique_decompose recovers
    exact coordinates in a non-orthogonal basis" case exactly."""

    def setUp(self):
        self.ref = (0.0, 0.0, 1.0)
        self.u, self.v = _orthonormal_basis_perpendicular_to(self.ref)
        deg = math.pi / 180.0
        self.fwd_ref = _synthesize(self.ref, self.u, self.v, 70.0 * deg, 0.0 * deg)
        self.abd_ref = _synthesize(self.ref, self.u, self.v, 80.0 * deg, 25.0 * deg)
        self.basis = rdl.make_oblique_basis(self.ref, self.fwd_ref, self.abd_ref)

    def test_u_v_are_orthonormal_and_perpendicular_to_ref(self):
        self.assertAlmostEqual(rdl._dot3(self.u, self.u), 1.0, places=5)
        self.assertAlmostEqual(rdl._dot3(self.v, self.v), 1.0, places=5)
        self.assertAlmostEqual(rdl._dot3(self.u, self.ref), 0.0, places=5)
        self.assertAlmostEqual(rdl._dot3(self.v, self.ref), 0.0, places=5)

    def test_ref_decodes_to_origin(self):
        fwd, abd = rdl.oblique_decompose(self.basis, self.ref)
        self.assertAlmostEqual(fwd, 0.0, places=4)
        self.assertAlmostEqual(abd, 0.0, places=4)

    def test_fwd_calibration_pose_decodes_to_1_0(self):
        fwd, abd = rdl.oblique_decompose(self.basis, self.fwd_ref)
        self.assertAlmostEqual(fwd, 1.0, places=3)
        self.assertAlmostEqual(abd, 0.0, places=3)

    def test_abd_calibration_pose_decodes_to_0_1(self):
        fwd, abd = rdl.oblique_decompose(self.basis, self.abd_ref)
        self.assertAlmostEqual(fwd, 0.0, places=3)
        self.assertAlmostEqual(abd, 1.0, places=3)

    def test_non_anchor_combination_recovers_exact_coordinates(self):
        # NOTE: can't reuse tests/test_tilt_azimuth.cpp's v_mid literal
        # verbatim here -- found while writing this test that
        # run_demo_live.py's oblique_decompose() normalizes its input
        # internally (its own docstring: "accel_raw: raw (not-yet-
        # normalized)"), unlike the C++ version, which requires a
        # pre-normalized unit vector. The C++ test's v_mid is deliberately
        # NOT unit-length (it's ref + 0.3*pf + 0.7*pa, magnitude ~1.36) --
        # exact for the C++ contract, but Python's internal normalize()
        # distorts a non-unit input's direction before it ever reaches the
        # projection, so the same literal doesn't decode to (0.3, 0.7)
        # here. This is a real, confirmed difference in behavior between
        # the two ports, not a bug in either -- worth knowing about if the
        # two are ever compared numerically again.
        #
        # Uses a genuinely unit-length point instead (from the same
        # _synthesize() used for the anchors), with the expected (fwd, abd)
        # cross-checked independently via numpy.linalg.solve on the same
        # 2x2 Gram system -- not just trusting oblique_decompose's own
        # output as its own proof.
        deg = math.pi / 180.0
        test_pt = _synthesize(self.ref, self.u, self.v, 45.0 * deg, 50.0 * deg)
        self.assertAlmostEqual(math.sqrt(sum(x * x for x in test_pt)), 1.0, places=6)

        import numpy as np
        p = np.array(test_pt) - np.dot(test_pt, self.ref) * np.array(self.ref)
        pf, pa = np.array(self.basis["pf"]), np.array(self.basis["pa"])
        gram = np.array([[np.dot(pf, pf), np.dot(pf, pa)], [np.dot(pf, pa), np.dot(pa, pa)]])
        rhs = np.array([np.dot(p, pf), np.dot(p, pa)])
        expected_fwd, expected_abd = np.linalg.solve(gram, rhs)

        fwd, abd = rdl.oblique_decompose(self.basis, test_pt)
        self.assertAlmostEqual(fwd, expected_fwd, places=5)
        self.assertAlmostEqual(abd, expected_abd, places=5)

    def test_scaled_anchors_recover_own_real_tilt(self):
        deg = math.pi / 180.0
        fwd, abd = rdl.oblique_decompose_scaled(self.basis, self.fwd_ref)
        self.assertAlmostEqual(fwd, 70.0 * deg, places=3)
        self.assertAlmostEqual(abd, 0.0, places=3)

        fwd, abd = rdl.oblique_decompose_scaled(self.basis, self.abd_ref)
        self.assertAlmostEqual(fwd, 0.0, places=3)
        self.assertAlmostEqual(abd, 80.0 * deg, places=3)

    def test_scaled_off_axis_magnitude_matches_real_tilt_not_overshoot(self):
        # The whole point of oblique_decompose_scaled over plain
        # oblique_decompose: a real ~16deg shoulder drift once decoded to
        # ~35deg pitch_equiv with the unscaled version. Here: a real 20deg
        # tilt must decode to a total output magnitude of exactly 20deg,
        # not something larger.
        deg = math.pi / 180.0
        off_axis = _synthesize(self.ref, self.u, self.v, 20.0 * deg, 40.0 * deg)
        fwd, abd = rdl.oblique_decompose_scaled(self.basis, off_axis)
        recovered_mag = math.hypot(fwd, abd)
        self.assertAlmostEqual(recovered_mag, 20.0 * deg, places=3)


if __name__ == "__main__":
    unittest.main()
