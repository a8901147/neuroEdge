"""Model command <-> real servo pulse. Physical facts this relies on (SESSION_LOG 2026-09-27): a real MEArm's shoulder and
elbow servos both sit at the base and drive their links through parallel linkages, so the UPPER ARM's absolute elevation is
a function of the shoulder pulse alone and the FOREARM's absolute elevation of the elbow pulse alone. The MuJoCo model
uses relative joints instead (forearm = shoulder + elbow), so the two are related by plain geometry, which is checked here
against the simulator itself rather than trusted."""
import math
import sys
import unittest
from pathlib import Path

import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mearm_pulse_map as pm  # noqa: E402

SCENE = Path(__file__).resolve().parent / "mearm_scene.xml"


def simulator_elevations(shoulder, elbow):
    """Upper-arm and forearm elevation (deg above horizontal, in the arm's vertical plane) read from the MuJoCo model."""
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    data.qpos[model.joint("shoulder").qposadr[0]] = shoulder
    data.qpos[model.joint("elbow").qposadr[0]] = elbow
    data.qpos[model.joint("tool").qposadr[0]] = math.pi / 2 - shoulder - elbow      # the scene's tendon/equality
    mujoco.mj_forward(model, data)

    def elevation(body):
        z = data.xmat[model.body(body).id].reshape(3, 3)[:, 2]       # the link's own axis (toward its far end)
        return math.degrees(math.atan2(z[2], z[0]))

    return elevation("upper_arm_link"), elevation("forearm_link"), elevation("tool_link")


class KinematicsAgreeWithTheSimulatorTest(unittest.TestCase):
    GRID = [(s, e) for s in (-0.14, 0.0, 0.3, 0.6, 0.898) for e in (0.995, 1.4, 1.9, 2.617)]

    def test_upper_arm_elevation(self):
        for s, e in self.GRID:
            with self.subTest(s=s, e=e):
                self.assertAlmostEqual(pm.upper_arm_elevation_deg(s), simulator_elevations(s, e)[0], places=6)

    def test_forearm_elevation(self):
        for s, e in self.GRID:
            with self.subTest(s=s, e=e):
                self.assertAlmostEqual(pm.forearm_elevation_deg(s, e), simulator_elevations(s, e)[1], places=6)

    def test_the_tool_link_stays_level_whatever_the_shoulder_and_elbow(self):
        # the claw-levelling parallelogram: why the model's tool joint is locked to pi/2 - shoulder - elbow
        for s, e in self.GRID:
            self.assertAlmostEqual(simulator_elevations(s, e)[2], 0.0, places=6)

    def test_the_inverse_recovers_the_model_command(self):
        for s, e in self.GRID:
            eu, ef = pm.upper_arm_elevation_deg(s), pm.forearm_elevation_deg(s, e)
            s2, e2 = pm.model_ctrl_from_elevations(eu, ef)
            self.assertAlmostEqual(s2, s, places=9)
            self.assertAlmostEqual(e2, e, places=9)


class FitTest(unittest.TestCase):
    def test_a_line_is_recovered_including_a_negative_slope(self):
        pts = [(p, 40.0 - 0.09 * (p - 1500)) for p in (1500, 1575, 1650, 1800)]
        f = pm.fit_angle_vs_pulse(pts)
        self.assertAlmostEqual(f["slope"], -0.09, places=9)
        self.assertAlmostEqual(f["at_1500"], 40.0, places=9)
        self.assertLess(f["max_residual"], 1e-9)

    def test_noise_shows_up_as_a_residual(self):
        pts = [(1500, 40.0), (1575, 47.0), (1650, 47.0), (1800, 70.0)]
        self.assertGreater(pm.fit_angle_vs_pulse(pts)["max_residual"], 3.0)

    def test_too_few_points_or_all_the_same_pulse_is_refused(self):
        with self.assertRaises(ValueError):
            pm.fit_angle_vs_pulse([(1500, 40.0), (1600, 45.0)])
        with self.assertRaises(ValueError):
            pm.fit_angle_vs_pulse([(1500, 40.0), (1500, 41.0), (1500, 42.0)])

    def test_non_finite_readings_are_refused(self):
        with self.assertRaises(ValueError):
            pm.fit_angle_vs_pulse([(1500, 40.0), (1600, float("nan")), (1700, 50.0)])


class ServoAngleMapTest(unittest.TestCase):
    def make(self, ks=-0.09, ke=0.10):
        return pm.ServoAngleMap(shoulder={"slope": ks, "at_1500": 55.0}, elbow={"slope": ke, "at_1500": -35.0})

    def test_pulses_to_link_elevations(self):
        m = self.make()
        self.assertAlmostEqual(m.upper_arm_elevation_deg(1500), 55.0)
        self.assertAlmostEqual(m.upper_arm_elevation_deg(1600), 55.0 - 9.0)
        self.assertAlmostEqual(m.forearm_elevation_deg(1500), -35.0)
        self.assertAlmostEqual(m.forearm_elevation_deg(1600), -35.0 + 10.0)

    def test_model_ctrl_from_pulses_follows_the_geometry(self):
        m = self.make()
        s, e = m.model_ctrl_from_pulses(1500, 1500)
        self.assertAlmostEqual(s, math.radians(90 - 55.0))
        self.assertAlmostEqual(e, math.radians(55.0 - (-35.0)))          # relative elbow = upper - forearm elevation

    def test_pulses_from_model_ctrl_round_trips_for_either_sign_of_slope(self):
        for ks, ke in ((-0.09, 0.10), (0.09, -0.10), (0.09, 0.10), (-0.09, -0.10)):
            m = self.make(ks, ke)
            for ps, pe in ((1500, 1500), (1575, 900), (1800, 1600), (1650, 1850)):
                s, e = m.model_ctrl_from_pulses(ps, pe)
                p2s, p2e = m.pulses_from_model_ctrl(s, e)
                self.assertAlmostEqual(p2s, ps, places=6)
                self.assertAlmostEqual(p2e, pe, places=6)

    def test_a_flat_fit_cannot_be_inverted(self):
        with self.assertRaises(ValueError):
            pm.ServoAngleMap(shoulder={"slope": 0.0, "at_1500": 55.0}, elbow={"slope": 0.1, "at_1500": -35.0})

    def test_the_slope_of_a_hobby_servo_is_checked_for_plausibility(self):
        # SG92R: about 180 deg over ~2000us = 0.09 deg/us; the parallel linkage can scale it, but 10x off means a units mistake
        self.assertTrue(pm.slope_is_plausible(0.09))
        self.assertTrue(pm.slope_is_plausible(-0.12))
        self.assertFalse(pm.slope_is_plausible(0.9))
        self.assertFalse(pm.slope_is_plausible(0.005))


class ReachTest(unittest.TestCase):
    def test_the_reachable_model_range_from_a_pulse_range(self):
        m = pm.ServoAngleMap(shoulder={"slope": -0.09, "at_1500": 55.0}, elbow={"slope": 0.10, "at_1500": -35.0})
        r = m.reachable_shoulder_ctrl((1500, 1800))
        # pulse 1500 -> elevation 55 -> ctrl rad(35);  pulse 1800 -> elevation 28 -> ctrl rad(62)
        self.assertAlmostEqual(r[0], math.radians(35.0))
        self.assertAlmostEqual(r[1], math.radians(62.0))

    def test_the_share_of_the_models_shoulder_travel_a_pulse_range_covers(self):
        m = pm.ServoAngleMap(shoulder={"slope": -0.09, "at_1500": 55.0}, elbow={"slope": 0.10, "at_1500": -35.0})
        share = m.shoulder_travel_share((1500, 1800), model_range=(-0.141261412, 0.898057932))
        self.assertAlmostEqual(share, math.radians(27.0) / (0.898057932 + 0.141261412), places=6)


if __name__ == "__main__":
    unittest.main()
