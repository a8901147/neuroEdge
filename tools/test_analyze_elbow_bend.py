"""analyze_elbow_bend.py and the geometry behind the firmware's elbow bend (the angle between the two raw gravity
vectors): exact when the sensor frames coincide with the arm straight and the elbow's axis is horizontal; blind to a
bend about a vertical axis; drifting with pose when a sensor is mounted rotated."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analyze_elbow_bend as a  # noqa: E402

GRAVITY = (0.0, 0.0, -1.0)   # world frame: x lateral, y forward, z up


def rot(axis, deg):
    """Rotation matrix about a unit axis (Rodrigues)."""
    x, y, z = axis
    t = math.radians(deg)
    c, s, C = math.cos(t), math.sin(t), 1 - math.cos(t)
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]


def mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def sensed(frame):
    """Gravity as an accelerometer with orientation `frame` (sensor -> world) reads it: frame^T g."""
    return tuple(sum(frame[r][c] * GRAVITY[r] for r in range(3)) for c in range(3))


def arm(raise_deg, bend_deg, raise_axis=(1, 0, 0), elbow_axis=(1, 0, 0), forearm_mount=None):
    """(upper-arm reading, forearm reading) for an arm raised by raise_deg about raise_axis and bent by bend_deg about
    elbow_axis (in the upper arm's frame). forearm_mount: an extra rotation of the forearm sensor on its segment."""
    upper = rot(raise_axis, raise_deg)
    fore = mul(upper, rot(elbow_axis, bend_deg))
    if forearm_mount is not None:
        fore = mul(fore, forearm_mount)
    return sensed(upper), sensed(fore)


class GeometryTest(unittest.TestCase):
    def test_aligned_sensors_and_a_horizontal_elbow_axis_read_the_true_bend_at_any_raise(self):
        for raise_deg in (0, 30, 60, 90):
            for bend in (0, 45, 90, 130):
                self.assertAlmostEqual(a.angle_deg(*arm(raise_deg, bend)), bend, places=6)

    def test_a_bend_about_a_vertical_axis_reads_zero(self):
        # after a 90-degree raise about x, the upper arm frame's y axis points straight up: a bend about it is a bend
        # about the vertical
        upper_y_is_vertical = (0, 1, 0)
        for bend in (30, 60, 90):
            self.assertAlmostEqual(a.angle_deg(*arm(90, bend, elbow_axis=upper_y_is_vertical)), 0.0, places=6)

    def test_a_rotated_forearm_mount_makes_a_straight_arm_drift_with_tilt(self):
        mount = rot((0, 0, 1), 20)   # forearm sensor turned 20 degrees about the forearm's long axis
        hanging = a.angle_deg(*arm(0, 0, forearm_mount=mount))
        raised = a.angle_deg(*arm(90, 0, forearm_mount=mount))
        self.assertAlmostEqual(hanging, 0.0, places=6)   # hanging: the long axis is vertical, the turn is invisible
        self.assertGreater(raised, 15.0)                  # raised: the same straight arm reads a bend


class AnalysisTest(unittest.TestCase):
    def test_bend_by_tilt_bins_by_tilt_from_the_first_sample(self):
        samples = [dict(zip(("upper_arm", "forearm"), arm(r, 20))) for r in range(0, 91) for _ in range(5)]
        bands = a.bend_by_tilt(samples)
        self.assertEqual(sorted(bands), [0, 15, 30, 45, 60, 75, 90])
        for v in bands.values():
            self.assertAlmostEqual(v, 20.0, places=6)

    def test_bands_with_too_few_samples_are_left_out(self):
        samples = [dict(zip(("upper_arm", "forearm"), arm(r, 0))) for r in (0, 0, 0, 0, 0, 40)]
        self.assertEqual(list(a.bend_by_tilt(samples)), [0])

    def test_the_committed_recording_reports_every_raise_phase(self):
        path = a.DATA_DIR / "arm_motion_20261004-021240.json"
        lines = a.report(path)
        self.assertEqual([ln.split()[0] for ln in lines[1:]], ["raise_forward", "raise_left_front", "raise_right_front"])
        for ln in lines[1:]:
            self.assertIn("change", ln)


if __name__ == "__main__":
    unittest.main()
