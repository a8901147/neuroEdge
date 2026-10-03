"""include/edgeneuro/control/mearm_calibration_data.hpp: the saved sensor calibration compiled into the firmware (the choice
of 2026-09-27: compile-time constants -- re-flash after re-calibrating). Generated from tools/mujoco_bridge/
shoulder_calibration.json, which is gitignored personal data: the staleness check runs only where that file exists."""
import json
import re
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gen_calibration_header as gen  # noqa: E402
import mearm_pathb as pb  # noqa: E402
import test_mearm_direction as fx  # noqa: E402

ROOT = HERE.parent.parent
HEADER = ROOT / "include/edgeneuro/control/mearm_calibration_data.hpp"


def parse(text):
    return {m[1]: float(m[2]) for m in re.finditer(r"constexpr float (k\w+) = ([-\d.eE+]+)f;", text)}


class RenderTest(unittest.TestCase):
    def test_every_value_round_trips_through_the_header(self):
        vals = parse(gen.render(fx.SAVED_9_13))
        for i, axis in enumerate("XYZ"):
            self.assertAlmostEqual(vals[f"kHang{axis}"], fx.SAVED_9_13["baseline_raw"][i], places=8)
            self.assertAlmostEqual(vals[f"kForward{axis}"], fx.SAVED_9_13["forward_raw"][i], places=8)
            self.assertAlmostEqual(vals[f"kLeft{axis}"], fx.SAVED_9_13["left_twist_raw"][i], places=8)
            self.assertAlmostEqual(vals[f"kRight{axis}"], fx.SAVED_9_13["right_twist_raw"][i], places=8)
        self.assertAlmostEqual(vals["kZeroElbow"], fx.SAVED_9_13["zero_elbow"], places=8)
        self.assertEqual(len(vals), 13 + 6)                      # + the base reach vectors (zero when not measured)
        self.assertIn("constexpr bool kHasBaseReach = false;", gen.render(fx.SAVED_9_13))

    def test_a_measured_base_reach_is_compiled_in_too(self):
        # 2026-10-03: the arm's comfortable left/right reach (measure_base_reach.py), raw vectors like the other poses
        left = [0.3, 0.6, 0.75]
        right = [0.25, -0.62, 0.74]
        text = gen.render(dict(fx.SAVED_9_13, base_reach_left_raw=left, base_reach_right_raw=right))
        vals = parse(text)
        self.assertIn("constexpr bool kHasBaseReach = true;", text)
        for i, axis in enumerate("XYZ"):
            self.assertAlmostEqual(vals[f"kBaseReachLeft{axis}"], left[i], places=8)
            self.assertAlmostEqual(vals[f"kBaseReachRight{axis}"], right[i], places=8)

    def test_a_base_reach_path_b_cannot_use_is_refused(self):
        bad = dict(fx.SAVED_9_13, base_reach_left_raw=[0.3, 0.6, 0.75], base_reach_right_raw=[0.3, 0.6, 0.75])
        with self.assertRaises(ValueError):
            gen.render(bad)

    def test_the_header_says_when_the_calibration_was_captured(self):
        saved = dict(fx.SAVED_9_13, captured_at="2026-09-13 16:35:48")
        self.assertIn("2026-09-13 16:35:48", gen.render(saved))

    def test_a_calibration_path_b_would_reject_is_refused_not_written(self):
        bad = dict(fx.SAVED_9_13, forward_raw=fx.SAVED_9_13["baseline_raw"])      # FORWARD == HANG
        with self.assertRaises(ValueError):
            gen.render(bad)
        missing = {k: v for k, v in fx.SAVED_9_13.items() if k != "zero_elbow"}
        with self.assertRaises((KeyError, ValueError)):
            gen.render(missing)

    def test_write_creates_the_file_and_never_touches_the_json(self):
        d = Path(tempfile.mkdtemp())
        src = d / "cal.json"
        src.write_text(json.dumps(fx.SAVED_9_13))
        before = src.read_text()
        gen.write(src, d / "h.hpp")
        self.assertEqual((d / "h.hpp").read_text(), gen.render(fx.SAVED_9_13))
        self.assertEqual(src.read_text(), before)


class CommittedHeaderTest(unittest.TestCase):
    def test_the_committed_header_is_a_calibration_path_b_accepts(self):
        v = parse(HEADER.read_text())
        pb.Calibration([v["kHangX"], v["kHangY"], v["kHangZ"]], [v["kForwardX"], v["kForwardY"], v["kForwardZ"]],
                       [v["kLeftX"], v["kLeftY"], v["kLeftZ"]], [v["kRightX"], v["kRightY"], v["kRightZ"]],
                       v["kZeroElbow"])

    def test_the_committed_header_is_generated_from_the_committed_golden_calibration(self):
        # since 2026-10-03 run_demo_live.py SENDS the calibration over UART; the compiled-in one is only what the board
        # boots with (and the C++ tests' fixture), so it comes from the committed golden file -- not the gitignored local
        # one, which changes with every re-calibration (it did on 2026-10-04 and made this test fail for that reason)
        self.assertEqual(gen.DEFAULT_SOURCE.name, "shoulder_calibration_golden_2026-09-13.json")
        self.assertEqual(HEADER.read_text(), gen.render(json.loads(gen.DEFAULT_SOURCE.read_text())),
                         "stale: re-run tools/mujoco_bridge/gen_calibration_header.py and re-flash")
        self.assertIn("UART", HEADER.read_text())


if __name__ == "__main__":
    unittest.main()
