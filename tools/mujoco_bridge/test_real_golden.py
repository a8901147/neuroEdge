"""include/edgeneuro/control/mearm_angle_data.hpp (the measured link-angle lines for the firmware) and data/real_golden.csv
(sensors -> real pulses through mearm_real, its default mode) are generated from Python, the source of truth; the C++ port in
include/edgeneuro/control/mearm_real.hpp is checked against the table by tests/test_mearm_real.cpp. Stale -> fail."""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gen_real_golden as gen  # noqa: E402

ROOT = HERE.parent.parent


class RealGoldenTest(unittest.TestCase):
    def test_the_committed_files_match_a_fresh_regeneration(self):
        header, golden = gen.render()
        self.assertEqual((ROOT / "include/edgeneuro/control/mearm_angle_data.hpp").read_text(), header,
                         "stale: run tools/mujoco_bridge/gen_real_golden.py")
        self.assertEqual((ROOT / "data/real_golden.csv").read_text(), golden,
                         "stale: run tools/mujoco_bridge/gen_real_golden.py")

    def test_the_table_is_big_and_covers_the_whole_safe_shoulder_range(self):
        rows = [l.split(",") for l in (ROOT / "data/real_golden.csv").read_text().splitlines()[1:]]
        self.assertGreaterEqual(len(rows), 400)
        shoulders = {int(r[4]) for r in rows}
        env_shoulders = gen.real.load_arm().table["shoulders"]
        self.assertIn(min(env_shoulders), shoulders)          # hanging -> the lowest safe shoulder
        self.assertIn(max(env_shoulders), shoulders)          # raised -> the highest measured safe shoulder

    def test_the_generator_writes_exactly_what_it_renders(self):
        import tempfile
        d = Path(tempfile.mkdtemp())
        gen.write(d / "h.hpp", d / "g.csv")
        header, golden = gen.render()
        self.assertEqual((d / "h.hpp").read_text(), header)
        self.assertEqual((d / "g.csv").read_text(), golden)


if __name__ == "__main__":
    unittest.main()
