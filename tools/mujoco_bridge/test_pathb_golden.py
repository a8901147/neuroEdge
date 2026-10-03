"""data/pathb_golden.csv + pathb_calibration.csv are what tests/test_mearm_pathb.cpp
checks the C++ Path B port against. They are generated from mearm_pathb.py (source
of truth); this fails if Python's mapping (or the embedded 9/13 calibration / 9/24
real log) changed and the tables were not regenerated (gen_pathb_golden.py)."""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gen_pathb_golden as gen  # noqa: E402

DATA = HERE.parent.parent / "data"


class PathBGoldenTest(unittest.TestCase):
    def test_committed_tables_match_a_fresh_regeneration(self):
        self.assertEqual((DATA / "pathb_calibration.csv").read_text(), gen.render_calibration(),
                         "stale: run tools/mujoco_bridge/gen_pathb_golden.py")
        self.assertEqual((DATA / "pathb_golden.csv").read_text(), gen.render_golden(),
                         "stale: run tools/mujoco_bridge/gen_pathb_golden.py")

    def test_the_table_is_big_enough_to_mean_something(self):
        n = len((DATA / "pathb_golden.csv").read_text().splitlines()) - 1
        self.assertGreaterEqual(n, 250)

    def test_the_generator_writes_exactly_what_it_renders(self):
        import tempfile
        out = Path(tempfile.mkdtemp()) / "sub"
        gen.write(out)
        self.assertEqual((out / "pathb_calibration.csv").read_text(), gen.render_calibration())
        self.assertEqual((out / "pathb_golden.csv").read_text(), gen.render_golden())


if __name__ == "__main__":
    unittest.main()
