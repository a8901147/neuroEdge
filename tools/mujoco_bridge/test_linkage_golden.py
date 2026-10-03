"""data/linkage_golden.csv is what tests/test_mearm_linkage.cpp checks the C++
project_elbow port against. It is generated from mearm_pathb.py (source of
truth); this test fails if Python's mapping changed and the table was not
regenerated (run gen_linkage_golden.py), so the C++ side can't silently drift.
"""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gen_linkage_golden as gen  # noqa: E402
import mearm_pathb as pb  # noqa: E402

CSV = HERE.parent.parent / "data" / "linkage_golden.csv"


class LinkageGoldenTest(unittest.TestCase):
    def test_committed_table_matches_a_fresh_regeneration(self):
        self.assertEqual(CSV.read_text(), gen.render(),
                         "stale: run tools/mujoco_bridge/gen_linkage_golden.py")

    def test_table_covers_out_of_range_requests_and_the_window_edges(self):
        rows = [tuple(map(float, l.split(","))) for l in CSV.read_text().splitlines()[1:]]
        self.assertGreaterEqual(len(rows), 100)
        self.assertTrue(any(r[1] < pb.ELBOW_EXTENDED for r in rows))
        self.assertTrue(any(r[1] > pb.ELBOW_FOLDED for r in rows))
        self.assertTrue(any(abs(r[0] - pb.SHOULDER_RAISED) < 1e-9 for r in rows))
        self.assertTrue(any(abs(r[0] - pb.SHOULDER_REST) < 1e-9 for r in rows))

    def test_the_generator_writes_exactly_what_it_renders(self):
        import tempfile
        out = Path(tempfile.mkdtemp()) / "sub" / "linkage_golden.csv"
        gen.write(out)                                   # creates the parent directory too
        self.assertEqual(out.read_text(), gen.render())


if __name__ == "__main__":
    unittest.main()
