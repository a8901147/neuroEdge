"""The real-arm safe envelope: built from the raw measurement files, and its clamp -- the Python reference the C++
PulseEnvelope is checked against. Properties, plus the raw data as the arbiter: the envelope must never allow a
(shoulder, elbow) pulse at which the person pressed Enter."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen_mearm_envelope as g  # noqa: E402
import measure_linkage_region as m  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
SYNTH = {"shoulders": [1500, 1575, 1650, 1800], "lo": [500, 500, 500, 500], "hi": [1500, 1850, 1850, 1600]}


class ClampReferenceTest(unittest.TestCase):
    def test_a_point_inside_is_unchanged(self):
        self.assertEqual(g.clamp(SYNTH, 1575, 1000), (1575, 1000))

    def test_the_shoulder_is_kept_inside_the_measured_breakpoints(self):
        for s in (-1e9, 0, 1499, 1801, 2100, 1e9, math.inf, -math.inf):
            out = g.clamp(SYNTH, s, 1000)
            self.assertTrue(1500 <= out[0] <= 1800, (s, out))

    def test_at_a_breakpoint_its_own_window_applies(self):
        self.assertEqual(g.clamp(SYNTH, 1500, 1800)[1], 1500)
        self.assertEqual(g.clamp(SYNTH, 1575, 1900)[1], 1850)
        self.assertEqual(g.clamp(SYNTH, 1800, 1900)[1], 1600)

    def test_between_breakpoints_the_more_conservative_neighbour_wins_no_interpolation(self):
        self.assertEqual(g.clamp(SYNTH, 1525, 1900)[1], 1500)      # between 1500 (<=1500) and 1575 (<=1850)
        self.assertEqual(g.clamp(SYNTH, 1725, 1900)[1], 1600)      # between 1650 (<=1850) and 1800 (<=1600)
        self.assertEqual(g.clamp(SYNTH, 1600, 1900)[1], 1850)      # between 1575 and 1650: both 1850

    def test_the_elbow_is_kept_above_the_lower_edge_too(self):
        self.assertEqual(g.clamp(SYNTH, 1575, 0)[1], 500)

    def test_idempotent(self):
        for s in range(1300, 2000, 37):
            for e in range(300, 2100, 61):
                once = g.clamp(SYNTH, s, e)
                self.assertEqual(g.clamp(SYNTH, *once), once)

    def test_non_finite_input_goes_to_the_rest_pose(self):
        for s, e in ((math.nan, 1000), (1600, math.nan), (math.nan, math.nan)):
            self.assertEqual(g.clamp(SYNTH, s, e), (1500, 1500))

    def test_infinite_elbow_clamps_to_the_window(self):
        self.assertEqual(g.clamp(SYNTH, 1575, math.inf)[1], 1850)
        self.assertEqual(g.clamp(SYNTH, 1575, -math.inf)[1], 500)

    def test_a_result_is_always_a_whole_microsecond(self):
        out = g.clamp(SYNTH, 1612.4, 1000.6)
        self.assertEqual(out, (1612, 1001))
        self.assertTrue(all(isinstance(v, int) for v in out))
        self.assertEqual(g.clamp(SYNTH, 1612.6, 1000.4), (1613, 1000))       # rounds to nearest, not down

    def test_between_breakpoints_the_higher_lower_edge_wins_as_well(self):
        table = {"shoulders": [1500, 1600, 1700], "lo": [600, 500, 700], "hi": [1500, 1800, 1700]}
        self.assertEqual(g.clamp(table, 1550, 0)[1], 600)
        self.assertEqual(g.clamp(table, 1650, 0)[1], 700)
        self.assertEqual(g.clamp(table, 1550, 5000)[1], 1500)
        self.assertEqual(g.clamp(table, 1650, 5000)[1], 1700)


class BuildTableRefusesBrokenEvidenceTest(unittest.TestCase):
    def rec(self, s, up=None, down=None, self_stop=None):
        return {"shoulder": s, "shoulder_bind_at_elbow_1500": self_stop, "shoulder_bind_cause": None,
                "elbow_up_bind": up, "elbow_down_bind": down, "elbow_up_limit": 1850, "elbow_down_limit": 500,
                "elbow_up_cause": None, "elbow_down_cause": None}

    def test_a_window_that_closes_up_is_refused_not_written(self):
        # stops so close together that, with the safety margin, nothing is left: 550-75=475 < 450+75=525
        with self.assertRaises(ValueError):
            g.build_table([self.rec(1500, up=550, down=450)])

    def test_no_usable_shoulder_position_is_refused(self):
        with self.assertRaises(ValueError):
            g.build_table([self.rec(1350, self_stop=1350)])
        with self.assertRaises(ValueError):
            g.build_table([])

    def test_a_normal_record_builds(self):
        t = g.build_table([self.rec(1500, up=1600, down=None)])
        self.assertEqual((t["shoulders"], t["lo"], t["hi"]), ([1500], [500], [1525]))


class BuildFromRawDataTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.records = m.load_records([ROOT / "data" / n for n in g.SOURCES])
        cls.table = g.build_table(cls.records)

    def test_the_source_files_are_the_labelled_runs_only(self):
        # run 1 (flawed tool) and run 2 (no cause labels) are not evidence for this table; the 2026-09-27 runs 3-5 and
        # the 2026-09-28 runs above shoulder 1800 (elbow held at 1200, then at 700 for 2025/2100) are
        self.assertEqual(len(g.SOURCES), 5)
        self.assertNotIn("mearm_linkage_20260927-114926.json", g.SOURCES)            # run 2, no cause labels
        for name in g.SOURCES:
            self.assertTrue((ROOT / "data" / name).exists(), name)
            self.assertNotIn("run1", name)
            self.assertNotIn("measurements", name)

    def test_breakpoints_are_sorted_and_windows_are_non_empty(self):
        t = self.table
        self.assertEqual(t["shoulders"], sorted(set(t["shoulders"])))
        self.assertEqual(len(t["shoulders"]), len(t["lo"]))
        self.assertEqual(len(t["shoulders"]), len(t["hi"]))
        self.assertTrue(all(lo < hi for lo, hi in zip(t["lo"], t["hi"])))

    def test_no_neighbouring_pair_of_windows_is_disjoint(self):
        # otherwise the conservative intersection between them would be empty
        t = self.table
        for i in range(len(t["shoulders"]) - 1):
            self.assertLess(max(t["lo"][i], t["lo"][i + 1]), min(t["hi"][i], t["hi"][i + 1]))

    def test_the_shoulder_range_is_exactly_the_measured_breakpoints(self):
        self.assertEqual(g.shoulder_range(self.table), (self.table["shoulders"][0], self.table["shoulders"][-1]))

    def test_the_envelope_never_allows_a_pulse_where_the_person_pressed_enter(self):
        checked = 0
        for r in self.records:
            s = r["shoulder"]
            if r["shoulder_bind_at_elbow_1500"] is not None:
                stop = r["shoulder_bind_at_elbow_1500"]
                below = stop < 1500
                out = g.clamp(self.table, stop, 1500)[0]
                self.assertTrue(out > stop if below else out < stop, (r, out))
                checked += 1
                continue
            if r["elbow_up_bind"] is not None:
                self.assertLess(g.clamp(self.table, s, r["elbow_up_bind"])[1], r["elbow_up_bind"], r)
                checked += 1
            if r["elbow_down_bind"] is not None:
                self.assertGreater(g.clamp(self.table, s, r["elbow_down_bind"])[1], r["elbow_down_bind"], r)
                checked += 1
        self.assertGreaterEqual(checked, 5)                   # a guard that the loop really looked at stops (7 in today's data)

    def test_the_table_records_where_its_evidence_stops(self):
        # nothing beyond the highest measured shoulder is claimed to be safe
        self.assertLessEqual(max(self.table["shoulders"]), max(r["shoulder"] for r in self.records))
        self.assertGreaterEqual(min(self.table["shoulders"]), 1500)


class GeneratedFilesTest(unittest.TestCase):
    def test_the_committed_header_matches_a_fresh_regeneration(self):
        self.assertEqual((ROOT / "include/edgeneuro/control/mearm_envelope_data.hpp").read_text(),
                         g.render_header(g.build_table(m.load_records([ROOT / "data" / n for n in g.SOURCES]))),
                         "stale: run tools/gen_mearm_envelope.py")

    def test_the_committed_golden_matches_a_fresh_regeneration(self):
        self.assertEqual((ROOT / "data/envelope_golden.csv").read_text(),
                         g.render_golden(g.build_table(m.load_records([ROOT / "data" / n for n in g.SOURCES]))),
                         "stale: run tools/gen_mearm_envelope.py")

    def test_the_golden_covers_out_of_range_points_on_every_side(self):
        rows = [tuple(map(float, l.split(","))) for l in (ROOT / "data/envelope_golden.csv").read_text().splitlines()[1:]]
        self.assertGreaterEqual(len(rows), 500)
        self.assertTrue(any(r[0] < 1500 for r in rows))
        self.assertTrue(any(r[0] > 1800 for r in rows))
        self.assertTrue(any(r[1] < 500 for r in rows))
        self.assertTrue(any(r[1] > 1850 for r in rows))

    def test_the_generator_writes_exactly_what_it_renders(self):
        import tempfile
        out = Path(tempfile.mkdtemp())
        g.write(out / "h.hpp", out / "g.csv")
        table = g.build_table(m.load_records([ROOT / "data" / n for n in g.SOURCES]))
        self.assertEqual((out / "h.hpp").read_text(), g.render_header(table))
        self.assertEqual((out / "g.csv").read_text(), g.render_golden(table))


if __name__ == "__main__":
    unittest.main()
