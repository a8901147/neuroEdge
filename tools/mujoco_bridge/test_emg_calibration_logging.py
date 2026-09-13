"""Tests for the EMG calibration long-term observation log added
2026-09-19 (see run_demo_live.py's EMG_CALIBRATION_LOG_DIR comment): the
user isn't confident mean+K*std is the right algorithm and wants real
relax/contract data from ordinary use over the coming week(s) before
revisiting the design. Covers log_emg_calibration() (the writer, called
from a real calibration session) and summarize_emg_calibration_logs.py
(the reader, for reviewing a week of entries without opening each by
hand) against a temp directory -- never the real
tools/mujoco_bridge/emg_calibration_logs/ (gitignored, real captured
data, not something a test run should create or depend on).

Usage:
    python3 -m unittest tools/mujoco_bridge/test_emg_calibration_logging.py -v
    python3 tools/mujoco_bridge/test_emg_calibration_logging.py
"""

import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_demo_live as rdl  # noqa: E402
import summarize_emg_calibration_logs as summarize  # noqa: E402


class LogEmgCalibrationTest(unittest.TestCase):
    def setUp(self):
        self.tmp_dir = Path(tempfile.mkdtemp())
        self._orig_log_dir = rdl.EMG_CALIBRATION_LOG_DIR
        rdl.EMG_CALIBRATION_LOG_DIR = self.tmp_dir / "emg_calibration_logs"

    def tearDown(self):
        rdl.EMG_CALIBRATION_LOG_DIR = self._orig_log_dir
        shutil.rmtree(self.tmp_dir, ignore_errors=True)

    def _log(self, suspect):
        rdl.log_emg_calibration(
            relaxed_tail=[(0.0, 2570, 2580), (0.1, 2575, 2585)],
            contracted_tail=[(0.0, 2800, 2900)],
            relaxed_mean=2578.8, relaxed_std=5.0, threshold=2679,
            contracted_mean=2934, contracted_std=100.0, suspect=suspect,
        )

    def test_creates_the_log_directory_on_first_call(self):
        self.assertFalse(rdl.EMG_CALIBRATION_LOG_DIR.exists())
        self._log(suspect=False)
        self.assertTrue(rdl.EMG_CALIBRATION_LOG_DIR.exists())

    def test_ok_session_filename_has_no_suspect_tag(self):
        self._log(suspect=False)
        names = [p.name for p in rdl.EMG_CALIBRATION_LOG_DIR.glob("*.json")]
        self.assertEqual(len(names), 1)
        self.assertTrue(names[0].endswith("_ok.json"), names[0])
        self.assertNotIn("SUSPECT", names[0])

    def test_suspect_session_filename_is_tagged_without_opening_the_file(self):
        # The whole point (per the user's own request) is telling suspect
        # sessions apart from a directory listing alone.
        self._log(suspect=True)
        names = [p.name for p in rdl.EMG_CALIBRATION_LOG_DIR.glob("*.json")]
        self.assertEqual(len(names), 1)
        self.assertTrue(names[0].endswith("_SUSPECT.json"), names[0])

    def test_raw_tails_round_trip_exactly(self):
        # The raw samples, not just summary stats, are the actual point --
        # a future re-analysis with a different algorithm needs the real
        # data, not just what this session's algorithm computed from it.
        relaxed = [(0.0, 2570, 2580), (0.1, 2575, 2585)]
        contracted = [(0.0, 2800, 2900)]
        rdl.log_emg_calibration(
            relaxed_tail=relaxed, contracted_tail=contracted,
            relaxed_mean=2578.8, relaxed_std=5.0, threshold=2679,
            contracted_mean=2934, contracted_std=100.0, suspect=False,
        )
        path = next(rdl.EMG_CALIBRATION_LOG_DIR.glob("*.json"))
        with open(path) as f:
            data = json.load(f)
        self.assertEqual([tuple(x) for x in data["relaxed_tail_raw"]], relaxed)
        self.assertEqual([tuple(x) for x in data["contracted_tail_raw"]], contracted)

    def test_records_the_current_emg_threshold_k(self):
        # So a week of entries spanning K's own tuning history (2.0 -> 20
        # -> 15 -> 20 in one session already) can be told apart by which
        # value actually produced each entry.
        self._log(suspect=False)
        path = next(rdl.EMG_CALIBRATION_LOG_DIR.glob("*.json"))
        with open(path) as f:
            data = json.load(f)
        self.assertEqual(data["emg_threshold_k"], rdl.EMG_THRESHOLD_K)

    def test_write_failure_does_not_raise(self):
        # Best-effort: a real calibration session finishing normally
        # matters more than this observational side-log succeeding.
        rdl.EMG_CALIBRATION_LOG_DIR = self.tmp_dir / "not" / "a" / "writable\0path"
        try:
            self._log(suspect=False)  # must not raise
        except Exception as e:  # pragma: no cover - failure path only
            self.fail(f"log_emg_calibration raised instead of degrading gracefully: {e}")


class SummarizeEmgCalibrationLogsTest(unittest.TestCase):
    def setUp(self):
        self.tmp_dir = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.tmp_dir, ignore_errors=True)

    def _write(self, name, **fields):
        with open(self.tmp_dir / name, "w") as f:
            json.dump(fields, f)

    def test_empty_directory_returns_no_entries(self):
        self.assertEqual(summarize.load_entries(self.tmp_dir), [])

    def test_loads_all_valid_entries_sorted_by_filename(self):
        self._write("2026-09-14_100000_ok.json", timestamp="2026-09-14_100000", suspect=False,
                    threshold=2600, contracted_mean=2900, emg_threshold_k=20.0, git_commit="aaa")
        self._write("2026-09-13_100000_ok.json", timestamp="2026-09-13_100000", suspect=False,
                    threshold=2600, contracted_mean=2900, emg_threshold_k=20.0, git_commit="bbb")
        entries = summarize.load_entries(self.tmp_dir)
        self.assertEqual([d["timestamp"] for _, d in entries],
                          ["2026-09-13_100000", "2026-09-14_100000"])

    def test_corrupt_file_is_skipped_not_fatal(self):
        self._write("2026-09-13_100000_ok.json", timestamp="2026-09-13_100000", suspect=False,
                    threshold=2600, contracted_mean=2900, emg_threshold_k=20.0, git_commit="aaa")
        (self.tmp_dir / "2026-09-13_110000_ok.json").write_text("{not valid json")
        entries = summarize.load_entries(self.tmp_dir)
        self.assertEqual(len(entries), 1)

    def test_main_on_a_missing_directory_does_not_raise(self):
        # First real use of this script will be against a log dir that
        # doesn't exist yet (calibrate_emg_threshold() creates it lazily,
        # on the first real session) -- main() itself must handle that,
        # not just load_entries().
        missing = self.tmp_dir / "does_not_exist"
        old_argv = sys.argv
        sys.argv = ["summarize_emg_calibration_logs.py", "--log-dir", str(missing)]
        try:
            summarize.main()  # must not raise
        finally:
            sys.argv = old_argv

    def test_main_with_suspect_only_filters_correctly(self):
        self._write("2026-09-13_100000_ok.json", timestamp="2026-09-13_100000", suspect=False,
                    threshold=2600, contracted_mean=2900, emg_threshold_k=20.0, git_commit="aaa")
        self._write("2026-09-13_110000_SUSPECT.json", timestamp="2026-09-13_110000", suspect=True,
                    threshold=2600, contracted_mean=2500, emg_threshold_k=20.0, git_commit="aaa")
        entries = summarize.load_entries(self.tmp_dir)
        suspect_entries = [(p, d) for (p, d) in entries if d.get("suspect")]
        self.assertEqual(len(entries), 2)
        self.assertEqual(len(suspect_entries), 1)
        self.assertEqual(suspect_entries[0][1]["timestamp"], "2026-09-13_110000")


if __name__ == "__main__":
    unittest.main()
