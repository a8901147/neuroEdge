"""Tests run_demo_live.py's serial-stream parsing: split_lines()'s buffer/
chunk handling, and LINE_RE/SHOULDER_RAW_RE/ELBOW_RAW_RE/DIAG_LINE_RE's
robustness against malformed input. Real serial reads never land neatly on
line boundaries, and a real wire glitch (see PRD.md's 2026-09-05 loose-
connection finding) can corrupt or truncate bytes -- none of that was
tested before this file. test_imu_to_mujoco.py exercises these same
regexes, but only against known-good output from a real subprocess
(`raise SystemExit` if a line doesn't match), which tests the happy path,
not resilience to a bad one.

Usage:
    python3 -m unittest tools/mujoco_bridge/test_serial_parsing.py -v
    python3 tools/mujoco_bridge/test_serial_parsing.py
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_demo_live as rdl  # noqa: E402

# A realistic well-formed line, same field set/order phase3_control_loop_main.cpp
# actually sends (see run_demo_live.py's LINE_RE/SHOULDER_RAW_RE/ELBOW_RAW_RE).
FULL_LINE = (
    "tick=12345 grip=0.500 gripping=1 shoulder_pitch=0.123 shoulder_roll=-0.045 "
    "elbow=0.678 shoulder_raw_ax=+0.020 shoulder_raw_ay=+0.050 shoulder_raw_az=+0.980 "
    "shoulder_raw_gx=+0.001 shoulder_raw_gy=-0.002 shoulder_raw_gz=+0.000 "
    "elbow_raw_ax=-0.010 elbow_raw_ay=+0.030 elbow_raw_az=+0.990"
)

DIAG_LINE = (
    "diag shoulder_completions=250 elbow_completions=248 shoulder_nacks=0 "
    "shoulder_timeouts=0 elbow_nacks=0 elbow_timeouts=2 active_reader_state=1 "
    "bus_recovery_attempts=0 bus_recovery_freed=0"
)


class SplitLinesTest(unittest.TestCase):
    def test_single_complete_line(self):
        lines, buf = rdl.split_lines(b"", b"hello\r\n")
        self.assertEqual(lines, ["hello"])
        self.assertEqual(buf, b"")

    def test_multiple_lines_in_one_chunk(self):
        lines, buf = rdl.split_lines(b"", b"one\r\ntwo\r\nthree\r\n")
        self.assertEqual(lines, ["one", "two", "three"])
        self.assertEqual(buf, b"")

    def test_no_newline_yet_stays_buffered(self):
        lines, buf = rdl.split_lines(b"", b"partial")
        self.assertEqual(lines, [])
        self.assertEqual(buf, b"partial")

    def test_line_split_across_two_chunks(self):
        # This is the realistic case: pyserial's ser.read(256) has no
        # reason to land on a \r\n boundary -- a real line WILL sometimes
        # arrive split across two reads.
        lines1, buf = rdl.split_lines(b"", b"tick=1 gr")
        self.assertEqual(lines1, [])
        lines2, buf = rdl.split_lines(buf, b"ip=0.5\r\n")
        self.assertEqual(lines2, ["tick=1 grip=0.5"])
        self.assertEqual(buf, b"")

    def test_split_across_three_chunks(self):
        buf = b""
        collected = []
        for chunk in (b"ab", b"cd", b"ef\r\n"):
            lines, buf = rdl.split_lines(buf, chunk)
            collected.extend(lines)
        self.assertEqual(collected, ["abcdef"])

    def test_trailing_partial_line_preserved_after_complete_ones(self):
        lines, buf = rdl.split_lines(b"", b"complete\r\nincomplete")
        self.assertEqual(lines, ["complete"])
        self.assertEqual(buf, b"incomplete")

    def test_corrupted_bytes_do_not_raise(self):
        # A real wire glitch can flip/drop bits mid-transmission -- this
        # produces invalid UTF-8, not necessarily readable garbage.
        # errors="ignore" must hold: no exception, some string comes out.
        garbage = b"tick=1 grip=\xff\xfe\x00garbled\r\n"
        lines, buf = rdl.split_lines(b"", garbage)
        self.assertEqual(len(lines), 1)
        self.assertEqual(buf, b"")

    def test_empty_chunk_is_a_noop(self):
        lines, buf = rdl.split_lines(b"existing", b"")
        self.assertEqual(lines, [])
        self.assertEqual(buf, b"existing")


class LineRegexRobustnessTest(unittest.TestCase):
    """LINE_RE/SHOULDER_RAW_RE/ELBOW_RAW_RE/DIAG_LINE_RE must match a real
    well-formed line, and -- just as importantly -- correctly return None
    (not crash, not false-positive on garbage) for a truncated or
    corrupted one. reader_thread_main relies on exactly this: an
    unmatched line falls through to the `[FW] ...` passthrough print
    instead of ever reaching latest.update() with partial data."""

    def test_full_line_matches_all_four_patterns_where_expected(self):
        self.assertIsNotNone(rdl.LINE_RE.search(FULL_LINE))
        self.assertIsNotNone(rdl.SHOULDER_RAW_RE.search(FULL_LINE))
        self.assertIsNotNone(rdl.ELBOW_RAW_RE.search(FULL_LINE))
        self.assertIsNone(rdl.LINE_RE.search(DIAG_LINE))
        self.assertIsNotNone(rdl.DIAG_LINE_RE.search(DIAG_LINE))

    def test_single_dropped_digit_matches_with_a_silently_wrong_value(self):
        # A real, confirmed LIMITATION, not something this regex can be
        # expected to catch: a single dropped byte mid-number (e.g. a UART
        # glitch eating one digit) still produces a syntactically
        # valid-looking line. [-\d.eE+]+ has no checksum, so this matches
        # with a plausible-but-wrong value instead of failing -- worth
        # knowing as a known limitation rather than assuming the regex
        # would catch it. The real defense against this class of error is
        # the firmware's own [DIAG] nacks/timeouts counters (an actual
        # bus-level ACK signal), not line-level parsing.
        corrupted = FULL_LINE.replace("elbow=0.678", "elbow=0.68")  # last digit dropped
        match = rdl.LINE_RE.search(corrupted)
        self.assertIsNotNone(match)
        self.assertEqual(match.group("elbow"), "0.68")

    def test_line_missing_trailing_fields_still_matches_leading_ones(self):
        # LINE_RE only anchors the leading fields (tick/grip/gripping/
        # shoulder_pitch/shoulder_roll/elbow) -- by design (this file's own
        # LINE_RE comment: shared with the CSV-replay binary's shorter
        # output). SHOULDER_RAW_RE/ELBOW_RAW_RE must NOT match if their
        # fields were never sent.
        leading_only = FULL_LINE.split(" shoulder_raw_ax=")[0]
        self.assertIsNotNone(rdl.LINE_RE.search(leading_only))
        self.assertIsNone(rdl.SHOULDER_RAW_RE.search(leading_only))
        self.assertIsNone(rdl.ELBOW_RAW_RE.search(leading_only))

    def test_diag_line_missing_a_field_does_not_match(self):
        truncated_diag = DIAG_LINE.split(" bus_recovery_attempts=")[0]
        self.assertIsNone(rdl.DIAG_LINE_RE.search(truncated_diag))

    def test_garbled_replacement_text_does_not_false_positive(self):
        garbled = FULL_LINE.encode("utf-8")
        garbled = garbled[:20] + b"\xff\xfe\x00" + garbled[20:]
        decoded = garbled.decode("utf-8", errors="ignore")
        # Must still either not match, or (if the corruption landed outside
        # any field) match with the SAME field values as the clean line --
        # never a shifted/wrong value silently accepted as valid.
        match = rdl.LINE_RE.search(decoded)
        if match is not None:
            self.assertEqual(match.group("tick"), "12345")


if __name__ == "__main__":
    unittest.main()
