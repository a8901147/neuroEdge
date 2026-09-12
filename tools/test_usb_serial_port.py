"""Unit tests for usb_serial_port.py's autodetect_port() -- extracted
2026-09-12 from run_demo_live.py after every other script with a --port
flag turned out to share the exact same hardcoded-CP2102-path problem, but
the extraction itself picked up zero test coverage in the move (the
original copy in run_demo_live.py never had a dedicated test either).
Covers the branches a real hardware session can't easily exercise on
demand: zero adapters plugged in, more than one plugged in at once, and
--cp2102 given when no CP2102 is actually connected.

Mocks glob.glob and Path.exists rather than touching real /dev entries --
this needs to pass with no hardware plugged in at all (CI has none).

Usage:
    python3 -m unittest tools/test_usb_serial_port.py -v
    python3 tools/test_usb_serial_port.py
"""

import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import usb_serial_port as usp  # noqa: E402


class AutodetectPortTest(unittest.TestCase):
    def test_no_candidates_raises_system_exit(self):
        with patch("usb_serial_port.glob.glob", return_value=[]):
            with self.assertRaises(SystemExit):
                usp.autodetect_port()

    def test_single_candidate_returned(self):
        with patch("usb_serial_port.glob.glob", return_value=["/dev/tty.usbserial-A73C97JW"]):
            self.assertEqual(usp.autodetect_port(), "/dev/tty.usbserial-A73C97JW")

    def test_multiple_candidates_returns_first_sorted(self):
        # Real scenario: both a CP2102 and an FT232RL plugged in at once --
        # must pick one deterministically (sorted, not glob's raw OS order)
        # rather than crash or pick randomly.
        with patch(
            "usb_serial_port.glob.glob",
            return_value=["/dev/tty.usbserial-Z999", "/dev/tty.usbserial-0001"],
        ):
            self.assertEqual(usp.autodetect_port(), "/dev/tty.usbserial-0001")

    def test_prefer_cp2102_when_present(self):
        with patch("usb_serial_port.Path.exists", return_value=True):
            self.assertEqual(usp.autodetect_port(prefer_cp2102=True), usp.CP2102_PORT)

    def test_prefer_cp2102_when_absent_raises_system_exit(self):
        with patch("usb_serial_port.Path.exists", return_value=False):
            with self.assertRaises(SystemExit):
                usp.autodetect_port(prefer_cp2102=True)

    def test_prefer_cp2102_ignores_other_candidates(self):
        # --cp2102 means "use CP2102 specifically", not "prefer it among
        # whatever's found" -- must not fall through to glob at all once
        # the fixed path exists, regardless of what else is plugged in.
        with patch("usb_serial_port.Path.exists", return_value=True), \
             patch("usb_serial_port.glob.glob", return_value=["/dev/tty.usbserial-A73C97JW"]) as mock_glob:
            self.assertEqual(usp.autodetect_port(prefer_cp2102=True), usp.CP2102_PORT)
            mock_glob.assert_not_called()


if __name__ == "__main__":
    unittest.main()
