"""mearm_calibration_link.py: the calibration as the one-line UART message the firmware parses
(include/edgeneuro/control/mearm_calibration_link.hpp -- C<20 values x1e6>,<checksum>\\n). The 9/13 message below is the
very one tests/test_mearm_calibration_link.cpp parses, so the two sides agree byte for byte."""
import math
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import mearm_calibration_link as link  # noqa: E402
import test_mearm_direction as fx  # noqa: E402

K913 = [992637, 46787, 265020, 33409, -10960, 1024878, 122861, 451801, 918384, 112252, -461119, 899955, 434012,
        0, 0, 0, 0, 0, 0, 0]


def expected(values):
    return ("C" + "".join(f"{v}," for v in values) + str(sum(values) % 1000000007) + "\n").encode("ascii")


class EncodeTest(unittest.TestCase):
    def test_the_9_13_calibration_is_exactly_the_message_the_cpp_parser_test_uses(self):
        self.assertEqual(link.encode(fx.SAVED_9_13), expected(K913))

    def test_a_measured_base_reach_is_included(self):
        saved = dict(fx.SAVED_9_13, base_reach_left_raw=[0.3, 0.6, 0.75], base_reach_right_raw=[0.25, -0.62, 0.74])
        v = K913[:13] + [1000000, 300000, 600000, 750000, 250000, -620000, 740000]
        self.assertEqual(link.encode(saved), expected(v))

    def test_negative_sums_give_a_non_negative_checksum(self):
        # the 9/13 vectors turned 180 deg about y (a proper rotation: still a valid calibration), mostly negative
        turn = lambda v: [-v[0], v[1], -v[2]]
        saved = dict(fx.SAVED_9_13, **{k: turn(fx.SAVED_9_13[k]) for k in
                                       ("baseline_raw", "forward_raw", "left_twist_raw", "right_twist_raw")})
        msg = link.encode(saved).decode()
        self.assertLess(sum(int(x) for x in msg[1:].split(",")[:20]), 0)     # the case being tested
        self.assertFalse(msg.strip().split(",")[-1].startswith("-"))

    def test_a_calibration_path_b_would_reject_is_refused_not_sent(self):
        bad = dict(fx.SAVED_9_13, forward_raw=fx.SAVED_9_13["baseline_raw"])     # FORWARD == HANG
        with self.assertRaises(ValueError):
            link.encode(bad)

    def test_a_non_finite_value_is_refused(self):
        bad = dict(fx.SAVED_9_13, zero_elbow=math.nan)
        with self.assertRaises(ValueError):
            link.encode(bad)


class SendTest(unittest.TestCase):
    def test_send_writes_the_message_one_byte_at_a_time_paced(self):
        # measured on the real board 2026-10-04: 8-byte chunks overran its single-byte receive register (uart_rx_overruns
        # +2 per message, every message malformed); one byte per 2 ms: 3/3 applied, no overrun; per 1 ms: 1 of 3 lost
        writes, sleeps = [], []

        class Port:
            def write(self, b):
                writes.append(bytes(b))
                return len(b)
        link.send(Port(), fx.SAVED_9_13, sleep=sleeps.append)
        self.assertEqual(b"".join(writes), link.encode(fx.SAVED_9_13))
        self.assertTrue(all(len(w) == 1 for w in writes))
        self.assertGreaterEqual(link.CHUNK_PAUSE_S, 0.002)
        self.assertEqual(len(sleeps), len(writes))


if __name__ == "__main__":
    unittest.main()
