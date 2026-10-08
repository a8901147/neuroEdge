"""measure_servo_response.py: what the board really sends to the four MEArm servos (TIM3 CCR1..4 read over SWD), phase by
phase. Checked against synthetic samples and a fake recorder -- no hardware."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import measure_servo_response as msr  # noqa: E402


def samples(values_per_ms, dt=0.001):
    """[(t, base, shoulder, elbow, claw), ...] from a list of 4-tuples, one per dt."""
    return [(i * dt, *v) for i, v in enumerate(values_per_ms)]


class ParseTest(unittest.TestCase):
    def test_openocd_lines_become_time_and_four_pulses(self):
        text = "1082 0x5d6 0x5e2 0x4da 0x514\n2294 0x5d7 0x5e2 0x4da 0x514\ngarbage\n"
        self.assertEqual(msr.parse_samples(text), [(0.001082, 1494, 1506, 1242, 1300), (0.002294, 1495, 1506, 1242, 1300)])

    def test_the_tcl_script_reads_the_four_ccr_registers_for_the_requested_time(self):
        tcl = msr.tcl_script("/tmp/x.txt", 7.5)
        self.assertIn("read_memory 0x40000434 32 4", tcl)         # TIM3->CCR1..CCR4 (base address + 0x34)
        self.assertIn("7500000", tcl)
        self.assertIn("/tmp/x.txt", tcl)


class StatsTest(unittest.TestCase):
    def test_a_still_servo_has_zero_spread_and_no_steps(self):
        st = msr.channel_stats(samples([(1500, 1600, 1200, 1300)] * 100), 0)
        self.assertEqual((st["spread"], st["changes"], st["max_step"]), (0, 0, 0))

    def test_spread_steps_and_rate_limit_hits(self):
        base = [1500, 1500, 1510, 1510, 1570, 1570, 1565]        # steps 10, 60, 5
        st = msr.channel_stats(samples([(b, 1600, 1200, 1300) for b in base]), 0)
        self.assertEqual(st["spread"], 70)
        self.assertEqual(st["changes"], 3)
        self.assertEqual(st["max_step"], 60)
        self.assertEqual(st["at_rate_limit"], 1)                   # only the 60 us step (the ramp's 10 ms limit)

    def test_std_is_reported(self):
        st = msr.channel_stats(samples([(1400, 0, 0, 0), (1600, 0, 0, 0)] * 50), 0)
        self.assertAlmostEqual(st["std"], 100.0)


class ProtocolTest(unittest.TestCase):
    def test_still_holds_cover_the_poses_where_the_base_was_seen_to_swing(self):
        names = [p[0] for p in msr.PHASES]
        for name in ("hold_hang", "hold_half", "hold_forward"):
            self.assertIn(name, names)
        self.assertTrue(any(n.startswith("task") for n in names))


class FlowTest(unittest.TestCase):
    def test_each_phase_waits_for_enter_and_can_be_redone(self):
        recorded = []

        def record(seconds):
            recorded.append(seconds)
            return samples([(1500, 1600, 1200, 1300)] * 10)

        answers = iter(["", "", "", "r", "", ""] + [""] * 40)      # phase 1: start, keep; phase 2: start, REDO, start, keep
        prompts = []

        def fake_input(prompt=""):
            prompts.append(prompt)
            return next(answers)
        phases = msr.run_protocol(record, fake_input, out=lambda *a, **k: None, sleep=lambda s: None, phases=msr.PHASES[:2])
        self.assertEqual(list(phases), [p[0] for p in msr.PHASES[:2]])
        self.assertEqual(len(recorded), 3)                         # phase 2 recorded twice
        self.assertTrue(any("Enter" in p for p in prompts))

    def test_results_go_to_a_new_file_never_overwriting(self):
        d = Path(tempfile.mkdtemp())
        p = msr.save({"hold_hang": samples([(1500, 1600, 1200, 1300)] * 3)}, d)
        data = json.loads(p.read_text())
        self.assertEqual(data["phases"]["hold_hang"][0], [0.0, 1500, 1600, 1200, 1300])
        with self.assertRaises(FileExistsError):
            msr.save({}, d, name=p.name)


class SummaryTest(unittest.TestCase):
    def test_the_summary_names_each_phase_and_servo(self):
        text = "\n".join(msr.summary_lines({"hold_hang": samples([(1500, 1600, 1200, 1300)] * 10)}))
        self.assertIn("hanging", text)
        for servo in ("base", "shoulder", "elbow", "claw"):
            self.assertIn(servo, text)


if __name__ == "__main__":
    unittest.main()
