"""servo_pose_4ch.py: the command parser, the step planner and the interactive loop
against a fake board that speaks servo_limit_finder_4ch's protocol. No hardware."""
import contextlib
import io
import re
import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
import servo_pose_4ch as sp  # noqa: E402


class FakeBoard:
    """Speaks servo_limit_finder_4ch: '1'-'4' select + print state, '+'/'-' move 25us
    (silently), clamped to 300..2700 like the firmware."""

    def __init__(self, start=(1475, 1475, 1475, 1475)):
        self.pulse = list(start)
        self.active = 0
        self.out = b""
        self.writes = []

    def write(self, data):
        for b in data:
            c = chr(b)
            self.writes.append(c)
            if c in "1234":
                self.active = int(c) - 1
                self.out += f"channel={sp.NAMES[self.active]} pulse_us={self.pulse[self.active]}\n".encode()
            elif c == "+" and self.pulse[self.active] + 25 <= 2700:
                self.pulse[self.active] += 25
            elif c == "-" and self.pulse[self.active] - 25 >= 300:
                self.pulse[self.active] -= 25

    @property
    def in_waiting(self):
        return len(self.out)

    def read(self, n=1):
        data, self.out = self.out[:n], self.out[n:]
        return data

    def reset_input_buffer(self):
        self.out = b""


class ParseTest(unittest.TestCase):
    def test_commands(self):
        self.assertEqual(sp.parse("1"), ("select", 1))
        self.assertEqual(sp.parse("+"), ("step", +1))
        self.assertEqual(sp.parse("- 4"), ("step", -4))
        self.assertEqual(sp.parse("+3"), ("step", +3))
        self.assertEqual(sp.parse("set 2 1650"), ("set", 2, 1650))
        self.assertEqual(sp.parse("set shoulder 1650"), ("set", 2, 1650))
        self.assertEqual(sp.parse("all 1500"), ("all", 1500))
        self.assertEqual(sp.parse("rest"), ("rest",))
        self.assertEqual(sp.parse("show"), ("show",))
        self.assertEqual(sp.parse("q"), ("quit",))
        self.assertEqual(sp.parse("  SET 3 900  "), ("set", 3, 900))

    def test_garbage_is_reported_not_raised(self):
        for line in ("", "hello", "set", "set 9 1500", "set 2 abc", "all", "+ x", "5"):
            self.assertEqual(sp.parse(line)[0], "error", line)


class PlanTest(unittest.TestCase):
    def test_steps_are_25us_and_signed(self):
        self.assertEqual(sp.plan_steps(1500, 1600), (+1, 4))
        self.assertEqual(sp.plan_steps(1500, 1400), (-1, 4))
        self.assertEqual(sp.plan_steps(1500, 1500), (+1, 0))

    def test_a_target_off_the_25us_grid_rounds_to_the_nearest_step(self):
        self.assertEqual(sp.plan_steps(1500, 1612), (+1, 4))   # 1600
        self.assertEqual(sp.plan_steps(1500, 1613), (+1, 5))   # 1625

    def test_targets_are_clamped_into_the_measured_range_and_say_so(self):
        for ch, (lo, hi) in sp.RANGES.items():
            self.assertEqual(sp.clamp_target(ch, lo - 500)[0], lo)
            self.assertEqual(sp.clamp_target(ch, hi + 500)[0], hi)
            self.assertTrue(sp.clamp_target(ch, hi + 500)[1])          # flagged as clamped
            self.assertFalse(sp.clamp_target(ch, (lo + hi) // 2)[1])

    def test_the_rest_pose_matches_the_firmware_header_and_is_inside_the_measured_ranges(self):
        header = (Path(__file__).resolve().parents[1] / "include/edgeneuro/control/mearm_servo_maps.hpp").read_text()
        for name, ch in zip(("Base", "Shoulder", "Elbow", "Claw"), (1, 2, 3, 4)):
            self.assertEqual(sp.REST[ch], int(re.search(rf"k{name}RestUs = (\d+)u", header)[1]), name)
            lo, hi = sp.RANGES[ch]
            self.assertTrue(lo <= sp.REST[ch] <= hi, name)

    def test_measured_ranges_match_the_firmware_maps(self):
        header = (Path(__file__).resolve().parents[1] / "include/edgeneuro/control/mearm_servo_maps.hpp").read_text()
        for name, ch in zip(("Base", "Shoulder", "Elbow", "Claw"), (1, 2, 3, 4)):
            lo = int(re.search(rf"k{name}LoUs = (\d+)u", header)[1])
            hi = int(re.search(rf"k{name}HiUs = (\d+)u", header)[1])
            self.assertEqual(sp.RANGES[ch], (lo, hi), name)


class SessionTest(unittest.TestCase):
    def run_session(self, lines, board=None):
        board = board or FakeBoard()
        out = io.StringIO()
        with mock.patch("builtins.input", side_effect=lines + ["q"]), \
                mock.patch.object(sp.time, "sleep", lambda s: None), contextlib.redirect_stdout(out):
            sp.session(board)
        return board, out.getvalue()

    def test_set_walks_the_channel_to_the_target(self):
        board, out = self.run_session(["set 2 1650"])
        self.assertEqual(board.pulse, [1475, 1650, 1475, 1475])
        self.assertIn("pulse_us=1650", out)

    def test_moves_are_made_one_25us_step_at_a_time_never_a_jump(self):
        board, _ = self.run_session(["set 1 2000"])
        moves = [c for c in board.writes if c in "+-"]
        self.assertEqual(moves, ["+"] * 21)                                    # (2000-1475)/25

    def test_a_target_outside_the_measured_range_is_clamped_and_reported(self):
        board, out = self.run_session(["set 4 2500"])
        self.assertEqual(board.pulse[3], 1500)                                 # the claw's limit (user, 2026-10-03)
        self.assertIn("1500", out)
        self.assertIn("範圍", out)

    def test_plus_and_minus_move_the_selected_channel(self):
        board, _ = self.run_session(["3", "+ 2", "- 1"])
        self.assertEqual(board.pulse, [1475, 1475, 1500, 1475])

    def test_all_sets_base_shoulder_and_elbow_within_their_own_ranges_and_always_puts_the_claw_at_its_rest(self):
        # "all 1500 (except the claw, which is 1300 = open)": the claw is never sent to the common value
        board, _ = self.run_session(["all 1500"])
        self.assertEqual(board.pulse, [1500, 1500, 1500, 1300])
        board, _ = self.run_session(["all 1000"])
        self.assertEqual(board.pulse, [1000, 1200, 1000, 1300])               # each clamped to its own lo
        board, _ = self.run_session(["all 1800"], board=FakeBoard(start=(1500, 1500, 1500, 1600)))
        self.assertEqual(board.pulse, [1800, 1800, 1800, 1300])               # even from a closed claw

    def test_rest_goes_to_the_chosen_rest_pose(self):
        board, out = self.run_session(["rest"], board=FakeBoard(start=(1900, 1300, 700, 1550)))
        self.assertEqual(board.pulse, [1500, 1500, 1500, 1300])
        self.assertRegex(out, r"base=1500 shoulder=1500 elbow=1500 claw=1300")

    def test_rest_and_all_move_the_elbow_before_the_shoulder(self):
        # (a shoulder move with the elbow elsewhere could bind the linkage)
        for line in ("rest", "all 1600"):
            board, _ = self.run_session([line], board=FakeBoard(start=(1500, 1300, 700, 1550)))
            selects = [c for c in board.writes if c in "1234"]
            firsts = []
            for c in selects:
                if c not in firsts:
                    firsts.append(c)
            self.assertLess(firsts.index("3"), firsts.index("2"), line)

    def test_show_reads_all_four_from_the_board(self):
        _, out = self.run_session(["show"], board=FakeBoard(start=(1500, 1600, 1700, 1400)))
        for name, v in zip(sp.NAMES, (1500, 1600, 1700, 1400)):
            self.assertRegex(out, rf"{name}\s+{v}")

    def test_a_bad_command_does_not_touch_the_board(self):
        board, out = self.run_session(["set 9 1500", "banana"])
        self.assertEqual([c for c in board.writes if c in "+-"], [])
        self.assertIn("看不懂", out)

    def test_quitting_prints_the_final_pose_as_a_paste_ready_line(self):
        _, out = self.run_session(["all 1500"])
        self.assertRegex(out, r"base=1500 shoulder=1500 elbow=1500 claw=1300")     # the claw stays at its rest

    def test_a_silent_board_is_reported_not_guessed(self):
        class Dead(FakeBoard):
            def write(self, data):
                pass
        with self.assertRaises(sp.BoardNotAnswering):
            sp.read_state(Dead(), 1)


if __name__ == "__main__":
    unittest.main()
