"""End-to-end (offline) test of `run_demo_live.py --mearm`: a FAKE serial
port streams real-format UART lines, a FAKE viewer stands in for the window,
and the REAL run_mearm_preview() -> reader thread -> LINE_RE parsing ->
mearm_pathb -> MuJoCo actuator ctrl path runs in between. Catches the class
of bug unit tests can't: wiring mistakes in the glue (wrong variable, name
only defined on one branch, a wait that never ends) -- several of which this
function already hit on real hardware.

(That --humanoid stays untouched is verified by hand with `git diff HEAD`
when the change is made, NOT by a permanent test: a "no removed lines vs
HEAD" test would go red the moment anyone legitimately retunes a Path A
constant, which is exactly the kind of test-fights-retuning this project
avoids.)

Uses the embedded 9/13 calibration vectors from test_mearm_direction.py.
"""

import contextlib
import io
import json
import math
import sys
import tempfile
import threading
import time
import types
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_demo_live as rdl  # noqa: E402
import mearm_calibration_link as link  # noqa: E402
import test_mearm_direction as fx  # noqa: E402  (embedded 9/13 vectors)


def uart_line(shoulder_raw, elbow, grip=0.0, gripping=0):
    ax, ay, az = shoulder_raw
    return (f"tick=1 grip={grip:.3f} gripping={gripping} shoulder_pitch=0.000 shoulder_roll=0.000 "
            f"elbow={elbow:.4f} shoulder_raw_ax={ax:+.4f} shoulder_raw_ay={ay:+.4f} "
            f"shoulder_raw_az={az:+.4f} shoulder_raw_gx=+0.000 shoulder_raw_gy=+0.000 "
            f"shoulder_raw_gz=+0.000 elbow_raw_ax=+0.000 elbow_raw_ay=+0.000 "
            f"elbow_raw_az=+1.000\r\n").encode()


_NOISE = __import__("random").Random(7)
_RAW_RE = __import__("re").compile(rb"(_raw_a[xyz]=)([-+]?\d+\.\d+)")


def with_noise(line):
    """Real accelerometers are never bit-for-bit constant: perturb every raw accel value a little (sensor_health treats a
    long run of identical readings as a frozen sensor -- the 2026-09-27 failure)."""
    # (an exact 0 is left alone: the firmware's all-zero "no reading yet" default is exactly zero, not noisy)
    return _RAW_RE.sub(lambda m: m.group(0) if float(m.group(2)) == 0.0 else
                       m.group(1) + f"{float(m.group(2)) + _NOISE.gauss(0, 0.003):+.4f}".encode(), line)


def with_unused_fields_changed(line):
    """The same accelerometer vectors, but the firmware's complementary-filter angles and the upper-arm gyro set to large
    values. The docs say neither drives any model; only the raw accelerometer vectors, the elbow bend and the grip do."""
    return (line.replace(b"shoulder_pitch=0.000 shoulder_roll=0.000", b"shoulder_pitch=1.400 shoulder_roll=-1.100")
                .replace(b"shoulder_raw_gx=+0.000 shoulder_raw_gy=+0.000 shoulder_raw_gz=+0.000",
                         b"shoulder_raw_gx=+3.000 shoulder_raw_gy=-2.500 shoulder_raw_gz=+4.000"))


class FakeSerial:
    """Streams the same line over and over (like a live 1kHz stream) until closed."""

    def __init__(self, line):
        self.line = line
        self.closed = False
        self.written = []

    def write(self, data):
        self.written.append(bytes(data))
        return len(data)

    def read(self, n):
        if self.closed:
            time.sleep(0.05)                  # a real port blocks for its timeout; don't leave a thread spinning
            return b""
        time.sleep(0.002)
        return with_noise(self.line)

    def close(self):
        self.closed = True


class FakeViewer:
    def __init__(self, ticks, frames=None):
        self.ticks = ticks
        self.frames = frames          # optional shared [count]: lets a fake port follow the SAME clock as the loop

    def is_running(self):
        self.ticks -= 1
        if self.frames is not None:
            self.frames[0] += 1
        return self.ticks > 0

    def sync(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def run_preview(line, saved, ticks=800, serial_factory=None, events=None, skip_emg=True, emg_calibrated=None,
                skip_calibration=True, cal_timeout=0.05, frames=None):
    """Runs the real run_mearm_preview against a fake port/viewer; returns
    (final data.ctrl by actuator name, captured stdout). serial_factory, if
    given, builds the fake port instead of the default constant stream."""
    tmp = Path(tempfile.mkdtemp())
    calib = tmp / "shoulder_calibration.json"
    calib.write_text(json.dumps(dict({"emg_threshold": 1129}, **saved)))
    args = types.SimpleNamespace(calibration_file=calib, port="fake", cp2102=False, baud=115200,
                                 skip_emg_calibration=skip_emg, skip_calibration=skip_calibration)
    captured = {}

    def fake_launch(model, data):
        captured["model"], captured["data"] = model, data
        return FakeViewer(ticks, frames)

    out = io.StringIO()
    events = events if events is not None else []
    ports = []

    def make_port(*a, **k):
        port = serial_factory() if serial_factory else FakeSerial(line)
        if not hasattr(port, "write"):
            port.write = lambda data: len(data)
        real_write = port.write

        def write(data):
            events.append(("write", bytes(data)))
            return real_write(data)
        port.write = write
        ports.append(port)
        return port

    def fake_input(prompt=""):
        events.append(("input", prompt))
        return ""

    def launch(model, data):
        events.append(("viewer", None))
        return fake_launch(model, data)

    def fake_emg_calibration(ser, latest, interactive=True):
        events.append(("emg_calibration", interactive))
        return emg_calibrated

    # most fakes never confirm the calibration; tests not about it should not wait for it. A fake port cannot overrun,
    # so the real board's per-byte pause (test_mearm_calibration_link) is not needed here either
    with mock.patch.object(rdl, "CAL_CONFIRM_TIMEOUT_S", cal_timeout), \
            mock.patch.object(link, "CHUNK_PAUSE_S", 0.0), \
            mock.patch.object(rdl, "calibrate_emg_threshold", fake_emg_calibration), \
            mock.patch.object(rdl.serial, "Serial", make_port), \
            mock.patch.object(rdl.mujoco.viewer, "launch_passive", launch), \
            mock.patch("builtins.input", fake_input), \
            contextlib.redirect_stdout(out):
        rdl.run_mearm_preview(args)
    m, d = captured["model"], captured["data"]
    ctrl = {n: float(d.ctrl[m.actuator(n).id]) for n in ("base", "shoulder", "elbow", "claw")}
    run_preview.last_calibration = json.loads(calib.read_text())
    return ctrl, out.getvalue()


class OnlyTheAccelerometerVectorsDriveTheModelTest(unittest.TestCase):
    """Design claim (README, PRD 5): the models follow the raw gravity vectors; the gyroscopes and the firmware's
    complementary-filter roll/pitch are diagnostic only."""

    def test_gyro_and_complementary_filter_fields_do_not_move_the_mearm_model(self):
        for raw in (fx.LEFT, fx.FORWARD):
            line = uart_line(raw, fx.STRAIGHT)
            changed = with_unused_fields_changed(line)
            self.assertNotEqual(line, changed)                 # the fields really were changed
            base, _ = run_preview(line, fx.SAVED_9_13, ticks=400)
            other, _ = run_preview(changed, fx.SAVED_9_13, ticks=400)
            for joint in ("base", "shoulder", "elbow", "claw"):
                with self.subTest(raw=raw, joint=joint):
                    self.assertAlmostEqual(base[joint], other[joint], delta=0.02)   # sensor-noise level only


class StartPoseTest(unittest.TestCase):
    """2026-10-03 (the author's design): before following, the person is asked to let the arm hang; on Enter the
    firmware is sent R, which walks every servo back to the start pose (base/shoulder/elbow 1500, claw 1300 open)."""

    def test_the_person_is_asked_to_let_the_arm_hang_and_R_is_sent_only_after_enter(self):
        events = []
        _ctrl, out = run_preview(uart_line(fx.HANG, fx.STRAIGHT), fx.SAVED_9_13, ticks=50, events=events)
        prompts = [i for i, e in enumerate(events) if e[0] == "input" and "hang" in e[1]]
        writes = [i for i, e in enumerate(events) if e[0] == "write" and e[1].startswith(b"R")]
        self.assertTrue(prompts, events)
        self.assertEqual(len(writes), 1, events)
        self.assertLess(prompts[0], writes[0])                     # R only after Enter

    def test_R_is_sent_before_the_preview_starts_following(self):
        events = []
        run_preview(uart_line(fx.HANG, fx.STRAIGHT), fx.SAVED_9_13, ticks=50, events=events)
        write_i = next(i for i, e in enumerate(events) if e[0] == "write" and e[1].startswith(b"R"))
        viewer_i = next(i for i, e in enumerate(events) if e[0] == "viewer")
        self.assertLess(write_i, viewer_i)


class CalibrationBoard(FakeSerial):
    """A fake phase3_control_loop with servos ON: streams the tick line plus a diag line ~once a second, and -- like the
    real firmware -- counts a complete "C...\n" calibration message as applied (cal_applied in the diag line)."""

    def __init__(self, line, confirms=True, garble_first=0, refuses=False):
        super().__init__(line)
        self.received = b""
        self.cal_applied = 0
        self.cal_malformed = 0
        self.cal_rejected = 0
        self.refuses = refuses                # arrives intact but fails the board's own check (pathb Calibration::make)
        self.confirms = confirms
        self.garble_first = garble_first      # this many messages arrive damaged (a lost byte), like the real overrun
        self.reads = 0

    def write(self, data):
        self.written.append(bytes(data))
        self.received += bytes(data)
        while b"\n" in self.received:
            msg, self.received = self.received.split(b"\n", 1)
            if msg.startswith(b"C") and self.garble_first > 0:
                self.garble_first -= 1
                self.cal_malformed += 1
            elif msg.startswith(b"C") and self.refuses:
                self.cal_rejected += 1
            elif msg.startswith(b"C") and self.confirms:
                self.cal_applied += 1
        return len(data)

    def read(self, n):
        out = super().read(n)
        self.reads += 1
        if self.reads % 200 == 0 and not self.closed:
            return (f"diag shoulder_completions=246 elbow_completions=247 shoulder_nacks=0 shoulder_timeouts=0 "
                    f"elbow_nacks=0 elbow_timeouts=0 active_reader_state=0 shoulder_wake_result=0 elbow_wake_result=0 "
                    f"shoulder_required=1 elbow_required=1 bus_recovery_attempts=0 bus_recovery_freed=0 "
                    f"shoulder_asleep_rewakes=0 elbow_asleep_rewakes=0 shoulder_pwr_mgmt_1=1 elbow_pwr_mgmt_1=1 "
                    f"shoulder_power_resets=0 elbow_power_resets=0 uart_skipped_lines=0 uart_dropped_bytes=0 "
                    f"uart_rx_overruns=0 cal_applied={self.cal_applied} cal_rejected={self.cal_rejected} "
                    f"cal_malformed={self.cal_malformed}\r\n").encode()
        return out


class CalibrationToBoardTest(unittest.TestCase):
    """2026-10-03 (the author's choice "B"): the calibration goes to the board over UART -- the real arm uses it at once,
    no re-flash. --skip-calibration sends the saved one; otherwise the four poses are captured first (the humanoid
    path's own calibrate_pose), saved, then sent. Sent before R, and confirmed from the board's diag line."""

    def test_a_message_damaged_on_the_way_is_sent_again(self):
        # 2026-10-04 on the real board: a byte lost to a receive overrun makes the message malformed -- that is a
        # transport error, not a bad calibration, so it is sent again (a bounded number of times)
        board = CalibrationBoard(uart_line(fx.HANG, fx.STRAIGHT), garble_first=1)
        _c, out = run_preview(None, fx.SAVED_9_13, ticks=30, serial_factory=lambda: board, cal_timeout=3.0)
        self.assertEqual(board.cal_applied, 1)
        self.assertEqual(b"".join(board.written).count(link.encode(fx.SAVED_9_13)), 2)
        self.assertIn("the board applied it", out)

    def test_a_calibration_the_board_refuses_is_reported_and_not_sent_again(self):
        board = CalibrationBoard(uart_line(fx.HANG, fx.STRAIGHT), refuses=True)
        _c, out = run_preview(None, fx.SAVED_9_13, ticks=30, serial_factory=lambda: board, cal_timeout=3.0)
        self.assertEqual(b"".join(board.written).count(link.encode(fx.SAVED_9_13)), 1)
        self.assertIn("rejected", out)

    def test_a_board_that_keeps_damaging_it_gives_up_after_a_few_tries(self):
        board = CalibrationBoard(uart_line(fx.HANG, fx.STRAIGHT), garble_first=99)
        _c, out = run_preview(None, fx.SAVED_9_13, ticks=30, serial_factory=lambda: board, cal_timeout=3.0)
        self.assertEqual(board.cal_applied, 0)
        self.assertEqual(b"".join(board.written).count(link.encode(fx.SAVED_9_13)), rdl.CAL_SEND_ATTEMPTS)
        self.assertIn("keeps its previous calibration", out)

    def test_skip_sends_the_saved_calibration_before_R_and_confirms_it(self):
        events = []
        board = CalibrationBoard(uart_line(fx.HANG, fx.STRAIGHT))
        _c, out = run_preview(None, fx.SAVED_9_13, ticks=30, events=events, serial_factory=lambda: board, cal_timeout=3.0)
        writes = [(i, e[1]) for i, e in enumerate(events) if e[0] == "write"]
        cal = [i for i, w in writes if w.startswith(b"C")]
        r = [i for i, w in writes if w.startswith(b"R")]
        sent = b"".join(w for _i, w in writes if not w.startswith((b"T", b"R")))
        self.assertEqual(sent, link.encode(fx.SAVED_9_13))
        self.assertLess(cal[0], r[0])
        self.assertIn("the board applied it", out)

    def test_a_board_that_never_confirms_is_reported_and_the_preview_still_runs(self):
        board = CalibrationBoard(uart_line(fx.HANG, fx.STRAIGHT), confirms=False)
        _c, out = run_preview(None, fx.SAVED_9_13, ticks=30, serial_factory=lambda: board, cal_timeout=1.5)
        # (1.5 s: long enough for its diag lines, which say cal_applied=0, to arrive -- that must NOT count)
        self.assertIn("did not confirm", out)

    def test_without_skip_the_four_poses_are_captured_saved_and_sent(self):
        # a DIFFERENT valid calibration from the saved 9/13 one (all poses turned 20 deg about x), so saving shows
        turn = lambda v: tuple(fx._rotate(v, (1.0, 0.0, 0.0), math.radians(20.0)))
        new = {k: turn(fx.SAVED_9_13[k]) for k in ("baseline_raw", "forward_raw", "left_twist_raw", "right_twist_raw")}
        poses = iter([(new["baseline_raw"], fx.STRAIGHT), (new["forward_raw"], None), (new["left_twist_raw"], None),
                      (new["right_twist_raw"], None)])
        calls = []

        def fake_pose(latest, instruction, ref_raw=None, min_tilt_deg=None, ignore=()):
            calls.append(instruction)
            return next(poses)
        board = CalibrationBoard(uart_line(fx.HANG, fx.STRAIGHT))
        with mock.patch.object(rdl, "calibrate_pose", fake_pose):
            _c, out = run_preview(None, fx.SAVED_9_13, ticks=30, serial_factory=lambda: board, skip_calibration=False,
                                  cal_timeout=3.0)
        self.assertEqual(len(calls), 4)
        saved = run_preview.last_calibration
        self.assertEqual(saved["forward_raw"], list(new["forward_raw"]))
        self.assertEqual(saved["right_twist_raw"], list(new["right_twist_raw"]))
        self.assertNotEqual(saved["forward_raw"], list(fx.FORWARD))
        self.assertIn(link.encode(saved), b"".join(board.written))
        self.assertIn("the board applied it", out)

    def test_a_fresh_calibration_drops_the_mujoco_alignment_made_for_the_old_one(self):
        # the saved alignment maps the OLD mount's readings to the model; with new poses it would point the model wrong
        turn = lambda v: tuple(fx._rotate(v, (1.0, 0.0, 0.0), math.radians(20.0)))
        poses = iter([(turn(ALIGNED[k]), fx.STRAIGHT if k == "baseline_raw" else None)
                      for k in ("baseline_raw", "forward_raw", "left_twist_raw", "right_twist_raw")])
        board = CalibrationBoard(uart_line(fx.HANG, fx.STRAIGHT))
        with mock.patch.object(rdl, "calibrate_pose", lambda *a, **k: next(poses)):
            _c, out = run_preview(None, ALIGNED, ticks=30, serial_factory=lambda: board, skip_calibration=False,
                                  cal_timeout=3.0)
        self.assertIn("is not used this run", out)
        self.assertIn("offline default mapping", out)


class EmgThresholdTest(unittest.TestCase):
    """2026-10-03: --mearm never sent the board an EMG threshold, so the firmware's built-in fallback (2800) applied
    instead of the calibrated one (e.g. 1129) and a grip had to be far stronger. Now it does what the humanoid path does,
    with the same functions: --skip-emg-calibration sends the saved threshold, otherwise the interactive calibration runs
    (and its result is saved). Before the R start pose, so the claw already responds properly from the start."""

    def writes(self, events):
        return [(i, e[1]) for i, e in enumerate(events) if e[0] == "write"]

    def test_skip_sends_the_saved_threshold_before_R(self):
        events = []
        run_preview(uart_line(fx.HANG, fx.STRAIGHT), fx.SAVED_9_13, ticks=30, events=events, skip_emg=True)
        writes = self.writes(events)
        t = [i for i, w in writes if w.startswith(b"T")]
        r = [i for i, w in writes if w.startswith(b"R")]
        self.assertEqual([w for _i, w in writes if w.startswith(b"T")], [b"T1129\n"])
        self.assertLess(t[0], r[0])
        self.assertFalse(any(e[0] == "emg_calibration" for e in events))

    def test_without_skip_the_interactive_calibration_runs_and_is_saved(self):
        events = []
        run_preview(uart_line(fx.HANG, fx.STRAIGHT), fx.SAVED_9_13, ticks=30, events=events, skip_emg=False,
                    emg_calibrated=1500)
        self.assertIn(("emg_calibration", True), events)
        self.assertEqual(run_preview.last_calibration["emg_threshold"], 1500)
        self.assertIn("emg_threshold_captured_at", run_preview.last_calibration)
        cal_i = next(i for i, e in enumerate(events) if e[0] == "emg_calibration")
        r_i = next(i for i, w in self.writes(events) if w.startswith(b"R"))
        self.assertLess(cal_i, r_i)


class EmgReleaseThresholdTest(unittest.TestCase):
    """2026-10-04: two thresholds. The author found the grip let go too easily: in the 13:12 calibration the relaxed level
    was ~1170, the threshold 1876, and the clench's lowest 10% only ~1908 -- a gentler hold while the arm moves dips under
    1876. The board now grips above the threshold and lets go only below a lower RELEASE threshold, halfway between the
    relaxed level and the threshold. Shared by --mearm and the humanoid path (apply_emg_threshold)."""

    def writes(self, events):
        return [w for e in events if e[0] == "write" for w in [e[1]] if w.startswith(b"T")]

    def test_the_release_threshold_is_halfway_between_relaxed_and_the_threshold(self):
        self.assertEqual(rdl.emg_release_threshold(1170.0, 1876), 1523)
        self.assertLess(rdl.emg_release_threshold(1170.0, 1876), 1876)
        self.assertGreater(rdl.emg_release_threshold(1170.0, 1876), 1170)

    def test_the_line_carries_the_release_threshold_when_there_is_one(self):
        class Port:
            def __init__(self):
                self.out = b""

            def write(self, b):
                self.out += b
        p = Port()
        rdl.send_emg_threshold(p, 1876, 1523)
        rdl.send_emg_threshold(p, 1876)
        self.assertEqual(p.out, b"T1876,1523\nT1876\n")

    def test_skip_sends_the_saved_release_threshold_too(self):
        events = []
        run_preview(uart_line(fx.HANG, fx.STRAIGHT), dict(fx.SAVED_9_13, emg_release_threshold=900), ticks=30,
                    events=events, skip_emg=True)
        self.assertEqual(self.writes(events), [b"T1129,900\n"])

    def test_an_older_calibration_without_one_sends_the_threshold_alone_and_says_how_to_get_one(self):
        events = []
        _c, out = run_preview(uart_line(fx.HANG, fx.STRAIGHT), fx.SAVED_9_13, ticks=30, events=events, skip_emg=True)
        self.assertEqual(self.writes(events), [b"T1129\n"])
        self.assertIn("release threshold", out)

    def test_a_fresh_calibration_saves_its_release_threshold(self):
        run_preview(uart_line(fx.HANG, fx.STRAIGHT), fx.SAVED_9_13, ticks=30, skip_emg=False,
                    emg_calibrated=rdl.EmgThreshold(1876, 1523))
        self.assertEqual(run_preview.last_calibration["emg_threshold"], 1876)
        self.assertEqual(run_preview.last_calibration["emg_release_threshold"], 1523)

    def test_the_calibration_itself_computes_and_sends_both(self):
        relaxed = [(i * 0.0125, 1150, 1170 + (i % 3) * 10) for i in range(160)]
        clench = [(i * 0.0125, 2300, 2500) for i in range(160)]
        tails = iter([relaxed, clench])
        sent = []

        class Port:
            def write(self, b):
                sent.append(b)
        with mock.patch.object(rdl, "capture_emg_window", lambda *a, **k: next(tails)), \
                contextlib.redirect_stdout(io.StringIO()):
            result = rdl.calibrate_emg_threshold(Port(), types.SimpleNamespace(snapshot_emg_raw=lambda: (1150, 1170)),
                                                 interactive=False)
        mean, std = rdl.emg_mean_std([s[2] for s in relaxed])
        self.assertEqual(int(result), int(round(mean + rdl.EMG_THRESHOLD_K * std)))
        self.assertEqual(result.release, rdl.emg_release_threshold(mean, int(result)))
        self.assertEqual(sent, [f"T{int(result)},{result.release}\n".encode()])


class RunMearmPreviewEndToEndTest(unittest.TestCase):
    def test_arm_left_drives_the_model_base_left_via_path_b_default(self):
        ctrl, out = run_preview(uart_line(fx.LEFT, fx.STRAIGHT), fx.SAVED_9_13)
        self.assertGreater(ctrl["base"], 0.5)
        self.assertIn("Path B's offline default mapping", out)

    def test_arm_right_drives_the_model_base_right(self):
        ctrl, _ = run_preview(uart_line(fx.RIGHT, fx.STRAIGHT), fx.SAVED_9_13)
        self.assertLess(ctrl["base"], -0.5)

    def test_raising_the_arm_lifts_the_model_shoulder(self):
        hang, _ = run_preview(uart_line(fx.HANG, fx.STRAIGHT), fx.SAVED_9_13)
        fwd, _ = run_preview(uart_line(fx.FORWARD, fx.STRAIGHT), fx.SAVED_9_13)
        # smaller ctrl = higher elevation (measured 2026-09-24)
        self.assertLess(fwd["shoulder"], hang["shoulder"] - 0.5)
        self.assertLess(abs(fwd["base"]), 0.05)

    def test_elbow_flexion_folds_the_model_elbow(self):
        straight, _ = run_preview(uart_line(fx.HANG, fx.STRAIGHT), fx.SAVED_9_13)
        flexed, _ = run_preview(uart_line(fx.HANG, fx.FLEXED), fx.SAVED_9_13)
        # window-limited: the linkage lets the elbow travel ~0.55 rad at a fixed shoulder
        self.assertGreater(flexed["elbow"], straight["elbow"] + 0.4)

    def test_the_live_numbers_line_shows_tilt_and_azimuth(self):
        _, out = run_preview(uart_line(fx.LEFT, fx.STRAIGHT), fx.SAVED_9_13)
        self.assertRegex(out, r"tilt=\s*\d+deg az=\s*[+-]\d+deg")

    def test_missing_calibration_key_exits_with_a_clear_message(self):
        broken = {k: v for k, v in fx.SAVED_9_13.items() if k != "right_twist_raw"}
        with self.assertRaises(SystemExit) as cm:
            run_preview(uart_line(fx.HANG, fx.STRAIGHT), broken)
        self.assertIn("right_twist_raw", str(cm.exception))

    def test_with_an_interactive_alignment_saved_it_is_used_instead(self):
        _, out = run_preview(uart_line(fx.LEFT, fx.STRAIGHT), ALIGNED)
        self.assertIn("using pose-anchored alignment", out)
        self.assertNotIn("Path B's offline default mapping", out)

    def test_an_alignment_from_the_old_oblique_decode_is_refused_with_a_clear_message(self):
        old_format = dict(fx.SAVED_9_13)
        old_format["mearm_alignment"] = {k: v for k, v in ALIGNED["mearm_alignment"].items()
                                         if k not in ("decode", "elbow_anchors")}
        old_format["mearm_alignment"]["anchors"] = {"shoulder": [[0, 1], [1, 0]], "base": [[0, 0], [1, 1]],
                                                    "elbow": [[0, 0], [1, 1]]}
        with self.assertRaises(SystemExit) as cm:
            run_preview(uart_line(fx.HANG, fx.STRAIGHT), old_format)
        self.assertIn("calibrate_mearm_alignment.py", str(cm.exception))


ALIGNED = dict(fx.SAVED_9_13)
ALIGNED["mearm_alignment"] = {
    "decode": "spherical",
    "elbow_anchors": [[0.434, 0.995], [2.6, 2.617]],
    "claw_open_ctrl": 0.14, "claw_closed_ctrl": 1.78,
    "captures": {"hang_raw": fx.HANG, "forward_raw": fx.FORWARD,
                 "left_twist_raw": fx.LEFT, "right_twist_raw": fx.RIGHT, "hang_elbow": 0.434},
}


class AlignedPathRespectsTheLinkageTest(unittest.TestCase):
    """The interactive-alignment branch must obey the shoulder/elbow linkage
    band exactly like the offline default does: it maps the elbow through
    anchors independently, which on a MeArm (claw-levelling parallel linkage)
    is infeasible for most (shoulder, elbow) pairs."""

    def test_sum_stays_in_the_band_for_every_pose_combination(self):
        import mearm_pathb as pb
        lo, hi = pb.linkage_band()
        for shoulder_name, shoulder in (("HANG", fx.HANG), ("FORWARD", fx.FORWARD), ("LEFT", fx.LEFT)):
            for elbow_name, elbow in (("straight", fx.STRAIGHT), ("flexed", fx.FLEXED)):
                with self.subTest(shoulder=shoulder_name, elbow=elbow_name):
                    ctrl, _ = run_preview(uart_line(shoulder, elbow), ALIGNED)
                    total = ctrl["shoulder"] + ctrl["elbow"]
                    self.assertGreaterEqual(total, lo - 1e-3)
                    self.assertLessEqual(total, hi + 1e-3)


    def test_matches_mearm_pathb_on_the_same_captures_and_anchors(self):
        # glue check: the preview must add nothing of its own to the mapping
        import mearm_pathb as pb
        cap = ALIGNED["mearm_alignment"]["captures"]
        cal = pb.Calibration(cap["hang_raw"], cap["forward_raw"], cap["left_twist_raw"],
                             cap["right_twist_raw"], cap["hang_elbow"])
        for name, raw in (("HANG", fx.HANG), ("FORWARD", fx.FORWARD), ("LEFT", fx.LEFT), ("RIGHT", fx.RIGHT)):
            for bend in (fx.STRAIGHT, fx.FLEXED):
                with self.subTest(pose=name, bend=bend):
                    ctrl, _ = run_preview(uart_line(raw, bend), ALIGNED)
                    b, sh, el = pb.ctrl_from_sensors(cal, raw, bend,
                                                     elbow_anchors=ALIGNED["mearm_alignment"]["elbow_anchors"])
                    # (the fake port adds sensor-like noise of ~0.003 g, so the match is to within that noise's effect)
                    self.assertAlmostEqual(ctrl["base"], b, delta=0.02)
                    self.assertAlmostEqual(ctrl["shoulder"], sh, delta=0.02)
                    self.assertAlmostEqual(ctrl["elbow"], el, delta=0.02)


class AlignedPreviewUsesWhatTheToolMeasuredTest(unittest.TestCase):
    """Fixture values that DIFFER from the offline defaults, so ignoring them fails."""

    def _saved(self):
        saved = dict(ALIGNED)
        al = dict(ALIGNED["mearm_alignment"])
        al["elbow_anchors"] = [[0.434, 2.617], [2.6, 0.995]]        # user said: fold looks like the OTHER direction
        al["claw_open_ctrl"], al["claw_closed_ctrl"] = 1.78, 0.14   # user said: the high end is OPEN
        saved["mearm_alignment"] = al
        return saved

    def test_measured_elbow_polarity_is_honoured(self):
        saved = self._saved()
        straight, _ = run_preview(uart_line(fx.HANG, fx.STRAIGHT), saved)
        flexed, _ = run_preview(uart_line(fx.HANG, fx.FLEXED), saved)
        self.assertLess(flexed["elbow"], straight["elbow"] - 0.4)      # default polarity would rise

    def test_measured_claw_ends_are_honoured(self):
        saved = self._saved()
        open_, _ = run_preview(uart_line(fx.HANG, fx.STRAIGHT, grip=0.0), saved)
        closed, _ = run_preview(uart_line(fx.HANG, fx.STRAIGHT, grip=1.0), saved)
        self.assertAlmostEqual(open_["claw"], 1.78, places=2)
        self.assertAlmostEqual(closed["claw"], 0.14, places=2)


class ScriptedSerial:
    """read() plays back `script(n)` for the n-th call: bytes, b"" (silence) or
    an exception instance to raise. Sleeps like a real 2ms poll."""

    def __init__(self, script):
        self.script, self.n, self.closed = script, 0, False

    def read(self, _n):
        if self.closed:
            time.sleep(0.05)                  # a real port blocks for its timeout; don't leave a thread spinning
            return b""
        time.sleep(0.002)
        item = self.script(self.n)
        self.n += 1
        if isinstance(item, BaseException):
            raise item
        return with_noise(item) if getattr(self, "noisy", True) else item

    def close(self):
        self.closed = True


def all_finite(ctrl):
    import math
    return all(math.isfinite(v) for v in ctrl.values())


class SensorHealthTest(unittest.TestCase):
    """The automatic checks of 2026-09-28 (sensor_health.py): no preview on faulty sensor data, and a loud warning plus a
    held model if a sensor fails mid-session -- so a wiring fault is never mistaken for an algorithm problem."""
    GOOD_LEFT = uart_line(fx.LEFT, fx.STRAIGHT)
    GOOD_RIGHT = uart_line(fx.RIGHT, fx.STRAIGHT)

    @staticmethod
    def frozen_forearm(upper):
        # the real 9/27 failure: the forearm stuck at (1.999939, 0, 0) while reads still complete
        line = uart_line(upper, fx.STRAIGHT).decode()
        line = line.replace("elbow_raw_ax=+0.000 elbow_raw_ay=+0.000 elbow_raw_az=+1.000",
                            "elbow_raw_ax=+1.999939 elbow_raw_ay=+0.000000 elbow_raw_az=+0.000000")
        assert "1.999939" in line
        return line.encode()

    def test_a_faulty_sensor_at_start_blocks_the_preview_with_a_clear_warning(self):
        frozen = self.frozen_forearm(fx.LEFT)

        def factory():
            s = ScriptedSerial(lambda n: frozen)
            s.noisy = False                                   # the frozen line stays bit-identical, like the real one
            return s
        with mock.patch.object(rdl, "MEARM_HEALTH_PREFLIGHT_MAX_S", 1.0):
            with self.assertRaises(SystemExit) as cm:
                run_preview(None, fx.SAVED_9_13, serial_factory=factory)
        self.assertIn("forearm", str(cm.exception))
        self.assertIn("cannot be trusted", str(cm.exception))

    def test_the_preview_starts_by_itself_once_the_sensors_become_healthy(self):
        frozen = self.frozen_forearm(fx.LEFT)

        # frozen until the preview has actually said so, then healthy: not "the first 200 reads", which on a slower
        # machine (CI, 2026-10-04) were used up before the health check even started. If it never warns, the board
        # stays frozen, the preflight times out and this fails.
        def factory():
            return ScriptedSerial(lambda n: frozen if "HARDWARE FAULT" not in sys.stdout.getvalue() else self.GOOD_LEFT)
        with mock.patch.object(rdl, "MEARM_HEALTH_PREFLIGHT_MAX_S", 20.0):
            ctrl, out = run_preview(None, fx.SAVED_9_13, serial_factory=factory)
        self.assertIn("HARDWARE FAULT", out)                           # it said so while waiting
        self.assertGreater(ctrl["base"], 0.5)                  # and then ran normally

    def test_a_line_arriving_between_the_check_and_the_use_is_never_applied(self):
        """CI 2026-10-07/08 (3 failures, always base=-0.9083...): a step checked one sample, then read the sensors AGAIN to
        compute the pose, so a line that arrived in between -- here the first line of a frozen forearm, with the arm
        already swung RIGHT -- was applied unchecked, and the model then held that wrong pose. Forced deterministically:
        the fake port hands over that line exactly inside the loop's own plausibility check."""
        frozen_right = self.frozen_forearm(fx.RIGHT)
        armed, released, stored = threading.Event(), threading.Event(), threading.Event()
        real_plausible = rdl.sensor_health.plausible

        def racing_plausible(v):
            ok = real_plausible(v)
            if (armed.is_set() and not released.is_set() and sys._getframe(1).f_code.co_name == "run_mearm_preview"
                    and v[0] == 0.0 and v[1] == 0.0):          # the loop's forearm check (exact zeros in GOOD lines)
                released.set()                                 # the step has checked its sample: let the next line in
                stored.wait(2.0)                               # ... and wait until the reader thread has stored it
            return ok

        events = []
        reads_since_viewer = [0]

        def script(n):
            if not any(e[0] == "viewer" for e in events) or reads_since_viewer[0] < 100:
                if any(e[0] == "viewer" for e in events):
                    reads_since_viewer[0] += 1
                return self.GOOD_LEFT                          # following LEFT for a while first
            if not armed.is_set():
                armed.set()
                released.wait(5.0)
                return frozen_right
            stored.set()                                       # the reader came back: the line above is stored
            return frozen_right

        def factory():
            return ScriptedSerial(script)
        with mock.patch.object(rdl.sensor_health, "plausible", racing_plausible):
            ctrl, out = run_preview(None, fx.SAVED_9_13, ticks=1500, serial_factory=factory, events=events)
        self.assertTrue(released.is_set() and stored.is_set(), "the race was never forced -- the test proved nothing")
        self.assertGreater(ctrl["base"], 0.5, out)             # held at LEFT: the unchecked RIGHT line was not applied
        self.assertIn("HARDWARE FAULT", out)

    def test_a_fault_mid_session_holds_the_model_warns_and_recovers(self):
        """Phases follow the viewer's frame count, not the number of reads (2026-10-08): with two independent clocks,
        a slower or faster reader thread moved the fault to a different frame and the result changed with the machine."""
        frozen_right = self.frozen_forearm(fx.RIGHT)

        def run(ticks):
            frames = [0]

            def script(_n):
                if frames[0] < 200:
                    return self.GOOD_LEFT
                if frames[0] < 2200:                           # >= 2 s, longer than the health check's 1 s window
                    return frozen_right                        # the arm swings RIGHT while the forearm is frozen
                return self.GOOD_LEFT

            return run_preview(None, fx.SAVED_9_13, ticks=ticks, serial_factory=lambda: ScriptedSerial(script),
                               frames=frames)
        ctrl_mid, out_mid = run(1500)                          # stopped inside the fault
        self.assertGreater(ctrl_mid["base"], 0.5)              # held at LEFT: the RIGHT swing was not followed
        self.assertIn("HARDWARE FAULT", out_mid)
        ctrl_end, out_end = run(4500)                          # >= 2.3 s of healthy data after the fault
        self.assertIn("back to normal", out_end)
        self.assertGreater(ctrl_end["base"], 0.5)


class HardwareWarningPreviewTest(unittest.TestCase):
    GOOD_LEFT = uart_line(fx.LEFT, fx.STRAIGHT)

    @staticmethod
    def diag(nacks, rewakes):
        return (f"diag shoulder_completions=246 elbow_completions=247 shoulder_nacks={nacks} shoulder_timeouts=0 "
                f"elbow_nacks=0 elbow_timeouts=0 active_reader_state=0 shoulder_wake_result=0 elbow_wake_result=0 "
                f"shoulder_required=1 elbow_required=1 bus_recovery_attempts=0 bus_recovery_freed=0 "
                f"shoulder_asleep_rewakes={rewakes} elbow_asleep_rewakes=0 shoulder_pwr_mgmt_1=1 elbow_pwr_mgmt_1=1 "
                f"shoulder_power_resets=0 elbow_power_resets=0\r\n").encode()

    def test_dropouts_are_shown_and_the_model_keeps_following(self):
        def script(n):
            if n in (300, 600):
                return self.diag(0, 353) if n == 300 else self.diag(4, 357)
            return self.GOOD_LEFT
        ctrl, out = run_preview(None, fx.SAVED_9_13, ticks=1200, serial_factory=lambda: ScriptedSerial(script))
        self.assertIn("dropped out", out)
        self.assertIn("upper arm", out)
        self.assertGreater(ctrl["base"], 0.5)                  # still following LEFT: a warning does not hold the model


class RobustnessToBadInputTest(unittest.TestCase):
    GOOD_LEFT = uart_line(fx.LEFT, fx.STRAIGHT)

    def test_junk_and_truncated_lines_between_valid_ones_are_ignored(self):
        junk = [b"\x00\xff garbage \r\n", b"tick=1 grip=0.0 gripp", b"Stage 5b: boot banner\r\n", b"\r\n"]

        def script(n):
            return junk[n % 4] if n % 2 else self.GOOD_LEFT

        ctrl, _ = run_preview(None, fx.SAVED_9_13, serial_factory=lambda: ScriptedSerial(script))
        self.assertTrue(all_finite(ctrl))
        self.assertGreater(ctrl["base"], 0.5)          # still follows the valid LEFT lines

    def test_infinite_or_overflowing_numbers_never_reach_the_model(self):
        # float("1e999") is inf -- it matches the line regex, so it is parsed
        # (formatted by hand: Python would print "+inf", which the regex ignores)
        bad = uart_line(fx.LEFT, fx.STRAIGHT).decode().replace(
            "elbow=", "elbow=1e999 x=").replace(
            f"shoulder_raw_ax={fx.LEFT[0]:+.4f}", "shoulder_raw_ax=+1e999").encode()
        self.assertIn(b"1e999", bad)
        # a stream of nothing but such readings is a sensor fault: the preview refuses to start (2026-09-28)
        with mock.patch.object(rdl, "MEARM_HEALTH_PREFLIGHT_MAX_S", 1.0):
            with self.assertRaises(SystemExit) as cm:
                run_preview(bad, fx.SAVED_9_13)
        self.assertIn("cannot be trusted", str(cm.exception))
        # ...and mixed into good readings mid-session, they are never applied: the model stays finite and where it was
        ctrl, _ = run_preview(None, fx.SAVED_9_13, ticks=900,
                              serial_factory=lambda: ScriptedSerial(lambda n: self.GOOD_LEFT if n < 150 or n % 2 else bad))
        self.assertTrue(all_finite(ctrl), ctrl)
        self.assertGreater(ctrl["base"], 0.5)

    def test_a_zero_vector_mid_session_holds_the_last_good_pose_and_warns(self):
        # (2026-09-28: a zero vector is a sensor fault; the model neither follows it nor jumps to rest)
        zero = uart_line((0.0, 0.0, 0.0), fx.STRAIGHT)
        ctrl, out = run_preview(None, fx.SAVED_9_13, ticks=900,
                                serial_factory=lambda: ScriptedSerial(lambda n: self.GOOD_LEFT if n < 150 else zero))
        self.assertGreater(ctrl["base"], 0.5)                  # still where the LEFT arm put it
        self.assertIn("HARDWARE FAULT", out)

    def test_and_it_follows_the_arm_again_once_real_data_returns(self):
        zero = uart_line((0.0, 0.0, 0.0), fx.STRAIGHT)
        ctrl, _ = run_preview(None, fx.SAVED_9_13, ticks=900,
                              serial_factory=lambda: ScriptedSerial(
                                  lambda n: zero if 60 <= n < 200 else self.GOOD_LEFT))
        self.assertGreater(ctrl["base"], 0.5)

    def test_a_missing_upper_arm_vector_prints_a_hint_instead_of_waiting_silently(self):
        # tick lines keep arriving but the upper-arm vector is the firmware's all-zero default
        zero = uart_line((0.0, 0.0, 0.0), fx.STRAIGHT)
        with mock.patch.object(rdl, "MEARM_WAIT_HINT_SECONDS", 0.05):
            _ctrl, out = run_preview(None, fx.SAVED_9_13, ticks=200,
                                     serial_factory=lambda: ScriptedSerial(
                                         lambda n: zero if n < 150 else self.GOOD_LEFT))
        self.assertIn("MPU6050", out)

    def test_an_unplugged_port_stops_the_preview_with_a_clear_message(self):
        import serial

        def script(n):
            return self.GOOD_LEFT if n < 50 else serial.SerialException("device disconnected")

        with self.assertRaises(SystemExit) as cm:
            run_preview(None, fx.SAVED_9_13, ticks=3000, serial_factory=lambda: ScriptedSerial(script))
        self.assertIn("serial port failed", str(cm.exception))
        self.assertIn("device disconnected", str(cm.exception))

    def test_a_silent_firmware_is_flagged_stale_once_and_ok_again_when_it_returns(self):
        def script(n):
            return b"" if 100 <= n < 400 else self.GOOD_LEFT

        with mock.patch.object(rdl, "STALE_AFTER_SECONDS", 0.05):
            _ctrl, out = run_preview(None, fx.SAVED_9_13, ticks=1500, serial_factory=lambda: ScriptedSerial(script))
        self.assertEqual(out.count("[STALE]"), 1, out)
        self.assertEqual(out.count("[OK]"), 1, out)
        self.assertLess(out.index("[STALE]"), out.index("[OK]"))


if __name__ == "__main__":
    unittest.main()
