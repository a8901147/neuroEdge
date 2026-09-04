"""IMU-signal -> MuJoCo integration test: the layer test_arm_kinematics.py and
test_arm_workspace.py deliberately skip (they drive MuJoCo ctrl directly,
never touching sensor data at all -- see their own docstrings). This one
feeds SYNTHETIC but PHYSICALLY-DERIVED raw MPU6050 accel/gyro readings
through the real C++ decode path (src/mujoco_bridge_demo.cpp, which mirrors
firmware/src/phase3_control_loop_main.cpp's shoulder axis remap + dot-product
elbow_bend -- resynced with firmware 2026-09-03 after being found stale, see
that file's own comment) and then through the exact same Data->ctrl mapping
run_demo_live.py uses, ending in a real MuJoCo forward-kinematics check.

This is the test that would have caught 2026-09-03's two real bugs before
ever touching hardware:
  1. shoulder_pitch's sign being backwards in the Data->MuJoCo ctrl mapping
     (run_demo_live.py/run_demo.py) -- positive pitch rendered as the arm
     swinging BEHIND the shoulder, not in front.
  2. The shoulder axis remap in phase3_control_loop_main.cpp feeding the
     wrong raw accelerometer channel into pitch (and the actual pitch-
     sensitive channel into the shared roll/reference denominator instead)
     -- a real forward-raise on hardware showed up almost entirely as
     shoulder_roll change (~0.1->1.9rad) with shoulder_pitch barely moving.

Covers a normal left-arm ROM within this project's real 2-IMU sensing budget
(shoulder_yaw and wrist unobservable -- see test_arm_workspace.py's own
docstring): forward flexion, backward extension, abduction (left),
adduction (right), and elbow flexion, each checked against REST.

Fixture provenance -- every pose below is REAL captured data, not invented:
  - REST, ABDUCTION_LEFT, ADDUCTION_RIGHT, ELBOW_FLEXION: captured
    2026-09-03 with tools/mujoco_bridge/log_raw_imu.py (a raw-only logger,
    no MuJoCo/mjpython/zero-pose handshake needed -- built specifically
    because the firmware's decoded shoulder_pitch/shoulder_roll go through
    ComplementaryFilter's gyro integration, which a live session the same
    night showed drifting multiple radians with ZERO corresponding
    accelerometer change -- raw values sidestep that bug entirely rather
    than working around it; see phase3_control_loop_main.cpp's "NOT YET
    RE-VERIFIED" gyro-pairing comment). Shoulder+elbow raw were captured
    together in the same pose for these four, so they're a real paired
    reading, not two different sessions' data stitched together.
  - BACKWARD_EXTENSION: also captured with log_raw_imu.py (same session as
    the four above), real paired shoulder+elbow.
  - FORWARD_RAISE: shoulder raw captured live via run_demo_live.py's [CORR]
    log during a real, isolated motion, before log_raw_imu.py existed --
    elbow raw for this one reuses REST's elbow reading as an approximation
    (the real forearm IMU wasn't logged in that session; the elbow stayed
    straight throughout the motion per the instructions given, so this is
    a reasonable stand-in, not a measurement of that specific moment).

Each pose is held constant for enough synthetic ticks (see HOLD_TICKS) for
the complementary filter (alpha=0.98, ~0.5s/500-tick time constant at this
1kHz stream rate) to fully converge, so the decoded values this test asserts
against are deterministic, not dependent on the filter's transient gyro-
integration path (gyro is held at exactly 0 throughout -- this test is
intentionally about the ACCEL-driven steady-state decode + Data->MuJoCo
mapping, not gyro dynamics).

Usage:
    python3 tools/mujoco_bridge/test_imu_to_mujoco.py
"""

import csv
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import mujoco

REPO_ROOT = Path(__file__).resolve().parents[2]
CPP_BINARY = REPO_ROOT / "build" / "debug-heapguard" / "edgeneuro_mujoco_bridge_demo"
SCENE_XML = REPO_ROOT / "tools" / "mujoco_bridge" / "arm_hand_scene.xml"

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_demo_live import (  # noqa: E402
    clamp,
    wrap_angle_delta,
    SHOULDER_PITCH_RANGE,
    SHOULDER_ROLL_RANGE,
    ELBOW_OFFSET,
    ELBOW_RANGE,
)

# Two REST references, not one -- FORWARD_RAISE was captured in an earlier,
# separate session (before log_raw_imu.py existed) than the other four
# (REST/BACKWARD_EXTENSION/ABDUCTION_LEFT/ADDUCTION_RIGHT/ELBOW_FLEXION,
# all captured together with log_raw_imu.py in one sitting). The mount gets
# re-adjusted between sessions, so comparing a pose against a REST from a
# DIFFERENT session bakes in a spurious "mount moved between sessions"
# offset on top of the real motion -- found the hard way: with a single
# shared REST, FORWARD_RAISE showed a suspiciously huge pitch delta AND a
# roll swing that looked exactly like 2026-09-03's bug #2 signature again,
# even though that bug was already fixed; re-capturing a same-session REST
# for the log_raw_imu.py group made ELBOW_FLEXION's shoulder-unchanged
# check immediately correct (delta dropped from ~1.3rad to ~0.04rad),
# which only makes sense if the earlier shared-REST comparison itself was
# the thing that was wrong, not the decode/mapping code.
#
# name -> (shoulder_raw(ax,ay,az), elbow_raw(ax,ay,az), rest_group) -- see
# module docstring for exactly where each number came from.
POSES = {
    "REST_A": ((0.960, -0.336, 0.005), (0.933, -0.231, 0.254), None),
    "FORWARD_RAISE": ((-0.27, -0.65, 0.72), (0.933, -0.231, 0.254), "A"),
    "REST_B": ((0.970, -0.317, -0.031), (0.957, -0.280, -0.124), None),
    "BACKWARD_EXTENSION": ((0.061, -0.964, -0.201), (-0.024, -0.645, -0.748), "B"),
    "ABDUCTION_LEFT": ((-0.037, -0.514, 0.872), (-0.170, -0.958, 0.209), "B"),
    "ADDUCTION_RIGHT": ((0.612, -0.646, 0.498), (-0.058, -0.988, -0.009), "B"),
    "ELBOW_FLEXION": ((0.951, -0.371, -0.024), (-0.838, -0.510, 0.067), "B"),
}
POSE_ORDER = list(POSES.keys())
REST_NAME = {"A": "REST_A", "B": "REST_B"}

HOLD_TICKS = 1000  # >> the filter's ~500-tick convergence time constant at kDt=0.001s (1kHz)
LINE_RE = re.compile(
    r"shoulder_pitch=(?P<shoulder_pitch>[-\d.eE+]+) shoulder_roll=(?P<shoulder_roll>[-\d.eE+]+) "
    r"elbow=(?P<elbow>[-\d.eE+]+)"
)


def write_fixture_csv(path):
    """Each pose in POSE_ORDER held for HOLD_TICKS ticks back-to-back --
    long enough for the complementary filter to fully settle within each
    segment, so the LAST row of each segment is a clean, deterministic
    steady-state reading."""
    with open(path, "w", newline="") as f:
        # lineterminator="\n": csv's default "\r\n" leaves a trailing \r on
        # each line that CsvSignalProvider's C++ parser (std::getline splits
        # on \n only) can't strip, which makes strtof's end-pointer check
        # fail on every row's last field and silently drops the whole file
        # as unparseable (found the hard way: 2000 written rows, 0 decoded).
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(
            ["emg0",
             "imu1_ax", "imu1_ay", "imu1_az", "imu1_gx", "imu1_gy", "imu1_gz",
             "imu2_ax", "imu2_ay", "imu2_az", "imu2_gx", "imu2_gy", "imu2_gz"]
        )
        for name in POSE_ORDER:
            (ax, ay, az), (eax, eay, eaz), _rest_group = POSES[name]
            for _ in range(HOLD_TICKS):
                writer.writerow([0.0, ax, ay, az, 0.0, 0.0, 0.0, eax, eay, eaz, 0.0, 0.0, 0.0])


def run_decode(csv_path):
    """Runs the real C++ decode binary against the fixture, returns
    {pose_name: (shoulder_pitch, shoulder_roll, elbow)} using the LAST line
    of each pose's HOLD_TICKS segment -- i.e. its settled reading."""
    if not CPP_BINARY.exists():
        raise SystemExit(
            f"bridge binary not found: {CPP_BINARY}\n"
            f"build it first: cmake --build build/debug-heapguard "
            f"--target edgeneuro_mujoco_bridge_demo"
        )
    result = subprocess.run(
        [str(CPP_BINARY), str(csv_path)],
        capture_output=True, text=True, check=True,
    )
    lines = result.stdout.strip().splitlines()
    expected = len(POSE_ORDER) * HOLD_TICKS
    if len(lines) != expected:
        raise SystemExit(
            f"expected {expected} decoded lines, got {len(lines)} -- "
            f"binary/fixture mismatch, not a real assertion failure"
        )

    def parse(line):
        m = LINE_RE.search(line)
        if not m:
            raise SystemExit(f"line didn't match LINE_RE: {line!r}")
        return float(m.group("shoulder_pitch")), float(m.group("shoulder_roll")), float(m.group("elbow"))

    decoded = {}
    for i, name in enumerate(POSE_ORDER):
        decoded[name] = parse(lines[(i + 1) * HOLD_TICKS - 1])
    return decoded


def mujoco_wrist_position(pitch_ctrl, roll_ctrl, elbow_ctrl=1.28):
    """Real mj_forward check -- elbow defaults to 1.28, the confirmed
    hang/straight reference (see run_demo_live.py's ELBOW_OFFSET comment)."""
    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)
    sp_adr = model.jnt_qposadr[model.joint("left_shoulder_pitch_joint").id]
    sr_adr = model.jnt_qposadr[model.joint("left_shoulder_roll_joint").id]
    el_adr = model.jnt_qposadr[model.joint("left_elbow_joint").id]
    shoulder_body = model.body("left_shoulder_roll_link").id
    wrist_body = model.body("left_wrist_yaw_link").id

    data.qpos[sp_adr] = pitch_ctrl
    data.qpos[sr_adr] = roll_ctrl
    data.qpos[el_adr] = elbow_ctrl
    mujoco.mj_forward(model, data)
    return data.xpos[wrist_body] - data.xpos[shoulder_body]  # (front, left, up)


def pitch_delta(decoded, name, rest):
    """wrap_angle_delta'd pitch difference from REST -- see
    run_demo_live.py's wrap_angle_delta comment: this mount's rest pose
    decodes very close to the atan2 branch cut, so a plain subtraction can
    read a near-2*pi false swing for what's actually a small real angle."""
    return wrap_angle_delta(decoded[name][0] - rest[0])


def roll_delta(decoded, name, rest):
    return wrap_angle_delta(decoded[name][1] - rest[1])


def to_ctrl(decoded, name, rest):
    """Zero-corrects `name`'s decoded (pitch, roll, elbow) against REST and
    maps through the exact same formulas run_demo_live.py's main loop
    uses, returning (pitch_ctrl, roll_ctrl, elbow_ctrl)."""
    _, _, elbow = decoded[name]
    _, _, rest_elbow = rest
    pitch_ctrl = clamp(-pitch_delta(decoded, name, rest), *SHOULDER_PITCH_RANGE)
    roll_ctrl = clamp(roll_delta(decoded, name, rest), *SHOULDER_ROLL_RANGE)
    elbow_ctrl = clamp(ELBOW_OFFSET - (elbow - rest_elbow), *ELBOW_RANGE)
    return pitch_ctrl, roll_ctrl, elbow_ctrl


def main():
    with tempfile.TemporaryDirectory() as tmpdir:
        csv_path = Path(tmpdir) / "fixture.csv"
        write_fixture_csv(csv_path)
        decoded = run_decode(csv_path)

    for name in POSE_ORDER:
        p, r, e = decoded[name]
        print(f"decoded {name:20s} shoulder_pitch={p:+.4f} shoulder_roll={r:+.4f} elbow={e:+.4f}")
    print()

    def rest_for(name):
        _, _, group = POSES[name]
        return decoded[REST_NAME[group]]

    failures = []

    def check(cond, msg):
        if not cond:
            failures.append(msg)

    # --- FORWARD_RAISE: pitch up, roll bounded, MuJoCo front > 0 ---
    rest = rest_for("FORWARD_RAISE")
    dp, dr = pitch_delta(decoded, "FORWARD_RAISE", rest), roll_delta(decoded, "FORWARD_RAISE", rest)
    pitch_ctrl, roll_ctrl, _ = to_ctrl(decoded, "FORWARD_RAISE", rest)
    rel = mujoco_wrist_position(pitch_ctrl, roll_ctrl)
    print(f"FORWARD_RAISE   d_pitch={dp:+.4f} d_roll={dr:+.4f}  ctrl: pitch={pitch_ctrl:+.4f} roll={roll_ctrl:+.4f}  |  "
          f"mujoco (front,left,up)=({rel[0]:+.4f},{rel[1]:+.4f},{rel[2]:+.4f})m")
    check(dp >= 0.3,
          f"FORWARD_RAISE: shoulder_pitch barely moved (delta={dp:+.4f}) -- "
          f"2026-09-03 bug #2 signature (shoulder axis remap regression).")
    check(abs(dr) < 0.5,
          f"FORWARD_RAISE: shoulder_roll swung {abs(dr):.4f}rad -- "
          f"2026-09-03 bug #2 signature (forward motion leaking into roll).")
    check(rel[0] >= 0.15,
          f"FORWARD_RAISE: MuJoCo front only {rel[0]:+.4f}m, expected >= +0.15m -- "
          f"2026-09-03 bug #1 signature (Data->MuJoCo pitch sign regression).")

    # --- BACKWARD_EXTENSION: pitch down (opposite sign from forward), roll bounded ---
    rest = rest_for("BACKWARD_EXTENSION")
    dp, dr = pitch_delta(decoded, "BACKWARD_EXTENSION", rest), roll_delta(decoded, "BACKWARD_EXTENSION", rest)
    pitch_ctrl, roll_ctrl, _ = to_ctrl(decoded, "BACKWARD_EXTENSION", rest)
    rel = mujoco_wrist_position(pitch_ctrl, roll_ctrl)
    print(f"BACKWARD_EXTENSION d_pitch={dp:+.4f} d_roll={dr:+.4f}  ctrl: pitch={pitch_ctrl:+.4f} roll={roll_ctrl:+.4f}  |  "
          f"mujoco (front,left,up)=({rel[0]:+.4f},{rel[1]:+.4f},{rel[2]:+.4f})m")
    check(dp <= -0.3,
          f"BACKWARD_EXTENSION: shoulder_pitch didn't drop (delta={dp:+.4f}), "
          f"expected <= -0.3 (opposite sign from FORWARD_RAISE).")
    check(abs(dr) < 0.5,
          f"BACKWARD_EXTENSION: shoulder_roll swung {abs(dr):.4f}rad, expected < 0.5rad.")
    check(rel[0] <= 0.05,  # behind or near-neutral, NOT swung forward like FORWARD_RAISE
          f"BACKWARD_EXTENSION: MuJoCo front={rel[0]:+.4f}m looks like it swung forward, "
          f"not backward -- Data->MuJoCo pitch sign may be wrong for this direction.")

    # --- ABDUCTION_LEFT: roll changes, MuJoCo left component clearly positive ---
    rest = rest_for("ABDUCTION_LEFT")
    dp, dr = pitch_delta(decoded, "ABDUCTION_LEFT", rest), roll_delta(decoded, "ABDUCTION_LEFT", rest)
    pitch_ctrl, roll_ctrl, _ = to_ctrl(decoded, "ABDUCTION_LEFT", rest)
    rel = mujoco_wrist_position(pitch_ctrl, roll_ctrl)
    print(f"ABDUCTION_LEFT  d_pitch={dp:+.4f} d_roll={dr:+.4f}  ctrl: pitch={pitch_ctrl:+.4f} roll={roll_ctrl:+.4f}  |  "
          f"mujoco (front,left,up)=({rel[0]:+.4f},{rel[1]:+.4f},{rel[2]:+.4f})m")
    check(dr >= 0.3,
          f"ABDUCTION_LEFT: shoulder_roll barely moved (delta={dr:+.4f}), expected >= +0.3.")
    check(rel[1] >= 0.1,
          f"ABDUCTION_LEFT: MuJoCo left component only {rel[1]:+.4f}m, expected >= +0.1m "
          f"(positive roll should swing the wrist to the wearer's own left).")

    # --- ADDUCTION_RIGHT: roll changes the OPPOSITE way from abduction ---
    rest = rest_for("ADDUCTION_RIGHT")
    dp, dr = pitch_delta(decoded, "ADDUCTION_RIGHT", rest), roll_delta(decoded, "ADDUCTION_RIGHT", rest)
    pitch_ctrl, roll_ctrl, _ = to_ctrl(decoded, "ADDUCTION_RIGHT", rest)
    rel = mujoco_wrist_position(pitch_ctrl, roll_ctrl)
    print(f"ADDUCTION_RIGHT d_pitch={dp:+.4f} d_roll={dr:+.4f}  ctrl: pitch={pitch_ctrl:+.4f} roll={roll_ctrl:+.4f}  |  "
          f"mujoco (front,left,up)=({rel[0]:+.4f},{rel[1]:+.4f},{rel[2]:+.4f})m")
    check(dr <= -0.15,
          f"ADDUCTION_RIGHT: shoulder_roll didn't drop (delta={dr:+.4f}), "
          f"expected <= -0.15 (opposite direction from ABDUCTION_LEFT).")
    check(rel[1] <= -0.02,
          f"ADDUCTION_RIGHT: MuJoCo left component={rel[1]:+.4f}m, expected clearly negative "
          f"(toward the wearer's right).")

    # --- ELBOW_FLEXION: elbow angle up, shoulder ~unchanged, MuJoCo wrist pulls up/in ---
    rest = rest_for("ELBOW_FLEXION")
    dp, dr = pitch_delta(decoded, "ELBOW_FLEXION", rest), roll_delta(decoded, "ELBOW_FLEXION", rest)
    e = decoded["ELBOW_FLEXION"][2]
    pitch_ctrl, roll_ctrl, elbow_ctrl = to_ctrl(decoded, "ELBOW_FLEXION", rest)
    rel = mujoco_wrist_position(pitch_ctrl, roll_ctrl, elbow_ctrl)
    print(f"ELBOW_FLEXION   d_pitch={dp:+.4f} d_roll={dr:+.4f}  ctrl: pitch={pitch_ctrl:+.4f} roll={roll_ctrl:+.4f} "
          f"elbow={elbow_ctrl:+.4f}  |  mujoco (front,left,up)=({rel[0]:+.4f},{rel[1]:+.4f},{rel[2]:+.4f})m")
    check(e - rest[2] >= 0.5,
          f"ELBOW_FLEXION: decoded elbow angle barely changed ({rest[2]:+.4f} -> {e:+.4f}), "
          f"expected >= +0.5 (real flexion is a ~110deg swing in the dot-product angle).")
    check(abs(dp) < 0.3 and abs(dr) < 0.3,
          f"ELBOW_FLEXION: shoulder moved (d_pitch={dp:+.4f}, d_roll={dr:+.4f}) "
          f"during a fixture meant to hold the upper arm still.")
    check(elbow_ctrl < 1.28 - 0.3,
          f"ELBOW_FLEXION: elbow ctrl={elbow_ctrl:+.4f} isn't meaningfully bent away from "
          f"straight (1.28) -- Data->MuJoCo elbow mapping may be wrong.")

    if failures:
        print("\nFAILED:")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)
    print("\nPASSED: all 5 poses (forward, backward, left, right, elbow) decode "
          "and map to the expected MuJoCo direction.")


if __name__ == "__main__":
    main()
