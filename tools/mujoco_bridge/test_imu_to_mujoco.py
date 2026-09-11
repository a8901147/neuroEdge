"""IMU-signal -> MuJoCo integration test: the layer test_arm_kinematics.py and
test_arm_workspace.py deliberately skip (they drive MuJoCo ctrl directly,
never touching sensor data at all -- see their own docstrings). This one
feeds SYNTHETIC but PHYSICALLY-DERIVED raw MPU6050 accel/gyro readings
through the real C++ decode path (src/mujoco_bridge_demo.cpp) and then
through the exact same Data->ctrl mapping run_demo_live.py uses, ending in a
real MuJoCo forward-kinematics check.

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

2026-09-04 rewrite: the shoulder decode itself moved from
ComplementaryFilter::pitch()/roll() (accel-only formula that folds back
past +-90deg, PLUS a gyro-integration path that a live session showed
drifting multiple radians with zero corresponding accel change) to
tilt_azimuth.hpp's oblique_decompose_scaled(), read from the C++ binary's
added pitch_equiv=/roll_equiv= fields (shoulder_pitch=/shoulder_roll= are
UNCHANGED and still read by tools/mujoco_bridge/run_demo.py's separate,
fully-synthetic-data consumer -- see src/mujoco_bridge_demo.cpp's own
comment on why both fields coexist). pitch_equiv/roll_equiv are already
REST-referenced by construction (the calibration basis's own ref IS this
session's REST reading), so no more wrap_angle_delta/zero-subtraction is
needed for the shoulder the way the old shoulder_pitch/shoulder_roll
needed -- only elbow still needs a zero-subtraction (elbow_bend is a plain
absolute dot-product angle, unaffected by any of this).

Covers a normal left-arm ROM within this project's real 2-IMU sensing budget
(shoulder_yaw and wrist unobservable -- see test_arm_workspace.py's own
docstring): forward flexion, backward extension, abduction (left),
adduction (right), and elbow flexion, each checked against REST.

Fixture provenance: all 6 poses below are REAL captured data, averaged over
a 5-repeat capture in a SINGLE sitting (tools/mujoco_bridge/
raw_imu_calibration.json, 2026-09-05, via `log_raw_imu.py --repeats 5`) --
deliberately not mixed with any other session's capture. An earlier version
of this test compared FORWARD_RAISE (a different, earlier session) against
a REST from yet another session, and hit a spurious huge pitch delta + a
roll swing that looked exactly like bug #2's signature again, even though
that bug was already fixed -- the mount shifts slightly between separate
wearing sessions, and that shift alone produces a fake "motion" on top of
the real one. Recapturing everything in one sitting (this file's current
POSES) made that spurious signal disappear entirely, which only makes sense
if the earlier cross-session comparison itself was the thing that was
wrong. See src/mujoco_bridge_demo.cpp's own calibration-basis comment for
why this same capture is also what's hardcoded as the shoulder's
calibration basis there.

Each pose is held constant for enough synthetic ticks (see HOLD_TICKS) for
the complementary filter (alpha=0.98, ~0.5s/500-tick time constant at this
1kHz stream rate) to fully converge -- the OLD shoulder_pitch=/shoulder_roll=
fields still go through that filter (see above), so this hold is kept even
though the NEW pitch_equiv=/roll_equiv= fields (this test's actual asserts)
are computed directly from the raw accel each tick with no such transient.

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
    SHOULDER_PITCH_RANGE,
    SHOULDER_ROLL_RANGE,
    ELBOW_OFFSET,
    ELBOW_RANGE,
)

# name -> (shoulder_raw(ax,ay,az), elbow_raw(ax,ay,az)) -- all 6 from one
# real sitting (5-repeat averaged), see module docstring for provenance. No
# more per-pose "rest group": pitch_equiv/roll_equiv are already referenced
# against this same capture's REST by construction (see
# src/mujoco_bridge_demo.cpp's hardcoded calibration basis), only elbow
# still needs REST["ELBOW_FLEXION"] style subtraction below.
POSES = {
    "REST": ((0.989913, -0.251028, -0.061491), (0.890306, -0.448106, 0.160676)),
    "FORWARD_RAISE": ((0.240373, -0.545771, 0.824905), (-0.285723, -0.902025, 0.308659)),
    "BACKWARD_EXTENSION": ((0.624760, -0.440018, -0.649655), (0.737540, 0.220604, -0.638560)),
    "ABDUCTION_LEFT": ((0.110822, -0.917584, 0.376458), (-0.062469, -0.666840, -0.718059)),
    "ADDUCTION_RIGHT": ((0.782668, -0.391534, 0.524923), (0.406508, -0.840744, 0.353306)),
    "ELBOW_FLEXION": ((0.995489, -0.140029, -0.086466), (-0.807085, -0.192952, 0.508370)),
}
POSE_ORDER = list(POSES.keys())

HOLD_TICKS = 1000  # >> the filter's ~500-tick convergence time constant at kDt=0.001s (1kHz)
LINE_RE = re.compile(
    r"elbow=(?P<elbow>[-\d.eE+]+) "
    r"pitch_equiv=(?P<pitch_equiv>[-\d.eE+]+) roll_equiv=(?P<roll_equiv>[-\d.eE+]+)"
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
            (ax, ay, az), (eax, eay, eaz) = POSES[name]
            for _ in range(HOLD_TICKS):
                writer.writerow([0.0, ax, ay, az, 0.0, 0.0, 0.0, eax, eay, eaz, 0.0, 0.0, 0.0])


def run_decode(csv_path):
    """Runs the real C++ decode binary against the fixture, returns
    {pose_name: (pitch_equiv, roll_equiv, elbow)} using the LAST line of
    each pose's HOLD_TICKS segment -- i.e. its settled reading."""
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
        return float(m.group("pitch_equiv")), float(m.group("roll_equiv")), float(m.group("elbow"))

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


def to_ctrl(decoded, name, rest_elbow):
    """Maps `name`'s decoded (pitch_equiv, roll_equiv, elbow) through the
    exact same formulas run_demo_live.py's main loop uses, returning
    (pitch_ctrl, roll_ctrl, elbow_ctrl). pitch_equiv/roll_equiv need no
    zero-subtraction (already REST-referenced by the calibration basis
    baked into src/mujoco_bridge_demo.cpp) -- only elbow does, same as
    before."""
    pitch_equiv, roll_equiv, elbow = decoded[name]
    pitch_ctrl = clamp(-pitch_equiv, *SHOULDER_PITCH_RANGE)
    roll_ctrl = clamp(roll_equiv, *SHOULDER_ROLL_RANGE)
    elbow_ctrl = clamp(ELBOW_OFFSET - (elbow - rest_elbow), *ELBOW_RANGE)
    return pitch_ctrl, roll_ctrl, elbow_ctrl


def main():
    with tempfile.TemporaryDirectory() as tmpdir:
        csv_path = Path(tmpdir) / "fixture.csv"
        write_fixture_csv(csv_path)
        decoded = run_decode(csv_path)

    for name in POSE_ORDER:
        p, r, e = decoded[name]
        print(f"decoded {name:20s} pitch_equiv={p:+.4f} roll_equiv={r:+.4f} elbow={e:+.4f}")
    print()

    rest_elbow = decoded["REST"][2]

    failures = []

    def check(cond, msg):
        if not cond:
            failures.append(msg)

    # --- FORWARD_RAISE: pitch up, roll ~exactly 0 (FORWARD_RAISE is one of
    # the 2 calibration poses, so oblique_decompose_scaled recovers (its
    # own tilt, 0) exactly by construction -- a MUCH tighter roll bound
    # than any non-calibration pose can expect, see ADDUCTION_RIGHT/
    # BACKWARD_EXTENSION below), MuJoCo front > 0 ---
    pitch_equiv, roll_equiv, _ = decoded["FORWARD_RAISE"]
    pitch_ctrl, roll_ctrl, _ = to_ctrl(decoded, "FORWARD_RAISE", rest_elbow)
    rel = mujoco_wrist_position(pitch_ctrl, roll_ctrl)
    print(f"FORWARD_RAISE   pitch_equiv={pitch_equiv:+.4f} roll_equiv={roll_equiv:+.4f}  "
          f"ctrl: pitch={pitch_ctrl:+.4f} roll={roll_ctrl:+.4f}  |  "
          f"mujoco (front,left,up)=({rel[0]:+.4f},{rel[1]:+.4f},{rel[2]:+.4f})m")
    check(pitch_equiv >= 0.3,
          f"FORWARD_RAISE: pitch_equiv barely moved ({pitch_equiv:+.4f}) -- "
          f"2026-09-03 bug #2 signature (shoulder axis remap regression).")
    check(abs(roll_equiv) < 0.1,
          f"FORWARD_RAISE: roll_equiv={roll_equiv:+.4f}, expected ~0 (exact by "
          f"construction -- it's one of the 2 calibration poses).")
    check(rel[0] >= 0.15,
          f"FORWARD_RAISE: MuJoCo front only {rel[0]:+.4f}m, expected >= +0.15m -- "
          f"2026-09-03 bug #1 signature (Data->MuJoCo pitch sign regression).")

    # --- BACKWARD_EXTENSION: pitch down (opposite sign from forward). NOT
    # one of the 2 calibration poses, so unlike FORWARD_RAISE/
    # ABDUCTION_LEFT above/below, real (and large, ~0.67rad) roll cross-
    # talk is EXPECTED here, not a bug -- see SESSION_LOG.md 2026-09-04's
    # oblique-basis analysis (real shoulder motion at these poses isn't
    # confined to 2 orthogonal planes). Only the pitch sign/magnitude and
    # the MuJoCo front position (the actual thing 2026-09-03's bug #1
    # broke) are asserted; roll is printed but not bounded. ---
    pitch_equiv, roll_equiv, _ = decoded["BACKWARD_EXTENSION"]
    pitch_ctrl, roll_ctrl, _ = to_ctrl(decoded, "BACKWARD_EXTENSION", rest_elbow)
    rel = mujoco_wrist_position(pitch_ctrl, roll_ctrl)
    print(f"BACKWARD_EXTENSION pitch_equiv={pitch_equiv:+.4f} roll_equiv={roll_equiv:+.4f} (cross-talk, not bounded)  "
          f"ctrl: pitch={pitch_ctrl:+.4f} roll={roll_ctrl:+.4f}  |  "
          f"mujoco (front,left,up)=({rel[0]:+.4f},{rel[1]:+.4f},{rel[2]:+.4f})m")
    check(pitch_equiv <= -0.3,
          f"BACKWARD_EXTENSION: pitch_equiv didn't drop ({pitch_equiv:+.4f}), "
          f"expected <= -0.3 (opposite sign from FORWARD_RAISE).")
    check(rel[0] <= 0.05,  # behind or near-neutral, NOT swung forward like FORWARD_RAISE
          f"BACKWARD_EXTENSION: MuJoCo front={rel[0]:+.4f}m looks like it swung forward, "
          f"not backward -- Data->MuJoCo pitch sign may be wrong for this direction.")

    # --- ABDUCTION_LEFT: roll changes, pitch ~exactly 0 (the other
    # calibration pose -- same reasoning as FORWARD_RAISE's pitch bound
    # above), MuJoCo left component clearly positive ---
    pitch_equiv, roll_equiv, _ = decoded["ABDUCTION_LEFT"]
    pitch_ctrl, roll_ctrl, _ = to_ctrl(decoded, "ABDUCTION_LEFT", rest_elbow)
    rel = mujoco_wrist_position(pitch_ctrl, roll_ctrl)
    print(f"ABDUCTION_LEFT  pitch_equiv={pitch_equiv:+.4f} roll_equiv={roll_equiv:+.4f}  "
          f"ctrl: pitch={pitch_ctrl:+.4f} roll={roll_ctrl:+.4f}  |  "
          f"mujoco (front,left,up)=({rel[0]:+.4f},{rel[1]:+.4f},{rel[2]:+.4f})m")
    check(roll_equiv >= 0.3,
          f"ABDUCTION_LEFT: roll_equiv barely moved ({roll_equiv:+.4f}), expected >= +0.3.")
    check(abs(pitch_equiv) < 0.1,
          f"ABDUCTION_LEFT: pitch_equiv={pitch_equiv:+.4f}, expected ~0 (exact by "
          f"construction -- it's the other calibration pose).")
    check(rel[1] >= 0.1,
          f"ABDUCTION_LEFT: MuJoCo left component only {rel[1]:+.4f}m, expected >= +0.1m "
          f"(positive roll should swing the wrist to the wearer's own left).")

    # --- ADDUCTION_RIGHT: roll changes the OPPOSITE way from abduction.
    # NOT a calibration pose -- and its real azimuth (SESSION_LOG.md 2026-09-04)
    # leans toward FORWARD_RAISE's direction rather than being ABDUCTION_
    # LEFT's clean opposite, so only a modest negative roll is expected
    # here, not a large one (an earlier version of this test asserted
    # <=-0.15, tuned against the OLD ComplementaryFilter decode -- the new
    # oblique-basis roll_equiv for this real pose is a real, smaller
    # ~-0.08, not a regression). ---
    pitch_equiv, roll_equiv, _ = decoded["ADDUCTION_RIGHT"]
    pitch_ctrl, roll_ctrl, _ = to_ctrl(decoded, "ADDUCTION_RIGHT", rest_elbow)
    rel = mujoco_wrist_position(pitch_ctrl, roll_ctrl)
    print(f"ADDUCTION_RIGHT pitch_equiv={pitch_equiv:+.4f} roll_equiv={roll_equiv:+.4f}  "
          f"ctrl: pitch={pitch_ctrl:+.4f} roll={roll_ctrl:+.4f}  |  "
          f"mujoco (front,left,up)=({rel[0]:+.4f},{rel[1]:+.4f},{rel[2]:+.4f})m")
    check(roll_equiv < 0.0,
          f"ADDUCTION_RIGHT: roll_equiv={roll_equiv:+.4f}, expected clearly negative "
          f"(opposite direction from ABDUCTION_LEFT), even if only modestly so.")
    check(rel[1] <= -0.02,
          f"ADDUCTION_RIGHT: MuJoCo left component={rel[1]:+.4f}m, expected clearly negative "
          f"(toward the wearer's right).")

    # --- ELBOW_FLEXION: elbow angle up, shoulder ~unchanged, MuJoCo wrist pulls up/in ---
    pitch_equiv, roll_equiv, elbow = decoded["ELBOW_FLEXION"]
    pitch_ctrl, roll_ctrl, elbow_ctrl = to_ctrl(decoded, "ELBOW_FLEXION", rest_elbow)
    rel = mujoco_wrist_position(pitch_ctrl, roll_ctrl, elbow_ctrl)
    print(f"ELBOW_FLEXION   pitch_equiv={pitch_equiv:+.4f} roll_equiv={roll_equiv:+.4f}  "
          f"ctrl: pitch={pitch_ctrl:+.4f} roll={roll_ctrl:+.4f} elbow={elbow_ctrl:+.4f}  |  "
          f"mujoco (front,left,up)=({rel[0]:+.4f},{rel[1]:+.4f},{rel[2]:+.4f})m")
    check(elbow - rest_elbow >= 0.5,
          f"ELBOW_FLEXION: decoded elbow angle barely changed ({rest_elbow:+.4f} -> {elbow:+.4f}), "
          f"expected >= +0.5 (real flexion is a ~110deg swing in the dot-product angle).")
    # 0.3rad margin: the real shoulder drift here is ~16deg (0.28rad) raw
    # tilt (a real person's upper arm isn't perfectly still while flexing
    # the elbow) -- oblique_decompose_scaled bounds pitch_equiv/roll_equiv's
    # combined magnitude to that same real tilt (SESSION_LOG.md 2026-09-04; the
    # unscaled oblique_decompose this replaced let it overshoot to ~35deg,
    # which this check would NOT have passed).
    check(abs(pitch_equiv) < 0.3 and abs(roll_equiv) < 0.3,
          f"ELBOW_FLEXION: shoulder moved (pitch_equiv={pitch_equiv:+.4f}, roll_equiv={roll_equiv:+.4f}) "
          f"more than a real ~16deg drift should, during a fixture meant to hold the upper arm still.")
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
