"""Phase 2 / Stage 6: drive the same MuJoCo whole-arm + hand simulation as
run_demo.py, but from REAL hardware instead of a CSV replay -- the actual
point of this project, not the CSV-replay prototype. Reads decoded
grip/shoulder/elbow state live over USART2 from the STM32 (via the CP2102
USB-to-TTL adapter), streamed by firmware/src/phase3_control_loop_main.cpp's
Stage 6 dual-MPU6050 + EMG loop.

A separate script from run_demo.py, not a --serial flag on it: a live
serial port and a CSV-replaying subprocess have different lifecycle/error
semantics (no "finished" sentinel here -- the stream just keeps going until
you stop it), and this keeps the already-working CSV-replay prototype
untouched. The MuJoCo model/actuator wiring, LatestSample class, and
LINE_RE regex are shared verbatim with run_demo.py, since the firmware's
output line format matches the CSV-replay binary's exactly on purpose.

IMPORTANT baud mismatch vs. other tools in this repo: tools/watch_myoware_uart.py
defaults to 9600 baud (matches earlier, simpler firmware stages). This
script defaults to 115200 baud, matching phase3_control_loop_main.cpp's
Stage 6 USART2 config (bumped from 9600 specifically to sustain 100Hz of the
new, longer dual-IMU output line -- see that file's usart2_init() comment).
Don't reuse watch_myoware_uart.py's baud default here, and don't reuse this
script's baud default against older/simpler firmware stages.

Usage:
    python3 -m pip install -r tools/mujoco_bridge/requirements.txt
    mjpython tools/mujoco_bridge/run_demo_live.py
    mjpython tools/mujoco_bridge/run_demo_live.py --port /dev/tty.usbserial-0001 --baud 115200

Must run as `mjpython`, not plain `python3` -- launch_passive raises
RuntimeError under plain CPython on macOS.
"""

import argparse
import json
import math
import re
import sys
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import serial

REPO_ROOT = Path(__file__).resolve().parents[2]
SCENE_XML = REPO_ROOT / "tools" / "mujoco_bridge" / "arm_hand_scene.xml"

DEFAULT_PORT = "/dev/tty.usbserial-0001"
DEFAULT_BAUD = 115200

# Same mapping/scale as run_demo.py -- see that file for the empirical
# tuning notes (palm-down quat fix, joint damping, GRIP_SCALE).
#
# 2026-09-02: retargeted from the shadow_hand-based rig's 10 finger
# actuators (4 fingers x 2 joints + thumb x2) to unitree_g1's simpler
# 7-DOF hand (thumb x3, index x2, middle x2, no ring/pinky) after the
# whole arm+hand was swapped from a hand-tuned shadow_hand weld to G1's
# real, vetted arm -- see arm_hand_scene.xml's top comment for why.
# thumb_0 (opposition/abduction) is deliberately left out here, same as
# the old rig's untouched abduction/thumb-base actuators -- it's not a
# curl joint, driving it by the same grip scalar as the others would
# rotate the thumb sideways instead of closing it.
#
# Target signs picked from each joint's own range direction (extracted
# from unitree_g1/g1_with_hands.xml, not guessed): middle/index ranges are
# entirely non-positive (0 = open, negative = closed) on this LEFT hand --
# mirrored from the source model's right hand, where the same joints are
# entirely non-negative -- so their targets are negative here. thumb_2's
# range is entirely non-negative (0 to +1.74533), so positive. thumb_1's
# range straddles zero (-0.724312 to 1.0472); its sign was picked to curl
# the same rotational direction as thumb_2 in the kinematic chain and
# confirmed by rendering the full 6-joint grip pose, not assumed -- see
# the scratch render this change was verified against.
# 2026-09-05: raw accelerometer noise was being reflected frame-by-frame
# straight into MuJoCo's ctrl every single physics tick, with no smoothing
# anywhere in the live tracking loop. First fix tried was an EMA low-pass
# (CTRL_SMOOTHING_ALPHA blend) -- reduced but didn't fix visible
# shakiness. Root cause found by the user directly (raw-data-grounded, not
# guessed): the arm hanging at rest with no muscle tension showed NO shake
# at all, but flicking the sensor's wire immediately produced visible
# shaking -- i.e. this isn't hand tremor, it's a loose/marginal physical
# connection glitching the raw I2C reads whenever the wire is disturbed
# (same fault class as the earlier frozen-elbow wiring bug in this
# project). An EMA still lets a single large glitch sample move the output
# by alpha*glitch in one tick; a hard rate limit is a stronger guarantee
# regardless of what's causing an implausible jump (bad reading OR genuine
# fast motion) -- caps how far ctrl can move per tick outright, per the
# user's explicit ask ("regardless of the cause, don't let MuJoCo's motion
# be too steep"). Doesn't fix bad data at the source -- the wiring itself
# should still get reseated/resecured -- but bounds how it looks either way.
MAX_CTRL_RATE_RAD_PER_SEC = 6.0

# 2026-09-05: the rate limiter above bounds worst-case per-tick jumps
# (good for occasional large glitches, e.g. the wire-flick spike), but
# doesn't reduce continuous small jitter every tick the way a low-pass
# does -- if the raw signal wobbles a little on EVERY sample, the rate
# limiter just tracks that wobble at its capped speed instead of
# smoothing it away. Re-adding an EMA, but this time on the RAW sensor
# signal (shoulder accel + decoded elbow angle) BEFORE it goes through
# oblique_decompose_scaled/the ctrl mapping, not on the already-mapped
# ctrl output like the first attempt -- filtering closer to the actual
# noise source. Runs alongside the rate limiter, not instead of it: they
# guard against two different kinds of bad signal.
RAW_SMOOTHING_ALPHA = 0.03

# 2026-09-07: was 0.6, changed after test_grasp_coverage.py's long-settle
# sweep found NO uniform-curl grip_scale is a permanently stable
# equilibrium on this object -- gravity + tiny contact-solver drift
# eventually wins at every value tested, sooner or later. 0.6 itself
# turned out to only hold for 6-10s before slipping (a too-short 1s
# post-lift settle in the original test made it look stably held -- see
# grasp_test_common.py's run_grasp_scenario). The user confirmed the real
# 6-step task only needs the object held for ~5-10s at a time (reach,
# carry to a point, release), not indefinitely, so the real question was
# "which value holds longest within a realistic task window," not "which
# value is eternally stable" (nothing is). A fine sweep at that longer,
# realistic settle time found grip_scale in [0.54,0.59] all held through
# ~11s and mostly through ~16s; 0.57 (this value) sits in the middle of
# that range with margin on both sides, and held through ~16s before
# failing around ~21s in the real test -- roughly double the ~10s the
# task actually needs. Kept in sync by hand with grasp_test_common.py's
# copy (which test_grip_kinematics.py also mirrors).
GRIP_SCALE = 0.57
GRIP_ACTUATORS = {
    "left_hand_thumb_1_joint": 1.0472,
    "left_hand_thumb_2_joint": 1.74533,
    "left_hand_middle_0_joint": -1.5708,
    "left_hand_middle_1_joint": -1.74533,
    "left_hand_index_0_joint": -1.5708,
    "left_hand_index_1_joint": -1.74533,
}

WRIST_ROLL_ACTUATOR = "left_wrist_roll_joint"
WRIST_PITCH_ACTUATOR = "left_wrist_pitch_joint"
WRIST_YAW_ACTUATOR = "left_wrist_yaw_joint"
SHOULDER_YAW_ACTUATOR = "left_shoulder_yaw_joint"

SHOULDER_PITCH_ACTUATOR = "left_shoulder_pitch_joint"
# 2026-09-03 CORRECTED: the 2026-09-02 comment here (and the ctrl
# assignment below) had the sign backwards. It claimed "positive pitch is
# flexion, confirmed by rendering" -- that rendering check was never
# actually re-run after the joint's own mechanical range in
# arm_hand_scene.xml turned out asymmetric (-3.0892 to +2.6704, not
# symmetric like the number that was eyeballed). Verified properly this
# time with tools/mujoco_bridge/test_arm_kinematics.py's geometry_summary()
# printed wrist-vs-shoulder numbers (a clean mj_forward check, not eyes on
# the viewer): ctrl=-1.0472 puts the wrist IN FRONT of the shoulder
# (front=+0.33m), ctrl=+2.6704 puts it BEHIND (front=-0.21m). So in MuJoCo's
# own joint frame, NEGATIVE ctrl is flexion (forward) and POSITIVE ctrl is
# extension (backward) -- the opposite of the old comment.
#
# This file's `shoulder_pitch` value (the data layer, IMU-derived) is kept
# meaning what it always meant -- positive = flexion/forward -- so the fix
# is entirely on the Data->MuJoCo mapping: negate it before clamping (see
# the ctrl assignment below), and swap which of the joint's two asymmetric
# mechanical limits acts as which anatomical cap. Real ROM is still the
# basis (AAOS/standard goniometry: ~0-180deg forward flexion, ~0-60deg
# backward extension from arm-at-side): flexion (now the ctrl-negative
# side) is capped at the joint's own forward ceiling, -3.0892rad (~177deg,
# short of the full 180deg a real shoulder can flex to, but that's G1's
# hardware limit); extension (now the ctrl-positive side) is capped at
# +1.0472rad (60deg), the real extension ROM -- well inside this joint's
# +2.6704rad backward ceiling, so the real anatomy is the binding limit
# there, not the hardware.
SHOULDER_PITCH_RANGE = (-3.0892, 1.0472)

# 2026-09-05, reverted: the constrained-envelope task's calibration
# baseline briefly moved from "arm hangs at side" to "arm straight
# forward" (which needed a -1.4784rad offset here, since ctrl=0 is fixed
# by this MJCF model's own joint definition as hang-down and doesn't move
# just because the real-world reference did). Baseline has since moved
# BACK to hang-down -- real hardware data (log_raw_imu.py's twist-check
# capture) showed that a pure horizontal left/right shoulder sweep barely
# changes the raw accelerometer reading at all (rotating about an axis too
# close to gravity-parallel to be observable by a single accelerometer),
# which made the old forward-reach baseline's LEFT_A calibration pose
# nearly degenerate. Adding a deliberate thumb-up/thumb-down twist to the
# LEFT/RIGHT reach fixed that (real Delta ay ~0.4g on the shoulder sensor,
# well above noise), but that twist is easiest to execute cleanly starting
# from a hang-down arm, not a forward-reaching one -- hence reverting the
# baseline. ctrl=0 (hang-down) now matches BASELINE again with no offset
# needed, same as before the 2026-09-05 forward-reach pivot.

SHOULDER_ROLL_ACTUATOR = "left_shoulder_roll_joint"
# 2026-09-02: widened from (-0.5, 0.8), same reasoning as
# SHOULDER_PITCH_RANGE above. Positive roll is abduction (confirmed by
# rendering). Real adduction ROM (arm sweeping back down past the side) is
# only about 0-50deg -- -0.8727rad here covers that. Real abduction ROM is
# 0-180deg, but this joint's own mechanical limit is +2.2515rad (~129deg,
# left_shoulder_roll_joint's range) -- again the robot's real hardware
# ceiling, not a clamp choice.
SHOULDER_ROLL_RANGE = (-0.8727, 2.2515)

ELBOW_ACTUATOR = "left_elbow_joint"
# 2026-09-02, corrected after visual comparison against the frozen right
# arm (which the user pointed out clearly hangs straight, unlike the left
# one): -1.0472 is NOT "arm straight" -- that was determined from the
# RAW ANGLE BETWEEN the upper-arm and forearm body vectors alone, which
# turned out to be the wrong metric. It ignores that those two "segment"
# reference bodies (left_shoulder_roll_link, left_elbow_link) both sit
# behind an extra shoulder_yaw_link offset that isn't part of a real
# anatomical upper-arm bone, so a locally-large angle between them doesn't
# mean the WHOLE arm points straight down -- confirmed the hard way:
# elbow=-1.0472 with shoulder_pitch=shoulder_roll=0 actually renders as the
# forearm swung up near the shoulder (looks like a mid-flexion "reaching"
# pose, not a hang), while elbow=+1.28 -- the exact value G1's own
# g1_with_hands.xml "stand" keyframe uses for its (frozen, undriven) right
# arm -- renders as a natural hang, symmetric with that right arm. Trusting
# the vendor's own resting value here instead of a self-derived numeric
# search, which also turned out unreliable near this range (a folded
# forearm brings the wrist close enough to the shoulder that its straight-
# line direction stops meaningfully indicating "hanging" at all).
#
# Flexion now runs the OPPOSITE direction from before: DOWN from +1.28
# (straight) toward -1.0472 (this joint's mechanical limit) is what swings
# the forearm up toward the shoulder -- confirmed by the same visual
# comparison (elbow=-1.0472 alone visibly looks like a mid-flexion reach).
# That span is 1.28 - (-1.0472) = 2.327rad (~133deg), close to a real
# elbow's ~140deg (2.44rad) full flexion range.
ELBOW_OFFSET = 1.28
ELBOW_RANGE = (-1.0472, 1.28)

# Identical to run_demo.py's LINE_RE -- the firmware's Stage 6 output line
# format is deliberately matched to the CSV-replay binary's, so this same
# pattern parses either source.
LINE_RE = re.compile(
    r"tick=(?P<tick>\d+) grip=(?P<grip>[-\d.eE+]+) gripping=(?P<gripping>\d) "
    r"shoulder_pitch=(?P<shoulder_pitch>[-\d.eE+]+) shoulder_roll=(?P<shoulder_roll>[-\d.eE+]+) "
    r"elbow=(?P<elbow>[-\d.eE+]+)"
)

# shoulder_raw_ax/ay/az: the UNMAPPED accelerometer axes, present later on the
# same tick line (phase3_control_loop_main.cpp always sends them, not just
# diagnostically -- see that file's 2026-09-01 comment). A separate regex
# rather than folding into LINE_RE above: LINE_RE only needs to match the
# leading fields to parse either source (this file's docstring), and these
# three are only meaningful for shoulder-axis-remap debugging, not normal
# operation -- added 2026-09-03 after a live test showed a forward-raise
# landing almost entirely on shoulder_roll instead of shoulder_pitch, to
# let the CURRENT physical mount's raw axes be measured directly (the
# firmware's own prescribed fix method) instead of guessing at a new remap.
#
# shoulder_raw_gx/gy/gz added 2026-09-05: a live debugging session needed
# the actual raw gyro (angular velocity), not just accel, to tell "the arm
# really moved a lot" apart from "the algorithm mis-split a small motion"
# when only looking at this script's own post-decode pitch/roll numbers
# wasn't convincing enough on its own.
SHOULDER_RAW_RE = re.compile(
    r"shoulder_raw_ax=(?P<shoulder_raw_ax>[-\d.eE+]+) "
    r"shoulder_raw_ay=(?P<shoulder_raw_ay>[-\d.eE+]+) "
    r"shoulder_raw_az=(?P<shoulder_raw_az>[-\d.eE+]+) "
    r"shoulder_raw_gx=(?P<shoulder_raw_gx>[-\d.eE+]+) "
    r"shoulder_raw_gy=(?P<shoulder_raw_gy>[-\d.eE+]+) "
    r"shoulder_raw_gz=(?P<shoulder_raw_gz>[-\d.eE+]+)"
)

# elbow_raw_ax/ay/az -- always sent (not just diagnostic), same as
# SHOULDER_RAW_RE's fields; added 2026-09-03 alongside the shoulder ones so a
# single capture session can record ground-truth raw vectors for BOTH
# sensors at once (needed for tools/mujoco_bridge/test_imu_to_mujoco.py's
# elbow-flexion fixtures, which need real elbow_raw data the same way its
# shoulder fixtures needed real shoulder_raw data).
ELBOW_RAW_RE = re.compile(
    r"elbow_raw_ax=(?P<elbow_raw_ax>[-\d.eE+]+) "
    r"elbow_raw_ay=(?P<elbow_raw_ay>[-\d.eE+]+) "
    r"elbow_raw_az=(?P<elbow_raw_az>[-\d.eE+]+)"
)

# The firmware's once-a-second diagnostic line (phase3_control_loop_main.cpp,
# tick_count % 1000 block) -- shoulder_completions/elbow_completions count
# real successful I2C reads (STOP reached after a full 14-byte transfer) in
# the preceding ~1s window, then reset to 0. This is a true bus-level
# liveness signal (an actual NACK/timeout, not a value threshold): a
# genuinely disconnected IMU reports exactly 0 completions every window,
# regardless of what its last-known decoded angle happens to read. Only
# present on the live serial stream, not the CSV-replay binary's output.
#
# nacks/timeouts (added 2026-08-23) split *why* completions is 0 for a given
# reader: nacks means that reader's own address got a clean "no ACK" (a real
# per-device signal -- that specific chip didn't answer); timeouts means the
# whole I2C1 peripheral wedged BUSY while THIS reader happened to be active,
# which can happen regardless of which physical device is actually at fault.
# Added to chase down an observed asymmetry: unplugging IMU#2 (elbow) also
# knocked IMU#1 (shoulder) offline, which plain completions==0 can't explain.
# Root cause found (2026-08-23): only VIN/GND were unplugged, not SDA/SCL --
# the now-unpowered MPU6050 stayed wired to the live bus and froze holding
# SDA low mid-transaction, wedging both readers. bus_recovery_attempts/freed
# (cumulative since boot) confirm the firmware's I2C bus-clear routine
# (phase3_control_loop_main.cpp's i2c1_bus_recovery(), UM10204 sec 3.1.16) is
# actually kicking in and un-sticking the bus, not just SWRST spinning.
DIAG_LINE_RE = re.compile(
    r"diag shoulder_completions=(?P<shoulder_completions>\d+) "
    r"elbow_completions=(?P<elbow_completions>\d+) "
    r"shoulder_nacks=(?P<shoulder_nacks>\d+) shoulder_timeouts=(?P<shoulder_timeouts>\d+) "
    r"elbow_nacks=(?P<elbow_nacks>\d+) elbow_timeouts=(?P<elbow_timeouts>\d+) "
    r"active_reader_state=(?P<active_reader_state>\d+) "
    r"bus_recovery_attempts=(?P<bus_recovery_attempts>\d+) "
    r"bus_recovery_freed=(?P<bus_recovery_freed>\d+)"
)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def ema_step(current, target, alpha):
    """One exponential-moving-average update: blends `alpha` of the way
    from `current` toward `target`. Used for RAW_SMOOTHING_ALPHA (raw
    shoulder accel + decoded elbow angle, smoothing continuous per-tick
    noise before it reaches the ctrl mapping) -- extracted 2026-09-05 as
    its own function so it has something a unit test can call directly,
    instead of only existing as an inline expression inside main()'s loop."""
    return current + alpha * (target - current)


def rate_limit_step(current, target, max_step):
    """Steps `current` toward `target` by at most `max_step`, never
    overshooting -- lands exactly on `target` if it's already within one
    step. Used for MAX_CTRL_RATE_RAD_PER_SEC (hard-caps how far ctrl can
    move in one tick, regardless of why the target jumped). Extracted
    2026-09-05, same reasoning as ema_step above."""
    return current + clamp(target - current, -max_step, max_step)


def calibration_tilt_deg(ref_raw, raw):
    """Real tilt (degrees) between a calibration candidate reading and the
    REST/BASELINE reference it's measured against -- the pure decision
    input behind _calibrate_pose()'s MIN_CALIBRATION_TILT_DEG retry guard,
    pulled out so that guard's logic can be unit tested without also
    needing to mock input()/serial capture."""
    ref_unit = _normalize3(ref_raw)
    raw_unit = _normalize3(raw)
    return math.degrees(math.acos(clamp(_dot3(ref_unit, raw_unit), -1.0, 1.0)))


# Pure-Python port of include/edgeneuro/fusion/tilt_azimuth.hpp's
# make_oblique_basis()/oblique_decompose() -- keep any change to the math in
# both places in sync. Replaces this file's old approach (subtract a single
# zero-pose reading from firmware's ComplementaryFilter-decoded
# shoulder_pitch/shoulder_roll, see the removed wrap_angle_delta and its
# comment in git history) because that decode has two independent real
# problems found 2026-09-03/04 (PRD.md Session Handoff): the gyro
# integration it depends on drifts multiple radians with zero corresponding
# accelerometer change, and its accel-only formula folds back past +-90deg
# so two genuinely different poses (e.g. forward-raise vs backward-
# extension) can decode to the same value. tilt_azimuth.hpp's approach
# fixes both by working from the raw accelerometer vector directly (no
# gyro) and centering on the real REST reading (no fixed-formula fold-
# back) -- but the further step of splitting that into two independent
# pitch/roll-like numbers can't assume FORWARD_RAISE and ABDUCTION_LEFT are
# perpendicular (real capture, 2026-09-04: only ~29deg apart, confirmed
# reproducible, not measurement noise -- see PRD.md), hence solving against
# the two REAL calibration directions (oblique, not assumed-orthogonal)
# below instead of a fixed cos/sin split.
def _normalize3(v):
    mag = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    return (v[0] / mag, v[1] / mag, v[2] / mag)


def _dot3(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _project_onto_tangent_plane(v, ref):
    d = _dot3(v, ref)
    return (v[0] - d * ref[0], v[1] - d * ref[1], v[2] - d * ref[2])


def make_oblique_basis(ref_raw, fwd_raw, abd_raw):
    """ref_raw/fwd_raw/abd_raw: raw (not-yet-normalized) calibration
    readings for REST/FORWARD_RAISE/ABDUCTION_LEFT. Returns a dict consumed
    by oblique_decompose(), plus the two calibration poses' own tilt angles
    (radians, via acos) so the caller can scale the dimensionless fwd/abd
    coefficients back into real angle-equivalents."""
    ref = _normalize3(ref_raw)
    fwd = _normalize3(fwd_raw)
    abd = _normalize3(abd_raw)
    pf = _project_onto_tangent_plane(fwd, ref)
    pa = _project_onto_tangent_plane(abd, ref)
    a11 = _dot3(pf, pf)
    a12 = _dot3(pf, pa)
    a22 = _dot3(pa, pa)
    det = a11 * a22 - a12 * a12
    tilt_fwd = math.acos(clamp(_dot3(ref, fwd), -1.0, 1.0))
    tilt_abd = math.acos(clamp(_dot3(ref, abd), -1.0, 1.0))
    return {
        "ref": ref, "pf": pf, "pa": pa,
        "a11": a11, "a12": a12, "a22": a22, "inv_det": 1.0 / det,
        "tilt_fwd": tilt_fwd, "tilt_abd": tilt_abd,
    }


def oblique_decompose(basis, accel_raw):
    """accel_raw: raw (not-yet-normalized) live accelerometer reading.
    Returns (fwd, abd) coefficients -- 1.0 means "exactly as far in that
    direction as its calibration reading", 0.0 means "at REST"."""
    accel = _normalize3(accel_raw)
    p = _project_onto_tangent_plane(accel, basis["ref"])
    b1 = _dot3(p, basis["pf"])
    b2 = _dot3(p, basis["pa"])
    fwd = (b1 * basis["a22"] - b2 * basis["a12"]) * basis["inv_det"]
    abd = (b2 * basis["a11"] - b1 * basis["a12"]) * basis["inv_det"]
    return fwd, abd


def oblique_decompose_scaled(basis, accel_raw):
    """Fixes a real overshoot in oblique_decompose()'s "coefficient times
    its calibration pose's own tilt" scaling: exact AT that pose, but for
    an off-axis reading it can badly overshoot the real angle (found on
    real hardware 2026-09-05: pitch_equiv/roll_equiv growing past +-3rad --
    impossible for a real shoulder -- during ordinary live tracking; also
    found earlier, offline, in a real ~16deg ELBOW_FLEXION drift that blew
    up to ~35deg -- same bug, same fix, this port just didn't exist yet
    when the C++ side (tilt_azimuth.hpp) got it first). Keeps the oblique
    decomposition for DIRECTION only, and substitutes the independently-
    measured real tilt (acos-based, physically exact, can't exceed pi) for
    MAGNITUDE, so total output magnitude can never exceed the real tilt."""
    accel = _normalize3(accel_raw)
    dot_ref = clamp(_dot3(accel, basis["ref"]), -1.0, 1.0)
    tilt = math.acos(dot_ref)
    fwd, abd = oblique_decompose(basis, accel_raw)
    mag = math.hypot(fwd, abd)
    if mag < 1e-6:
        return 0.0, 0.0
    return tilt * fwd / mag, tilt * abd / mag


class Tee:
    """Mirrors writes to multiple streams -- used to send every existing
    print() call (unchanged at each call site) to both the terminal and a
    log file, since debugging sessions on this project routinely produce
    more scrollback than fits in a chat message (hit the 50,000-character
    truncation limit during the 2026-08-23 IMU recovery investigation)."""

    def __init__(self, *streams):
        self._streams = streams

    def write(self, data):
        for s in self._streams:
            s.write(data)
            s.flush()

    def flush(self):
        for s in self._streams:
            s.flush()


# How long without a parsed line before the viewer loop treats the stream as
# stale and warns -- deliberately much larger than the firmware's own 100Hz
# (10ms) output interval, so normal jitter never trips it, but small enough
# to notice a real problem (e.g. the I2C bus getting stuck again, which this
# project has hit repeatedly -- see PRD.md Stage 6) within about half a
# second instead of only when you happen to notice the arm isn't moving.
STALE_AFTER_SECONDS = 0.5

# The periodic [SENSORS] block and [DIAG] line print only when something in them
# actually changed (like the existing [STALE]/[OK] messages already do), plus
# at least this often regardless -- so a long silent stretch reads as
# "confirmed still healthy" rather than being ambiguous with "the script
# itself hung". 2026-08-23: before this, both printed unconditionally every
# ~200 steps / ~1s and produced pages of identical repeated lines with the
# real transitions buried in them.
STATUS_HEARTBEAT_SECONDS = 30.0


class LatestSample:
    def __init__(self):
        self._lock = threading.Lock()
        self.grip = 0.0
        self.shoulder_pitch = 0.0
        self.shoulder_roll = 0.0
        self.elbow = 0.0
        # Unmapped shoulder accelerometer axes -- see SHOULDER_RAW_RE's
        # comment. None until the first line carrying them arrives.
        self.shoulder_raw_ax = None
        self.shoulder_raw_ay = None
        self.shoulder_raw_az = None
        self.shoulder_raw_gx = None
        self.shoulder_raw_gy = None
        self.shoulder_raw_gz = None
        self.elbow_raw_ax = None
        self.elbow_raw_ay = None
        self.elbow_raw_az = None
        self.last_update_monotonic = time.monotonic()
        self.has_received_data = False  # only True once update() has actually run at least once
        self.port_error = None  # set by the reader thread on a real port-level failure
        # Completions from the firmware's once-a-second diag line -- None
        # until the first one arrives (~1s after boot).
        self.shoulder_completions = None
        self.elbow_completions = None
        # Accumulated nacks+timeouts since each reader last had a window with
        # completions>0 -- resets to 0 the instant it comes back online, so
        # this is "how many retries since it went offline", not "since boot".
        self.shoulder_retry_attempts = 0
        self.elbow_retry_attempts = 0

    def update(self, grip, shoulder_pitch, shoulder_roll, elbow):
        with self._lock:
            self.grip = grip
            self.shoulder_pitch = shoulder_pitch
            self.shoulder_roll = shoulder_roll
            self.elbow = elbow
            self.last_update_monotonic = time.monotonic()
            self.has_received_data = True

    def update_shoulder_raw(self, ax, ay, az):
        with self._lock:
            self.shoulder_raw_ax = ax
            self.shoulder_raw_ay = ay
            self.shoulder_raw_az = az

    def snapshot_shoulder_raw(self):
        with self._lock:
            return self.shoulder_raw_ax, self.shoulder_raw_ay, self.shoulder_raw_az

    def update_shoulder_raw_gyro(self, gx, gy, gz):
        with self._lock:
            self.shoulder_raw_gx = gx
            self.shoulder_raw_gy = gy
            self.shoulder_raw_gz = gz

    def snapshot_shoulder_raw_gyro(self):
        with self._lock:
            return self.shoulder_raw_gx, self.shoulder_raw_gy, self.shoulder_raw_gz

    def update_elbow_raw(self, ax, ay, az):
        with self._lock:
            self.elbow_raw_ax = ax
            self.elbow_raw_ay = ay
            self.elbow_raw_az = az

    def snapshot_elbow_raw(self):
        with self._lock:
            return self.elbow_raw_ax, self.elbow_raw_ay, self.elbow_raw_az

    def mark_port_error(self, message):
        with self._lock:
            self.port_error = message

    def update_diag(self, shoulder_completions, elbow_completions,
                     shoulder_nacks, shoulder_timeouts, elbow_nacks, elbow_timeouts):
        with self._lock:
            self.shoulder_completions = shoulder_completions
            self.elbow_completions = elbow_completions
            if shoulder_completions > 0:
                self.shoulder_retry_attempts = 0
            else:
                self.shoulder_retry_attempts += shoulder_nacks + shoulder_timeouts
            if elbow_completions > 0:
                self.elbow_retry_attempts = 0
            else:
                self.elbow_retry_attempts += elbow_nacks + elbow_timeouts

    def imu_retry_attempts(self):
        with self._lock:
            return self.shoulder_retry_attempts, self.elbow_retry_attempts

    def imu_liveness(self):
        """(shoulder_online, elbow_online), each True/False/None -- None
        means no diag line has arrived yet (~1s after boot). Based on the
        firmware's own completions counters, i.e. a real I2C ACK/completion
        result, not a threshold on decoded angle values: 0 completions in
        the preceding ~1s window means that reader genuinely never finished
        a transaction, regardless of what its last decoded angle reads."""
        with self._lock:
            if self.shoulder_completions is None:
                return None, None
            return self.shoulder_completions > 0, self.elbow_completions > 0

    def snapshot(self):
        with self._lock:
            return self.grip, self.shoulder_pitch, self.shoulder_roll, self.elbow

    def seconds_since_update(self):
        with self._lock:
            return time.monotonic() - self.last_update_monotonic

    def is_ready(self):
        with self._lock:
            return self.has_received_data

    def status(self):
        """(is_stale, port_error) -- port_error is a hard failure (the OS-level
        port itself broke, e.g. the CP2102 was unplugged); is_stale just means
        no valid line has arrived recently, which is what you see when the
        STM32 is still connected but its firmware has stopped producing output
        (e.g. stuck retrying a wedged I2C bus -- the USB-serial link itself
        stays up the whole time, so this looks identical to "the board is
        still there but has nothing to say" from the Python side, and can NOT
        be told apart from "the wire fell out of the sensor" without also
        checking the firmware/hardware directly, e.g. via SWD)."""
        with self._lock:
            port_error = self.port_error
            stale = (time.monotonic() - self.last_update_monotonic) > STALE_AFTER_SECONDS
        return stale, port_error


def split_lines(buf, chunk):
    """Appends `chunk` to `buf`, splits out every complete \\r\\n-terminated
    line, and returns (lines, remaining_buf). A chunk boundary landing
    mid-line is handled correctly -- the incomplete trailing line stays in
    the returned buffer for the next call instead of being dropped or
    decoded early, which is exactly what real serial reads do (pyserial's
    ser.read(256) has no reason to land on a line boundary). Decoding uses
    errors="ignore" (matches this function's original inline behavior) so
    a corrupted byte from a real wire glitch doesn't crash the reader.
    Extracted 2026-09-06 from reader_thread_main's inline buffer handling
    so this exact behavior has its own test, independent of needing a real
    or mocked serial port."""
    buf = buf + chunk
    lines = []
    while b"\r\n" in buf:
        raw, buf = buf.split(b"\r\n", 1)
        lines.append(raw.decode("utf-8", errors="ignore"))
    return lines, buf


def reader_thread_main(ser, latest):
    # Read raw bytes and split on the firmware's own \r\n line ending
    # rather than iterating the pyserial object directly -- matches
    # tools/watch_myoware_uart.py's established pattern for this project's
    # UART output, which is more robust to a mid-line disconnect/reconnect
    # than relying on pyserial's own line iteration.
    buf = b""
    # Edge-triggered like [STALE]/[OK] below: completions is
    # deliberately excluded from this key since it fluctuates by design
    # every healthy window (255, 254, 255, ...) and would defeat the point
    # of suppressing noise -- only the fields that are boring-when-zero
    # decide whether this line is worth reprinting.
    last_diag_key = None
    last_diag_print_time = 0.0
    try:
        while True:
            chunk = ser.read(256)
            if not chunk:
                continue
            lines, buf = split_lines(buf, chunk)
            for line in lines:
                match = LINE_RE.search(line)
                if match:
                    latest.update(
                        float(match.group("grip")),
                        float(match.group("shoulder_pitch")),
                        float(match.group("shoulder_roll")),
                        float(match.group("elbow")),
                    )
                    raw_match = SHOULDER_RAW_RE.search(line)
                    if raw_match:
                        latest.update_shoulder_raw(
                            float(raw_match.group("shoulder_raw_ax")),
                            float(raw_match.group("shoulder_raw_ay")),
                            float(raw_match.group("shoulder_raw_az")),
                        )
                        latest.update_shoulder_raw_gyro(
                            float(raw_match.group("shoulder_raw_gx")),
                            float(raw_match.group("shoulder_raw_gy")),
                            float(raw_match.group("shoulder_raw_gz")),
                        )
                    elbow_raw_match = ELBOW_RAW_RE.search(line)
                    if elbow_raw_match:
                        latest.update_elbow_raw(
                            float(elbow_raw_match.group("elbow_raw_ax")),
                            float(elbow_raw_match.group("elbow_raw_ay")),
                            float(elbow_raw_match.group("elbow_raw_az")),
                        )
                    continue
                diag_match = DIAG_LINE_RE.search(line)
                if diag_match:
                    latest.update_diag(
                        int(diag_match.group("shoulder_completions")),
                        int(diag_match.group("elbow_completions")),
                        int(diag_match.group("shoulder_nacks")),
                        int(diag_match.group("shoulder_timeouts")),
                        int(diag_match.group("elbow_nacks")),
                        int(diag_match.group("elbow_timeouts")),
                    )
                    diag_key = (
                        diag_match.group("shoulder_nacks"), diag_match.group("shoulder_timeouts"),
                        diag_match.group("elbow_nacks"), diag_match.group("elbow_timeouts"),
                        diag_match.group("bus_recovery_attempts"), diag_match.group("bus_recovery_freed"),
                    )
                    now = time.monotonic()
                    if diag_key != last_diag_key or (now - last_diag_print_time) >= STATUS_HEARTBEAT_SECONDS:
                        # Printed directly (not folded into the periodic status
                        # dict) so nack/timeout counts are visible in real time
                        # while physically unplugging a sensor to diagnose it.
                        print(
                            f"[DIAG] shoulder: completions={diag_match.group('shoulder_completions')} "
                            f"nacks={diag_match.group('shoulder_nacks')} "
                            f"timeouts={diag_match.group('shoulder_timeouts')} | "
                            f"elbow: completions={diag_match.group('elbow_completions')} "
                            f"nacks={diag_match.group('elbow_nacks')} "
                            f"timeouts={diag_match.group('elbow_timeouts')} | "
                            f"bus_recovery: attempts={diag_match.group('bus_recovery_attempts')} "
                            f"freed={diag_match.group('bus_recovery_freed')} (cumulative since boot)"
                        )
                        last_diag_key = diag_key
                        last_diag_print_time = now
                    continue
                # Neither LINE_RE nor DIAG_LINE_RE matched -- previously
                # silently dropped, which hid real firmware output (e.g. the
                # boot-time "Stage 5b: ..." banner, or a "wake write FAILED"
                # error right before blink_code()'s infinite halt loop in
                # phase3_control_loop_main.cpp) with nothing to show for it
                # but a script that looked hung. Printed here instead so a
                # firmware-side halt/error is visible instead of silent.
                if line.strip():
                    print(f"[FW] {line}")
    except serial.SerialException as exc:
        # A real OS-level port failure (e.g. the CP2102 adapter was
        # physically unplugged) -- distinct from the firmware just going
        # quiet, which doesn't raise anything here since the port itself
        # stays open. Surfaced through LatestSample rather than printed
        # directly, since this runs on a background thread.
        latest.mark_port_error(str(exc))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument(
        "--log-file", default=None,
        help="also append all terminal output to this file (tee-style), for sessions too "
             "long to paste in full",
    )
    parser.add_argument(
        "--calibration-file", type=Path,
        default=REPO_ROOT / "tools" / "mujoco_bridge" / "shoulder_calibration.json",
        help="where to save the interactive shoulder calibration (BASELINE/FORWARD/LEFT_TWIST/"
             "RIGHT_TWIST raw vectors + elbow zero) after capturing it, or load it from when "
             "--skip-calibration is passed.",
    )
    parser.add_argument(
        "--skip-calibration", action="store_true",
        help="skip the interactive BASELINE/FORWARD/LEFT_TWIST/RIGHT_TWIST prompts and load a "
             "previously-saved calibration from --calibration-file instead. Only valid if the IMU "
             "mount/strap hasn't changed since that calibration was captured -- the raw-vector-to-"
             "real-pose mapping is specific to how the sensor happens to be strapped on that "
             "session (see the calibration comment below), so re-run without this flag after "
             "re-mounting or re-strapping either sensor.",
    )
    parser.add_argument(
        "--optional-sensors", default="",
        help="comma-separated subset of {shoulder,elbow,emg} allowed to be absent this run "
             "without crashing (default: empty -- every sensor required, this script's original "
             "behavior). This is an opt-in relaxation, not real hardware detection: a sensor NOT "
             "listed here is still assumed present, and this script may still hang/crash if it "
             "actually isn't. 'shoulder' skips the FORWARD/LEFT_TWIST/RIGHT_TWIST calibration "
             "poses (there's nothing real to calibrate) and holds pitch/roll fixed at 0 "
             "(BASELINE/hang-down) for the whole session instead of live-tracking -- without this, "
             "an absent shoulder IMU's raw reading stays the zero vector the firmware initializes "
             "it to, and normalizing a zero vector during calibration is a real divide-by-zero, "
             "not a hang (found 2026-09-07, testing MyoWare with neither IMU connected). 'elbow' "
             "and 'emg' are accepted for a consistent vocabulary but need no special handling -- "
             "both already degrade safely to a constant if their channel is silent (elbow_bend's "
             "own shoulder_mag/elbow_mag>0.1 guard in firmware; GripStateMachine simply never "
             "crosses threshold) -- listing them here doesn't change this script's behavior.",
    )
    args = parser.parse_args()

    VALID_OPTIONAL_SENSORS = {"shoulder", "elbow", "emg"}
    optional_sensors = {s.strip() for s in args.optional_sensors.split(",") if s.strip()}
    unknown_sensors = optional_sensors - VALID_OPTIONAL_SENSORS
    if unknown_sensors:
        sys.exit(f"--optional-sensors: unknown name(s) {sorted(unknown_sensors)} -- "
                  f"choices are {sorted(VALID_OPTIONAL_SENSORS)}")
    shoulder_optional = "shoulder" in optional_sensors

    if not SCENE_XML.exists():
        sys.exit(f"arm+hand scene not found: {SCENE_XML}")

    if args.log_file:
        log_fh = open(args.log_file, "a", buffering=1)
        log_fh.write(f"\n--- session start {time.strftime('%Y-%m-%d %H:%M:%S')} ---\n")
        sys.stdout = Tee(sys.stdout, log_fh)

    ser = serial.Serial(args.port, args.baud, timeout=1)
    print(f"Listening on {args.port} @ {args.baud} baud -- Ctrl+C to stop")

    latest = LatestSample()
    reader = threading.Thread(target=reader_thread_main, args=(ser, latest), daemon=True)
    reader.start()

    # Shoulder calibration: 3 poses feed the oblique basis, each averaged
    # over a settle+hold window -- replaces the old single-pose "zero"
    # capture (git history) now that the shoulder is decoded via
    # oblique_decompose() above instead of subtracting a zero reference
    # from firmware's ComplementaryFilter output. Which 3 real poses fill
    # the 3 slots has changed over time (REST/FORWARD_RAISE/ABDUCTION_LEFT
    # for the original full-ROM task; briefly BASELINE/DOWN/LEFT_A for a
    # forward-reach-baseline constrained task; now BASELINE/FORWARD/
    # LEFT_TWIST -- see the calibration calls below for why) -- the
    # function itself doesn't care, only real distinct captured directions
    # matter. Elbow keeps the same zero-offset approach as before
    # (firmware's dot-product elbow_bend has neither of the problems that
    # drove this shoulder change), captured during the BASELINE hold
    # alongside the shoulder reading.
    #
    # Record for RECORD_SECONDS but only average the last SETTLE_TAIL_SECONDS
    # -- same as log_raw_imu.py's record_pose()/average_tail(), and for the
    # same reason: the 3/2/1 countdown ending is not the same instant as
    # "the arm has actually finished moving and settled", especially for a
    # bigger reach like FORWARD_RAISE. An earlier version of this function
    # averaged the WHOLE window starting immediately after the countdown
    # (no settle margin at all) -- confirmed too short on real hardware
    # (2026-09-05): the person was still mid-motion when averaging started,
    # producing calibration readings measurably smaller/less-separated than
    # log_raw_imu.py's own captures of the same poses.
    #
    # RECORD_SECONDS/SETTLE_TAIL_SECONDS raised again (was 4.0/1.5, kept in
    # sync with log_raw_imu.py's own constants -- see that file's comment):
    # real data showed even the 1.5s "settled" tail was still drifting
    # internally for a deliberately EXAGGERATED calibration pose (PURE_DOWN's
    # az moved another -0.14 comparing the tail's own first half to its
    # second half) -- a big effortful reach can keep settling well past 2.5s
    # in, not just during an initial "moving" phase.
    RECORD_SECONDS = 6.0
    SETTLE_TAIL_SECONDS = 2.5

    def _capture_window(seconds=RECORD_SECONDS, tail_seconds=SETTLE_TAIL_SECONDS):
        # Prints raw_shoulder every ~0.5s during the recording (added
        # 2026-09-05): a real debugging session had no way to tell "the arm
        # really didn't move much during this capture" apart from "it moved
        # plenty but got averaged/timed wrong" -- only the final settled
        # value was ever visible. This makes the actual trajectory visible
        # in the log itself, not just the end result.
        samples = []  # (t, raw_shoulder, elbow)
        t_start = time.monotonic()
        deadline = t_start + seconds
        last_print_t = -1.0
        while time.monotonic() < deadline:
            raw = latest.snapshot_shoulder_raw()
            if raw[0] is not None:
                t = time.monotonic() - t_start
                samples.append((t, raw, latest.snapshot()[3]))
                if t - last_print_t >= 0.5:
                    print(f"    t={t:4.1f}s raw_shoulder=({raw[0]:+.3f},{raw[1]:+.3f},{raw[2]:+.3f})")
                    last_print_t = t
            time.sleep(0.02)
        t_max = samples[-1][0]
        tail = [s for s in samples if s[0] >= t_max - tail_seconds]
        if not tail:
            tail = samples[-5:]
        n = len(tail)
        raw_avg = (
            sum(s[1][0] for s in tail) / n,
            sum(s[1][1] for s in tail) / n,
            sum(s[1][2] for s in tail) / n,
        )
        elbow_avg = sum(s[2] for s in tail) / n

        # Warns (2026-09-05) if the tail window itself still shows real
        # drift -- see RECORD_SECONDS's comment above for why a big
        # effortful pose can still be mid-settle this far in. Printed only,
        # not an automatic retry (unlike MIN_CALIBRATION_TILT_DEG's guard
        # below) -- read it and judge whether to redo with more hold time.
        if len(tail) >= 4:
            t_mid = (tail[0][0] + tail[-1][0]) / 2.0
            first_half = [s[1] for s in tail if s[0] < t_mid]
            second_half = [s[1] for s in tail if s[0] >= t_mid]
            if first_half and second_half:
                fh = [sum(v[i] for v in first_half) / len(first_half) for i in range(3)]
                sh = [sum(v[i] for v in second_half) / len(second_half) for i in range(3)]
                drift = max(abs(sh[i] - fh[i]) for i in range(3))
                if drift > 0.05:
                    print(f"  警告:這段錄製的『穩定期』內部,shoulder raw 還在漂移"
                          f"(前半段 vs 後半段最大差異 {drift:.3f}g)——這個姿勢可能還沒真的定住,"
                          f"考慮重錄、保持動作更久再結束。")

        return raw_avg, elbow_avg

    # Interactive per-pose calibration -- press Enter when actually in
    # position, then a visible 3/2/1 countdown before the capture window
    # starts, same pattern as log_raw_imu.py's proven interactive flow.
    # The earlier version of this calibration just printed an instruction
    # and slept a fixed 2s regardless of whether the person was actually
    # ready -- with no confirmation step, a slow transition (or a chat-
    # paced back-and-forth) meant the capture window could start before
    # the arm ever got there, silently baking a wrong reading into the
    # whole session's calibration basis.
    # min_tilt_deg/ref_raw: automatic retry guard against a too-small
    # calibration pose, added 2026-09-05 after DOWN/LEFT_A came out at
    # 1.8-9.2deg from BASELINE four separate real-hardware attempts in a
    # row despite the instruction already saying "exaggerate this" --
    # relying on the person to notice the printed tilt themselves and
    # manually redo clearly wasn't reliable enough. Loops the SAME prompt
    # instead of proceeding with an almost-degenerate basis (see the
    # comment above this function's call sites for why that basis
    # amplifies ordinary hand-tremor noise into large, unstable output).
    def _calibrate_pose(instruction, ref_raw=None, min_tilt_deg=None):
        while True:
            print(instruction)
            print("準備好後按 Enter。")
            input()
            print(f"3 秒後開始 -- 請保持住直到錄製結束(共 {RECORD_SECONDS:.0f} 秒,前段是移動時間,"
                  f"只有最後 {SETTLE_TAIL_SECONDS:.1f} 秒會拿來平均)。")
            for n in (3, 2, 1):
                print(f"  {n}...", flush=True)
                time.sleep(1.0)
            print("開始錄製!請維持姿勢。")
            result = _capture_window()
            if ref_raw is None or min_tilt_deg is None:
                return result
            raw, _ = result
            tilt_deg = calibration_tilt_deg(ref_raw, raw)
            if tilt_deg >= min_tilt_deg:
                print(f"  角度足夠(tilt={tilt_deg:.1f}deg >= {min_tilt_deg:.0f}deg),採用這次錄製。\n")
                return result
            print(f"  角度太小(tilt={tilt_deg:.1f}deg,需要 >= {min_tilt_deg:.0f}deg)——"
                  f"這個角度離基準點太近,校正基底會不穩定,請重來一次,這次動作要更誇張。\n")

    while not latest.is_ready():
        time.sleep(0.05)
    # 5s bound only when shoulder is marked optional -- with a firmware
    # build that still sends the full tick= line for an absent IMU (just
    # zero-valued fields, see phase3_control_loop_main.cpp's
    # kRequireShoulderImu), this wait would already pass quickly on its
    # own; the timeout is defensive for an older/different firmware build
    # that might not send shoulder_raw_* fields at all when the IMU never
    # completes a read. Required (not optional) sensors keep the original
    # unbounded wait -- if shoulder is genuinely required and never shows
    # up, that SHOULD hang here rather than silently proceed.
    _shoulder_wait_deadline = (time.time() + 5.0) if shoulder_optional else None
    while latest.snapshot_shoulder_raw()[0] is None:
        if _shoulder_wait_deadline is not None and time.time() > _shoulder_wait_deadline:
            print("警告:shoulder 標記為選配,5 秒內沒收到 shoulder_raw,略過等待——"
                  "肩膀 pitch/roll 整個 session 都會固定在 BASELINE。")
            break
        time.sleep(0.05)

    # 2026-09-05, second pass: calibration baseline moved back to "arm
    # hangs at side" (see SHOULDER_PITCH_FORWARD_OFFSET's comment above).
    # FORWARD takes the old fwd slot (pitch axis: hang-down <-> forward
    # reach). LEFT_TWIST takes the abd slot (roll axis) instead of the old
    # LEFT_A -- a real capture via log_raw_imu.py (raw_imu_twist_check.json)
    # showed a plain horizontal left/right sweep barely changes shoulder_raw
    # at all (rotating about an axis too close to gravity-parallel for a
    # single accelerometer to see), but adding a deliberate thumb-up twist
    # to the same reach produced a real, large, clearly separable signal
    # (shoulder ay: +0.471 with twist vs +0.055 without, for the same "arm
    # swung left" target -- a 0.42g difference, far above the 0.05g noise/
    # drift floor used elsewhere in this project). RIGHT_TWIST (mirror:
    # thumb-down) is captured too but NOT fed into make_oblique_basis() --
    # it's a held-out validation check instead, printed below, confirming
    # the basis built from LEFT_TWIST alone also correctly recognizes the
    # opposite-side motion.
    # Each step below explicitly says "回到 BASELINE 再做" -- see git
    # history (the DOWN/LEFT_A version of this comment) for why an implicit
    # "start from wherever you happen to be" produced an almost-degenerate
    # basis once; kept here even though FORWARD/LEFT_TWIST are big,
    # unambiguous motions less likely to suffer from it.
    # 20deg minimum: comfortably above real hand-tremor-scale noise
    # (measured elsewhere in this project at a few degrees) and still
    # well within a real shoulder's comfortable ROM in either direction.
    MIN_CALIBRATION_TILT_DEG = 20.0

    # --skip-calibration (2026-09-05): re-doing all 4 poses every single
    # run got tedious once the pipeline itself was already trusted -- load
    # a previously-saved capture instead of prompting. Only valid as long
    # as the physical strap/mount hasn't changed, since these are raw
    # vectors specific to that mounting (see the big comment above this
    # block) -- NOT re-verified against anything live, just trusted at
    # face value, so re-run without this flag after any re-mount/re-strap.
    if args.skip_calibration:
        if not args.calibration_file.exists():
            sys.exit(f"--skip-calibration passed but {args.calibration_file} doesn't exist -- "
                      f"run once without --skip-calibration first to create it.")
        with open(args.calibration_file) as f:
            saved = json.load(f)
        baseline_raw = tuple(saved["baseline_raw"])
        forward_raw = tuple(saved["forward_raw"])
        left_twist_raw = tuple(saved["left_twist_raw"])
        right_twist_raw = tuple(saved["right_twist_raw"])
        zero_elbow = saved["zero_elbow"]
        print(f"--skip-calibration: loaded shoulder/elbow calibration from {args.calibration_file} "
              f"(captured {saved.get('captured_at', 'unknown time')}) -- not re-verified against "
              f"the current mount, only trusted at face value.")
    else:
        baseline_raw, zero_elbow = _calibrate_pose(
            "Calibrating elbow zero (shoulder 標記為選配,肩膀姿勢不重要,只需要手肘打直): "
            "請把手肘打直、手臂放鬆下垂。"
            if shoulder_optional else
            "Calibrating shoulder -- BASELINE: 請把手臂自然垂下,手肘打直。")

        if shoulder_optional:
            forward_raw = left_twist_raw = right_twist_raw = None
            print("shoulder 標記為選配,跳過 FORWARD/LEFT_TWIST/RIGHT_TWIST 校正姿勢——"
                  "肩膀 pitch/roll 整個 session 都會固定在 BASELINE,不會即時追蹤。\n")
        else:
            forward_raw, _ = _calibrate_pose(
                "FORWARD: 先回到 BASELINE(垂下),然後手肘打直,手臂往前伸直到底,手腕不要轉。",
                ref_raw=baseline_raw, min_tilt_deg=MIN_CALIBRATION_TILT_DEG)

            left_twist_raw, _ = _calibrate_pose(
                "LEFT_TWIST: 先回到 BASELINE(垂下),然後手肘打直,手臂往左甩到底,"
                "同時大拇指轉朝上。",
                ref_raw=baseline_raw, min_tilt_deg=MIN_CALIBRATION_TILT_DEG)

            right_twist_raw, _ = _calibrate_pose(
                "RIGHT_TWIST(驗證用,不會進入校正基底): 先回到 BASELINE(垂下),然後手肘打直,"
                "手臂往右甩到底,同時大拇指轉朝下。",
                ref_raw=baseline_raw, min_tilt_deg=MIN_CALIBRATION_TILT_DEG)

        if shoulder_optional:
            print("shoulder 選配模式下不存校正檔(沒有真正的肩膀校正資料可存)。")
        else:
            args.calibration_file.parent.mkdir(parents=True, exist_ok=True)
            with open(args.calibration_file, "w") as f:
                json.dump({
                    "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "baseline_raw": baseline_raw,
                    "forward_raw": forward_raw,
                    "left_twist_raw": left_twist_raw,
                    "right_twist_raw": right_twist_raw,
                    "zero_elbow": zero_elbow,
                }, f, indent=2)
            print(f"Calibration saved to {args.calibration_file} -- next run can pass "
                  f"--skip-calibration to reuse it instead of re-prompting.")

    # shoulder_optional forces shoulder_basis=None unconditionally here,
    # even on the --skip-calibration path above (which would otherwise
    # have loaded real forward_raw/left_twist_raw from a past session's
    # file) -- what matters isn't how the basis was built, it's that the
    # LIVE shoulder_raw feeding oblique_decompose_scaled() every tick
    # would be the zero vector all session if the IMU is genuinely absent
    # now, and normalizing that is a real divide-by-zero regardless of a
    # perfectly valid basis. See MAX_CTRL_RATE_RAD_PER_SEC's neighborhood
    # in the main loop below for where pitch_equiv/roll_equiv fall back to
    # a flat 0.0 instead of calling oblique_decompose_scaled at all.
    if shoulder_optional:
        shoulder_basis = None
        print(f"Shoulder: 選配、未即時追蹤,pitch/roll 整個 session 固定為 0(BASELINE)。"
              f"elbow zero={zero_elbow:.3f}")
        forward_raw = left_twist_raw = right_twist_raw = None
    else:
        shoulder_basis = make_oblique_basis(baseline_raw, forward_raw, left_twist_raw)
    if shoulder_basis is not None:
        print(f"Shoulder calibrated: BASELINE=({baseline_raw[0]:+.3f},{baseline_raw[1]:+.3f},{baseline_raw[2]:+.3f}) "
              f"FORWARD=({forward_raw[0]:+.3f},{forward_raw[1]:+.3f},{forward_raw[2]:+.3f}) "
              f"[tilt={math.degrees(shoulder_basis['tilt_fwd']):.1f}deg] "
              f"LEFT_TWIST=({left_twist_raw[0]:+.3f},{left_twist_raw[1]:+.3f},{left_twist_raw[2]:+.3f}) "
              f"[tilt={math.degrees(shoulder_basis['tilt_abd']):.1f}deg]  elbow zero={zero_elbow:.3f}")

        # Validation-only check (2026-09-05): RIGHT_TWIST was never fed into
        # make_oblique_basis() above -- decode it through the resulting basis
        # anyway and print the result. Expect roll_equiv here to come out
        # negative (opposite side from LEFT_TWIST, which the basis defines as
        # positive) and reasonably close in magnitude to tilt_abd -- if it
        # doesn't, LEFT_TWIST/RIGHT_TWIST aren't as mirror-symmetric on this
        # body as the log_raw_imu.py capture suggested, and the basis may need
        # rebuilding from an average of both sides instead of LEFT_TWIST alone.
        _right_pitch_check, _right_roll_check = oblique_decompose_scaled(shoulder_basis, right_twist_raw)
        print(f"  (validation) RIGHT_TWIST decodes to pitch_equiv={_right_pitch_check:+.3f} "
              f"roll_equiv={_right_roll_check:+.3f} rad -- expect roll_equiv negative, "
              f"magnitude near {shoulder_basis['tilt_abd']:.3f} rad if left/right are symmetric")

    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)

    grip_actuator_ids = {
        name: model.actuator(name).id for name in GRIP_ACTUATORS
    }
    shoulder_pitch_id = model.actuator(SHOULDER_PITCH_ACTUATOR).id
    shoulder_roll_id = model.actuator(SHOULDER_ROLL_ACTUATOR).id
    elbow_id = model.actuator(ELBOW_ACTUATOR).id
    # For the periodic [CORR] line below -- lets a raw-sensor value and the
    # MuJoCo pose it produced be read off the same line instead of manually
    # lining up two separate logs by eye/timestamp.
    shoulder_body_id = model.body("left_shoulder_roll_link").id
    wrist_body_id = model.body("left_wrist_yaw_link").id

    data.ctrl[model.actuator(WRIST_ROLL_ACTUATOR).id] = 0.0
    data.ctrl[model.actuator(WRIST_PITCH_ACTUATOR).id] = 0.0
    data.ctrl[model.actuator(WRIST_YAW_ACTUATOR).id] = 0.0
    data.ctrl[model.actuator(SHOULDER_YAW_ACTUATOR).id] = 0.0

    was_stale = False  # edge-triggered: only print on stale<->healthy transitions, not every frame
    last_status_key = None  # edge-triggered the same way, for the [SENSORS] block below
    last_status_print_time = 0.0

    # Rate-limited ctrl state for the final ctrl values -- see
    # MAX_CTRL_RATE_RAD_PER_SEC's comment above. None until the first tick, so
    # the very first frame snaps straight to its target instead of easing
    # up from an arbitrary 0.0 start.
    smoothed_pitch_ctrl = None
    smoothed_roll_ctrl = None
    smoothed_elbow_ctrl = None

    # Raw-domain EMA state -- see RAW_SMOOTHING_ALPHA's comment. None until
    # the first tick, same reasoning as the rate-limited state above.
    smoothed_shoulder_raw = None
    smoothed_elbow_raw_scalar = None

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            step_count = 0
            while viewer.is_running():
                step_start = time.time()

                grip, old_shoulder_pitch, old_shoulder_roll, elbow = latest.snapshot()
                shoulder_raw = latest.snapshot_shoulder_raw()
                is_stale, port_error = latest.status()

                # Raw-domain EMA (see RAW_SMOOTHING_ALPHA's comment) --
                # smooths the shoulder accel vector and the decoded elbow
                # angle BEFORE either goes into oblique_decompose_scaled or
                # the ctrl mapping below, so continuous per-tick noise gets
                # averaged out instead of just rate-capped downstream.
                if smoothed_shoulder_raw is None:
                    smoothed_shoulder_raw = shoulder_raw
                    smoothed_elbow_raw_scalar = elbow
                else:
                    smoothed_shoulder_raw = tuple(
                        ema_step(smoothed_shoulder_raw[i], shoulder_raw[i], RAW_SMOOTHING_ALPHA)
                        for i in range(3)
                    )
                    smoothed_elbow_raw_scalar = ema_step(smoothed_elbow_raw_scalar, elbow, RAW_SMOOTHING_ALPHA)

                if port_error is not None:
                    sys.exit(f"\nserial port failed: {port_error}\n"
                             f"(the CP2102 adapter was likely unplugged -- this needs the script "
                             f"restarted after reconnecting, unlike a firmware-side stall)")

                if is_stale and not was_stale:
                    print(f"\n[STALE] no valid line from the STM32 in >{STALE_AFTER_SECONDS}s "
                          f"(last good data {latest.seconds_since_update():.1f}s ago) -- holding last pose. "
                          f"Port is still open, so this is the board/firmware going quiet (e.g. a stuck "
                          f"I2C bus, see PRD.md Stage 6), not a USB disconnect. No restart needed once "
                          f"it's fixed -- ImuReader's own SWRST recovery + this script should both "
                          f"resume on their own.")
                elif was_stale and not is_stale:
                    print(f"[OK] data flowing again after {latest.seconds_since_update():.1f}s gap")
                was_stale = is_stale

                for name, upper_range in GRIP_ACTUATORS.items():
                    data.ctrl[grip_actuator_ids[name]] = grip * GRIP_SCALE * upper_range
                # oblique_decompose_scaled gives a radian-equivalent whose
                # magnitude is bounded by the real measured tilt (see its
                # own docstring -- the plain "coefficient times the
                # calibration pose's own tilt" version overshot badly for
                # off-axis readings, confirmed on real hardware). No
                # wrap_angle_delta needed here (unlike the old
                # ComplementaryFilter-decoded values this replaces): this
                # is a linear projection + acos, no atan2 branch cut to
                # wrap around.
                # shoulder_raw can't still be None here: main() already
                # blocked until the first raw reading arrived, before
                # calibration, and LatestSample never resets it afterward.
                # shoulder_basis is None exactly when --optional-sensors
                # included "shoulder" -- smoothed_shoulder_raw would be the
                # zero vector all session in that case (the IMU never
                # completes a real read), and oblique_decompose_scaled
                # normalizes its input, so calling it would be a real
                # divide-by-zero, not just a meaningless result. Falls back
                # to flat 0.0 instead, matching the printed "fixed at
                # BASELINE" promise from the calibration step above.
                if shoulder_basis is not None:
                    pitch_equiv, roll_equiv = oblique_decompose_scaled(shoulder_basis, smoothed_shoulder_raw)
                else:
                    pitch_equiv, roll_equiv = 0.0, 0.0
                # pitch_equiv is FORWARD-direction again now that BASELINE
                # is hang-down (see the calibration comment above):
                # +pitch_equiv means tilting FROM hang-down TOWARD forward
                # reach, which is real shoulder flexion -- MORE negative
                # ctrl (SHOULDER_PITCH_RANGE's own comment: negative =
                # flexion/forward from arm-at-side). Negated here, no
                # offset needed, since ctrl=0 already IS BASELINE now.
                target_pitch_ctrl = clamp(-pitch_equiv, *SHOULDER_PITCH_RANGE)
                target_roll_ctrl = clamp(roll_equiv, *SHOULDER_ROLL_RANGE)
                target_elbow_ctrl = clamp(ELBOW_OFFSET - (smoothed_elbow_raw_scalar - zero_elbow), *ELBOW_RANGE)
                # Hard rate limit (see MAX_CTRL_RATE_RAD_PER_SEC's comment)
                # -- steps toward each target by at most max_ctrl_step per
                # tick, no matter how far away the target jumped to. Unlike
                # an EMA blend, this bounds worst-case per-tick movement
                # directly instead of only reducing its expected size.
                max_ctrl_step = MAX_CTRL_RATE_RAD_PER_SEC * model.opt.timestep
                if smoothed_pitch_ctrl is None:
                    smoothed_pitch_ctrl = target_pitch_ctrl
                    smoothed_roll_ctrl = target_roll_ctrl
                    smoothed_elbow_ctrl = target_elbow_ctrl
                else:
                    smoothed_pitch_ctrl = rate_limit_step(smoothed_pitch_ctrl, target_pitch_ctrl, max_ctrl_step)
                    smoothed_roll_ctrl = rate_limit_step(smoothed_roll_ctrl, target_roll_ctrl, max_ctrl_step)
                    smoothed_elbow_ctrl = rate_limit_step(smoothed_elbow_ctrl, target_elbow_ctrl, max_ctrl_step)
                data.ctrl[shoulder_pitch_id] = smoothed_pitch_ctrl
                data.ctrl[shoulder_roll_id] = smoothed_roll_ctrl
                data.ctrl[elbow_id] = smoothed_elbow_ctrl

                mujoco.mj_step(model, data)
                # Rendered far less often than stepped (2026-09-02 fix): this
                # scene's full 49-mesh G1 mannequin (added so the arm has a
                # real body for scale/orientation reference, see
                # arm_hand_scene.xml's top comment) is expensive enough to
                # draw that calling viewer.sync() every single 2ms physics
                # step made this loop's own `remaining` pacing below go
                # permanently negative -- confirmed directly (test_arm_kinematics.py's
                # git history) by timing a plain rest hold against this same
                # scene, which took 30+ real seconds for what should have
                # been 2 simulated seconds. Every iteration still steps
                # physics (so the arm's simulated state tracks real time
                # correctly), just doesn't redraw every single one of those
                # steps -- ~20 physics steps between redraws is still a
                # smooth-looking ~25fps at this timestep.
                if step_count % 20 == 0:
                    viewer.sync()

                step_count += 1
                # [CORR]: raw (zero-corrected) sensor value, the UNMAPPED
                # shoulder accelerometer axes behind it, the ctrl it
                # produced, and the resulting MuJoCo wrist position, all on
                # one line -- added 2026-09-03 to debug a reported sideways
                # drift during a pure forward-raise motion (2026-09-03: a
                # live test showed the motion landing almost entirely on
                # shoulder_roll instead of shoulder_pitch -- raw axes added
                # after that, to re-derive the shoulder remap from real
                # measurement instead of guessing, same method as the
                # firmware's own 2026-09-01 remap comment describes).
                # ~2Hz (every 250 physics steps) -- slow enough that a
                # Monitor-style live tail doesn't get rate-limited over a
                # long idle stretch, still fast enough to trace a few-
                # second motion.
                if step_count % 250 == 0:
                    rel = data.xpos[wrist_body_id] - data.xpos[shoulder_body_id]
                    raw_ax, raw_ay, raw_az = latest.snapshot_shoulder_raw()
                    raw_str = ("n/a" if raw_ax is None else
                               f"({raw_ax:+.3f},{raw_ay:+.3f},{raw_az:+.3f})")
                    # Full raw gyro (rad/s), not just accel -- added
                    # 2026-09-05 so a live debugging session can tell "the
                    # arm really moved a lot" (large angular velocity)
                    # apart from "the algorithm mis-split a small motion"
                    # without having to trust only this file's own post-
                    # decode pitch_equiv/roll_equiv numbers.
                    raw_gx, raw_gy, raw_gz = latest.snapshot_shoulder_raw_gyro()
                    gyro_str = ("n/a" if raw_gx is None else
                                f"({raw_gx:+.3f},{raw_gy:+.3f},{raw_gz:+.3f})")
                    e_raw_ax, e_raw_ay, e_raw_az = latest.snapshot_elbow_raw()
                    e_raw_str = ("n/a" if e_raw_ax is None else
                                 f"({e_raw_ax:+.3f},{e_raw_ay:+.3f},{e_raw_az:+.3f})")
                    print(f"[CORR] oblique: pitch_equiv={pitch_equiv:+.3f} roll_equiv={roll_equiv:+.3f} rad  "
                          f"old_decode(unzeroed): shoulder_pitch={old_shoulder_pitch:+.3f} "
                          f"shoulder_roll={old_shoulder_roll:+.3f}  elbow={elbow - zero_elbow:+.3f}  |  "
                          f"raw_shoulder_accel(ax,ay,az)={raw_str}  raw_shoulder_gyro(gx,gy,gz)={gyro_str} rad/s  "
                          f"raw_elbow_accel(ax,ay,az)={e_raw_str}  |  "
                          f"ctrl: pitch={data.ctrl[shoulder_pitch_id]:+.3f} roll={data.ctrl[shoulder_roll_id]:+.3f} "
                          f"elbow={data.ctrl[elbow_id]:+.3f}  |  "
                          f"mujoco wrist(front,left,up)=({rel[0]:+.3f},{rel[1]:+.3f},{rel[2]:+.3f})m")
                if step_count % 200 == 0:
                    # IMU#1/IMU#2: a real protocol-level signal, not a threshold on
                    # decoded values -- shoulder_online/elbow_online come from the
                    # firmware's own shoulder_completions/elbow_completions (see
                    # imu_liveness()'s docstring), i.e. an actual I2C ACK'd,
                    # fully-completed read in the last ~1s, the same thing an NACK/
                    # timeout would report as 0 for. MyoWare has no equivalent: its
                    # ENV output is a bare analog voltage with no protocol, no ACK,
                    # nothing to "ping" -- the ADC returns a plausible-looking number
                    # whether or not anything is even plugged in, and `grip` (the
                    # only thing currently on the wire) legitimately sits at exactly
                    # 0.0 whenever the wearer is simply relaxed, so no threshold on
                    # it can tell "relaxed" apart from "disconnected". Line arrival
                    # is used as its liveness proxy instead, which only catches a
                    # full board/port failure, not a lone MyoWare unplug.
                    if is_stale:
                        imu1_state = imu2_state = myo_state = "offline"
                        shoulder_retries = elbow_retries = 0
                    else:
                        shoulder_online, elbow_online = latest.imu_liveness()
                        shoulder_retries, elbow_retries = latest.imu_retry_attempts()
                        imu1_state = "unknown" if shoulder_online is None else ("online" if shoulder_online else "offline")
                        imu2_state = "unknown" if elbow_online is None else ("online" if elbow_online else "offline")
                        myo_state = "online"
                    # Retry counts are part of the key so the block keeps
                    # reprinting once a second (matching the firmware's own
                    # diag cadence) while something is actually offline and
                    # retrying, instead of only printing once on the initial
                    # online->offline transition and then going quiet.
                    status_key = (imu1_state, imu2_state, myo_state, shoulder_retries, elbow_retries)
                    now = time.time()
                    if status_key != last_status_key or (now - last_status_print_time) >= STATUS_HEARTBEAT_SECONDS:
                        def _line(label, state, retries):
                            padded = f"{label}:".ljust(15)
                            if state == "offline":
                                return f"  {padded} OFFLINE - retrying ({retries} attempts, 0 success since offline)"
                            return f"  {padded} {state}"
                        print("[SENSORS]")
                        print(_line("IMU#1 (0x68)", imu1_state, shoulder_retries))
                        print(_line("IMU#2 (0x69)", imu2_state, elbow_retries))
                        print(_line("MyoWare 2.0", myo_state, 0))
                        last_status_key = status_key
                        last_status_print_time = now

                remaining = model.opt.timestep - (time.time() - step_start)
                if remaining > 0:
                    time.sleep(remaining)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
