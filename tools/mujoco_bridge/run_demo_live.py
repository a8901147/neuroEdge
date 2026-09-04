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
GRIP_SCALE = 0.6
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
SHOULDER_RAW_RE = re.compile(
    r"shoulder_raw_ax=(?P<shoulder_raw_ax>[-\d.eE+]+) "
    r"shoulder_raw_ay=(?P<shoulder_raw_ay>[-\d.eE+]+) "
    r"shoulder_raw_az=(?P<shoulder_raw_az>[-\d.eE+]+)"
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
            buf += chunk
            while b"\r\n" in buf:
                raw, buf = buf.split(b"\r\n", 1)
                line = raw.decode("utf-8", errors="ignore")
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
    args = parser.parse_args()

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

    # Shoulder calibration: 3 poses (REST, FORWARD_RAISE, ABDUCTION_LEFT),
    # each averaged over a settle+hold window -- replaces the old single-
    # pose "zero" capture (git history) now that the shoulder is decoded
    # via oblique_decompose() above instead of subtracting a zero reference
    # from firmware's ComplementaryFilter output. Elbow keeps the same
    # zero-offset approach as before (firmware's dot-product elbow_bend has
    # neither of the problems that drove this shoulder change), captured
    # during the REST hold alongside the shoulder reading.
    #
    # Averaging window (not a single instantaneous read): measured directly
    # against real hardware (raw_capture2.py, 2026-08-29) that an arm
    # someone is actively trying to hold still still drifts ~0.2-0.27rad
    # over several seconds (hand tremor, not sensor noise) -- a fixed
    # window absorbs that without needing the reading to fully stop
    # changing, which never converges in practice.
    def _capture_window(seconds=2.0):
        raw_window = []
        elbow_window = []
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            raw = latest.snapshot_shoulder_raw()
            if raw[0] is not None:
                raw_window.append(raw)
            elbow_window.append(latest.snapshot()[3])
            time.sleep(0.05)
        n = len(raw_window)
        raw_avg = (
            sum(w[0] for w in raw_window) / n,
            sum(w[1] for w in raw_window) / n,
            sum(w[2] for w in raw_window) / n,
        )
        elbow_avg = sum(elbow_window) / len(elbow_window)
        return raw_avg, elbow_avg

    while not latest.is_ready():
        time.sleep(0.05)
    while latest.snapshot_shoulder_raw()[0] is None:
        time.sleep(0.05)

    print("Calibrating shoulder -- REST: get the arm down at your side, elbow "
          "straight, now (2s to get in position, then ~2s of averaging)...")
    time.sleep(2.0)  # time to get in position, not a stability guarantee
    rest_raw, zero_elbow = _capture_window(2.0)

    print("FORWARD_RAISE: raise the arm forward to the highest comfortable "
          "point, now (2s to get in position, then ~2s of averaging)...")
    time.sleep(2.0)
    fwd_raw, _ = _capture_window(2.0)

    print("ABDUCTION_LEFT: raise the arm out to the LEFT side, now (2s to "
          "get in position, then ~2s of averaging)...")
    time.sleep(2.0)
    abd_raw, _ = _capture_window(2.0)

    shoulder_basis = make_oblique_basis(rest_raw, fwd_raw, abd_raw)
    print(f"Shoulder calibrated: REST=({rest_raw[0]:+.3f},{rest_raw[1]:+.3f},{rest_raw[2]:+.3f}) "
          f"FORWARD_RAISE=({fwd_raw[0]:+.3f},{fwd_raw[1]:+.3f},{fwd_raw[2]:+.3f}) "
          f"[tilt={math.degrees(shoulder_basis['tilt_fwd']):.1f}deg] "
          f"ABDUCTION_LEFT=({abd_raw[0]:+.3f},{abd_raw[1]:+.3f},{abd_raw[2]:+.3f}) "
          f"[tilt={math.degrees(shoulder_basis['tilt_abd']):.1f}deg]  elbow zero={zero_elbow:.3f}")

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

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            step_count = 0
            while viewer.is_running():
                step_start = time.time()

                grip, old_shoulder_pitch, old_shoulder_roll, elbow = latest.snapshot()
                shoulder_raw = latest.snapshot_shoulder_raw()
                is_stale, port_error = latest.status()

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
                # oblique_decompose gives dimensionless fwd/abd coefficients
                # (1.0 = "as far as the calibration pose"); scale each by
                # its OWN calibration pose's real tilt angle to get a
                # radian-equivalent, then clamp the same way the old
                # zero-referenced decode did. Negated for pitch -- see
                # SHOULDER_PITCH_RANGE's comment: positive
                # data (flexion/forward) needs NEGATIVE ctrl in this
                # joint's frame. No wrap_angle_delta needed here (unlike
                # the old ComplementaryFilter-decoded values this replaces):
                # oblique_decompose is a linear projection with no atan2
                # branch cut to wrap around.
                # shoulder_raw can't still be None here: main() already
                # blocked until the first raw reading arrived, before
                # calibration, and LatestSample never resets it afterward.
                fwd_coeff, abd_coeff = oblique_decompose(shoulder_basis, shoulder_raw)
                pitch_equiv = fwd_coeff * shoulder_basis["tilt_fwd"]
                roll_equiv = abd_coeff * shoulder_basis["tilt_abd"]
                data.ctrl[shoulder_pitch_id] = clamp(-pitch_equiv, *SHOULDER_PITCH_RANGE)
                data.ctrl[shoulder_roll_id] = clamp(roll_equiv, *SHOULDER_ROLL_RANGE)
                data.ctrl[elbow_id] = clamp(ELBOW_OFFSET - (elbow - zero_elbow), *ELBOW_RANGE)

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
                    e_raw_ax, e_raw_ay, e_raw_az = latest.snapshot_elbow_raw()
                    e_raw_str = ("n/a" if e_raw_ax is None else
                                 f"({e_raw_ax:+.3f},{e_raw_ay:+.3f},{e_raw_az:+.3f})")
                    print(f"[CORR] oblique: fwd={fwd_coeff:+.3f} abd={abd_coeff:+.3f} "
                          f"(pitch_equiv={pitch_equiv:+.3f} roll_equiv={roll_equiv:+.3f} rad)  "
                          f"old_decode(unzeroed): shoulder_pitch={old_shoulder_pitch:+.3f} "
                          f"shoulder_roll={old_shoulder_roll:+.3f}  elbow={elbow - zero_elbow:+.3f}  |  "
                          f"raw_shoulder(ax,ay,az)={raw_str}  raw_elbow(ax,ay,az)={e_raw_str}  |  "
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
