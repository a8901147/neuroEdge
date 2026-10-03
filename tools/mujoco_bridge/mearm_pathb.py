"""Path B, offline default mapping: real sensors -> MeArm model ctrl, from the
SAVED calibration alone (no new capture needed).

  Path A: sensors -> MuJoCo humanoid arm     (run_demo_live.py --humanoid, untouched)
  Path B: sensors -> MuJoCo MeArm arm        (this module + run_demo_live.py --mearm)

Scope (2026-09-25, explicit): get the BIG directions right before the next
hardware session -- arm left => model left, arm up => model up, elbow bend =>
model folds -- from the 9/13 calibration (the one calibration that
demonstrably worked for a full real 6-step task). Fine detail (exact anchors,
the elbow's visual up/down convention) is left to the interactive
calibration (calibrate_mearm_alignment.py) at the hardware.

Why a SPHERICAL decode (tilt, azimuth) instead of run_demo_live.py's oblique
(pitch_equiv, roll_equiv): a MeArm base+shoulder IS an azimuth/elevation
pan-tilt, so "how far from hanging" (tilt) -> shoulder and "which direction"
(azimuth) -> base is the natural correspondence. Measured on the saved 9/13
vectors: the oblique decode leaks pitch into a right swing (RIGHT_TWIST
decodes to pitch +1.08 / roll -0.63 rad, so swinging right would also lift the
shoulder to ~85%), while in (tilt, azimuth) LEFT/RIGHT come out symmetric
(+-28 deg) at a near-constant tilt (68-72 deg). The oblique decode itself is
Path A's and stays exactly as it is; nothing here imports or modifies it.

Sensing limits worth remembering (they shape the design):
  * Azimuth is not measured, it is a GESTURE PROXY: an accelerometer cannot see
    rotation about gravity, so left/right only shows up through the thumb-twist
    the wearer adds (+-28 deg of decoded azimuth stands in for a much larger
    physical swing; the anchors below supply the gain).
  * Azimuth is undefined at tilt 0 (hanging) and noisy near it -> the base
    output is faded in over a tilt window instead of trusting it there.
  * The MeArm cannot reach behind the body -> that region fades back to REST
    smoothly instead of aliasing (azimuth flips sign at +-180 deg; a hard
    switch there would slam a real servo across its range).

Shoulder/elbow are NOT independent on a MeArm (2026-09-25): its claw-leveling
parallel linkage couples them (mearm_scene.xml: tool = pi/2 - shoulder - elbow,
tool limited), so only a diagonal band of (shoulder + elbow) is reachable.
Commanding them independently -- the first version of this module -- made a
real logged motion violate the band at every pose, the model's constraint
solver pinned the tool link and the shoulder lagged its command by ~18deg
(RMS); on a real MEArm the same thing would be a binding linkage. The shoulder
keeps priority (lifting is the big visible direction) and the elbow is fitted
into the window the linkage allows at that shoulder (project_elbow), which
costs elbow amplitude: at a fixed shoulder the elbow can travel only ~32deg,
so a 129deg human flexion shows as a small fold in the right direction. The
band limits are MeArmPilot's, measured on THEIR unit -- not this project's.

Pure functions only (no serial, no viewer, no MuJoCo import) so it is
unit-testable and cannot form an import cycle with run_demo_live.py.
"""

import math

# MeArm model ctrl targets -- copied from mearm_scene.xml's <actuator> block
# and measured behavior (2026-09-24: shoulder ctrl +0.898 = LOWEST arm
# elevation ~43deg, -0.141 = HIGHEST ~87deg; elbow ctrl 0.995 = EXTENDED,
# 2.617 = FOLDED). test_mearm_direction.py pins these against run_demo_live's
# MEARM_* constants so a drift between the two copies is caught.
SHOULDER_REST = 0.898057932        # arm hanging  -> lowest elevation
SHOULDER_RAISED = -0.141261412     # arm forward  -> highest elevation
ELBOW_EXTENDED = 0.994603031
ELBOW_FOLDED = 2.61715444
BASE_LIMIT = 1.08210414
BASE_SWING = 0.9                   # model base rotation for the calibrated LEFT/RIGHT swing (rad)

# Elbow flexion swing measured on real hardware (9/05 single-sitting capture,
# test_imu_to_mujoco.py's ELBOW_FLEXION vs REST: dot-product angle 17.7deg ->
# 146.7deg). Only this RELATIVE swing is borrowed -- 9/05's absolute vectors
# come from a different wearing session (upper-arm HANG differs 24.9deg from
# 9/13's) and must not be mixed with the 9/13 file.
ELBOW_SWING_RAD = math.radians(129.0)

# The MeArm's claw-leveling parallel linkage couples shoulder and elbow.
# mearm_scene.xml models it as  tool = TOOL_LOCK_SUM - shoulder - elbow  with the
# passive tool joint limited to TOOL_LIMIT, so (shoulder + elbow) may only lie in
#   [TOOL_LOCK_SUM - TOOL_LIMIT[1], TOOL_LOCK_SUM - TOOL_LIMIT[0]]  = [1.858, 2.511].
# Both numbers are copied from that file (MeArmPilot's measurement on THEIR unit;
# test_mearm_direction.py pins them to the XML). They are NOT measured on this
# project's own MEArm -- see SESSION_LOG.md's TODO (measure the real feasible
# shoulder/elbow region with servo_limit_finder_4ch before any real-servo use).
TOOL_LOCK_SUM = 1.57079632679
TOOL_LIMIT = (-0.940003288, -0.286958051)
LINKAGE_MARGIN = 0.05              # rad kept inside the band so the tool link never sits exactly on its limit

TILT_FADE_IN_DEG = (8.0, 20.0)     # base output faded in over this tilt window (20deg = the calibration's own minimum tilt)
BEHIND_FADE_DEG = (90.0, 135.0)    # |azimuth| window over which the arm fades back to REST


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _mul(a, s):
    return tuple(x * s for x in a)


def _unit(v):
    n = math.sqrt(_dot(v, v))
    if n < 1e-9:
        raise ValueError("cannot normalize a zero vector")
    return tuple(x / n for x in v)


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def _smoothstep(x):
    x = _clamp(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def interp_anchors(anchors, x, out_lo, out_hi):
    """Piecewise-linear through (x, y) anchors, extrapolating along the end
    segments, then clamped to [out_lo, out_hi]. Needs >=2 anchors with
    distinct x. (Same idea as run_demo_live.apply_anchor_map -- duplicated on
    purpose so this module has no dependency on run_demo_live.py.)"""
    if math.isnan(x):
        raise ValueError("interp_anchors: x is NaN")
    pts = sorted((float(a), float(b)) for a, b in anchors)
    if len(pts) < 2:
        raise ValueError("need at least 2 anchors")
    for (xa, _), (xb, _) in zip(pts, pts[1:]):
        if xb - xa < 1e-9:
            raise ValueError(f"anchors need distinct x (got {xa} and {xb})")
    if x <= pts[0][0]:
        (xa, ya), (xb, yb) = pts[0], pts[1]
    elif x >= pts[-1][0]:
        (xa, ya), (xb, yb) = pts[-2], pts[-1]
    else:
        for i in range(len(pts) - 1):
            if pts[i][0] <= x <= pts[i + 1][0]:
                (xa, ya), (xb, yb) = pts[i], pts[i + 1]
                break
    y = ya + (x - xa) * (yb - ya) / (xb - xa)
    return _clamp(y, min(out_lo, out_hi), max(out_lo, out_hi))


class Calibration:
    """Everything derived from the saved calibration, built once."""

    def __init__(self, hang, forward, left, right, zero_elbow):
        self.h = _unit(hang)
        t_fwd = self._tangent(_unit(forward))
        self.e1 = _unit(t_fwd)                      # azimuth 0 = the FORWARD direction
        t_left = self._tangent(_unit(left))
        self.e2 = _unit(_sub(t_left, _mul(self.e1, _dot(t_left, self.e1))))   # + azimuth = LEFT
        self.tilt_forward = math.acos(_clamp(_dot(self.h, _unit(forward)), -1.0, 1.0))
        _, self.az_left = self.decode(left)
        _, self.az_right = self.decode(right)
        if self.tilt_forward < math.radians(20.0):
            raise ValueError("FORWARD is <20deg from HANG -- calibration unusable")
        if not (self.az_left > 0.1 and self.az_right < -0.1):
            raise ValueError(
                f"LEFT/RIGHT azimuths ({math.degrees(self.az_left):.0f}, "
                f"{math.degrees(self.az_right):.0f} deg) are not on opposite sides of FORWARD")
        self.zero_elbow = float(zero_elbow)

    def _tangent(self, v):
        return _sub(v, _mul(self.h, _dot(v, self.h)))

    def decode(self, raw):
        """(tilt, azimuth) in radians: tilt = angle from HANG (>=0), azimuth =
        direction of the tilt, 0 = FORWARD, + = LEFT, range (-pi, pi]."""
        v = _unit(raw)
        tilt = math.acos(_clamp(_dot(v, self.h), -1.0, 1.0))
        t = self._tangent(v)
        if math.sqrt(_dot(t, t)) < 1e-9:
            return tilt, 0.0
        return tilt, math.atan2(_dot(t, self.e2), _dot(t, self.e1))


def make_calibration(saved):
    """From run_demo_live.py's calibration dict (shoulder_calibration.json)."""
    for key in ("baseline_raw", "forward_raw", "left_twist_raw", "right_twist_raw", "zero_elbow"):
        if key not in saved:
            raise KeyError(f"calibration is missing '{key}'")
    return Calibration(saved["baseline_raw"], saved["forward_raw"], saved["left_twist_raw"],
                       saved["right_twist_raw"], saved["zero_elbow"])


def linkage_band():
    """(lo, hi) allowed for shoulder + elbow, with LINKAGE_MARGIN kept inside."""
    return (TOOL_LOCK_SUM - TOOL_LIMIT[1] + LINKAGE_MARGIN,
            TOOL_LOCK_SUM - TOOL_LIMIT[0] - LINKAGE_MARGIN)


def elbow_window(shoulder):
    """(lo, hi) of elbow ctrl that keeps shoulder + elbow inside the linkage
    band AND inside the elbow actuator's own range. Its width is constant
    (band width - 2*margin, ~0.55 rad ~ 32 deg): at any fixed shoulder the
    elbow can only travel that far -- a mechanism fact, not a tuning choice."""
    band_lo, band_hi = linkage_band()
    return max(ELBOW_EXTENDED, band_lo - shoulder), min(ELBOW_FOLDED, band_hi - shoulder)


def elbow_request(cal, elbow_bend):
    """The elbow ctrl the human's bend asks for, ignoring the linkage
    (zero_elbow -> EXTENDED, zero_elbow + real swing -> FOLDED)."""
    return interp_anchors(
        [(cal.zero_elbow, ELBOW_EXTENDED), (cal.zero_elbow + ELBOW_SWING_RAD, ELBOW_FOLDED)],
        elbow_bend, ELBOW_EXTENDED, ELBOW_FOLDED)


def project_elbow(shoulder, request):
    """Fit the requested elbow into the window the linkage allows at this
    shoulder, PROPORTIONALLY (not a hard clamp: a clamp would already saturate
    at half a real flexion when the shoulder is low). The shoulder keeps
    priority -- lifting is the big visible direction -- and the elbow adapts:
    a straight human arm raised high asks for 'extended' but the mechanism can
    only hold that with the forearm folded to the window's lower edge."""
    # Same guards as the C++ port (include/edgeneuro/control/mearm_linkage.hpp):
    # the shoulder is clamped into its actuator range (NaN -> rest) so the window
    # can't go empty/non-finite, and a NaN request means 'extended'. A NaN
    # reaching data.ctrl would poison the whole MuJoCo state.
    if math.isnan(shoulder):
        shoulder = SHOULDER_REST
    shoulder = _clamp(shoulder, SHOULDER_RAISED, SHOULDER_REST)
    lo, hi = elbow_window(shoulder)
    if math.isnan(request):
        fraction = 0.0
    else:
        fraction = (request - ELBOW_EXTENDED) / (ELBOW_FOLDED - ELBOW_EXTENDED)
    return lo + _clamp(fraction, 0.0, 1.0) * (hi - lo)


REST_CTRL = (0.0, SHOULDER_REST, elbow_window(SHOULDER_REST)[0])


def fades(tilt, az):
    """(pole, front), each 0..1: the base fades in as the arm leaves HANG (azimuth is meaningless near hanging) and the
    whole arm fades back to rest behind the body. Shared by ctrl_from_sensors and mearm_real.base_pulse."""
    t_lo, t_hi = (math.radians(d) for d in TILT_FADE_IN_DEG)
    pole = _smoothstep((tilt - t_lo) / (t_hi - t_lo))
    lo, hi = (math.radians(d) for d in BEHIND_FADE_DEG)
    front = _clamp((hi - abs(az)) / (hi - lo), 0.0, 1.0)   # 1 in front, 0 behind the body
    return pole, front


def ctrl_from_sensors(cal, upper_raw, elbow_bend, elbow_anchors=None):
    """(base, shoulder, elbow) MeArm model ctrl.

    upper_raw: the upper-arm IMU's raw accel (ax, ay, az) in g;
    elbow_bend: firmware's elbow value (angle between the two IMUs' gravity
    vectors, rad, unsigned). Base and shoulder come ONLY from the upper-arm
    IMU, so an elbow-only motion cannot move them (tested). The elbow comes
    from elbow_bend but is fitted into the linkage window at the current
    shoulder, so shoulder + elbow is always feasible (tested).

    elbow_anchors: optional measured [(bend, elbow_ctrl), ...] pairs (from
    calibrate_mearm_alignment.py) replacing the default zero_elbow/129deg
    guess; the linkage projection is applied to them all the same.
    """
    if (upper_raw is None or upper_raw[0] is None
            or not all(math.isfinite(c) for c in upper_raw) or math.hypot(*upper_raw) < 1e-6):
        base, shoulder = 0.0, SHOULDER_REST     # no valid reading (incl. NaN/inf) -> rest, never a guess
    else:
        tilt, az = cal.decode(upper_raw)
        pole, front = fades(tilt, az)
        base = interp_anchors(
            [(cal.az_right, -BASE_SWING), (0.0, 0.0), (cal.az_left, BASE_SWING)],
            az, -BASE_LIMIT, BASE_LIMIT) * pole * front
        shoulder = interp_anchors(
            [(0.0, SHOULDER_REST), (cal.tilt_forward, SHOULDER_RAISED)],
            tilt * front, SHOULDER_RAISED, SHOULDER_REST)
    if not math.isfinite(elbow_bend):
        request = ELBOW_EXTENDED                # a garbage elbow reading -> straight, never a guess
    elif elbow_anchors is None:
        request = elbow_request(cal, elbow_bend)
    else:
        request = interp_anchors(elbow_anchors, elbow_bend, ELBOW_EXTENDED, ELBOW_FOLDED)
    return base, shoulder, project_elbow(shoulder, request)
