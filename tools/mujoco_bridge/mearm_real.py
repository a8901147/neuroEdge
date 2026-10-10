"""Sensors -> REAL MEArm servo pulses (shoulder, elbow): the last step of Path B, offline and tested before any real use.

    Path B decode (mearm_pathb)  ->  model shoulder command  +  the elbow the person asks for (NOT linkage-projected)
      -> [mode] -> measured link-angle map (mearm_pulse_map, data/mearm_angles_*_signed.json)  -> pulses
      -> measured safe envelope (gen_mearm_envelope / edgeneuro::PulseEnvelope)

Why the elbow is not projected here: mearm_pathb.project_elbow squeezes the elbow into the linkage band of the MuJoCo
model (MeArmPilot's numbers, ~32 deg). Measured on THIS arm (SESSION_LOG 2026-09-27) that band does not exist in the safe
region; what does exist -- base-plate collisions and a linkage bind at high shoulder + high elbow -- is exactly what the
measured envelope encodes. The elbow is carried as the MODEL's relative elbow angle, i.e. the angle between upper arm and
forearm, which is what a human elbow bend means.

Modes (the shoulder only; the real arm reaches ~1/3 of the model's shoulder travel):
  geometric  the model command converted exactly: real link angles = the MuJoCo MeArm's, but most of a human raise
             saturates at the ends of the safe range;
  stretch    the model's whole shoulder travel is spread over the safe range: the whole raise moves the arm, but the
             real upper arm is at a different angle from the model's.
The base is base_pulse() (measured range and direction, 2026-09-28); the claw is grip -> claw_map.
"""
import json
import math
import sys
from collections import namedtuple
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
import gen_mearm_envelope as env  # noqa: E402
import mearm_pathb as pb  # noqa: E402
import mearm_pulse_map as pm  # noqa: E402

ANGLE_FILE = HERE.parent.parent / "data" / "mearm_angles_20260927-133753_signed.json"
MODES = ("geometric", "stretch", "height_reach")
# 2026-09-27: stretch (the arm follows from the first movement); 2026-10-03: height_reach -- the author's design, checked on
# the real arm: the forearm servo sets the claw's height, the upper-arm servo its reach (see pulses()).
DEFAULT_MODE = "height_reach"
REST_SHOULDER_US = env.REST_SHOULDER
# Base, measured on the real arm 2026-09-28: usable 500..2500 us, and 1700 us turns LEFT (seen from behind the arm).
REST_BASE_US = 1500
BASE_RANGE_US = (500, 2500)

Arm = namedtuple("Arm", "angles table")


def load_arm(angle_file=ANGLE_FILE):
    data = json.loads(Path(angle_file).read_text())
    fit = lambda pts: pm.fit_angle_vs_pulse([(p["pulse"], p["angle_deg"]) for p in pts])
    angles = pm.ServoAngleMap(shoulder=fit(data["shoulder_points"]), elbow=fit(data["elbow_points"]))
    table = env.build_table(env.m.load_records([env.ROOT / "data" / n for n in env.SOURCES]))
    return Arm(angles, table)


def pulses(arm, cal, upper_raw, elbow_bend, mode=DEFAULT_MODE):
    """(shoulder_us, elbow_us) whole microseconds, always inside the measured envelope."""
    if mode not in MODES:
        raise ValueError(f"mode must be one of {MODES}")
    if mode == "height_reach":
        return _height_reach(arm, cal, upper_raw, elbow_bend)
    _base, shoulder, _projected = pb.ctrl_from_sensors(cal, upper_raw, elbow_bend)
    if mode == "stretch":
        lo_ctrl, hi_ctrl = arm.angles.reachable_shoulder_ctrl(env.shoulder_range(arm.table))
        frac = (pb.SHOULDER_REST - shoulder) / (pb.SHOULDER_REST - pb.SHOULDER_RAISED)     # 0 = hanging, 1 = raised
        shoulder = hi_ctrl - min(max(frac, 0.0), 1.0) * (hi_ctrl - lo_ctrl)
    ps, _ = arm.angles.pulses_from_model_ctrl(shoulder, pb.ELBOW_EXTENDED)
    s, _ = env.clamp(arm.table, ps, env.REST_ELBOW)
    # The elbow (2026-09-28, the author's hand checks on the real arm): the person's bend is spread over the elbow window the
    # envelope allows AT THIS SHOULDER -- straight = its top, fully bent = its bottom, a LOWER pulse = more bent (the
    # direction checked by hand; the first mapping ran the other way). Converting the model's elbow angle instead left a
    # dead zone: the straight end lay outside the envelope and the first ~45 deg of a real bend were clamped flat.
    f = 0.0 if not math.isfinite(elbow_bend) else \
        (pb.elbow_request(cal, elbow_bend) - pb.ELBOW_EXTENDED) / (pb.ELBOW_FOLDED - pb.ELBOW_EXTENDED)
    hi = env.clamp(arm.table, s, math.inf)[1]
    lo = env.clamp(arm.table, s, -math.inf)[1]
    return env.clamp(arm.table, s, hi - min(max(f, 0.0), 1.0) * (hi - lo))

def smooth_elbow_window(table, shoulder_us):
    """(top, bottom) of the elbow window used to SPREAD the person's raise, as a CONTINUOUS function of the shoulder
    pulse, never outside the measured envelope (2026-10-03: the envelope's own window steps at its measured shoulder
    positions -- top 1500 -> 1850 at 1575 -> 1600 past 1650 ... -- so spreading across it made one degree of elbow bend
    jump the elbow servo up to 350 us). At each measured shoulder the top is the LOWEST top of that point and its
    neighbours (the bottom the highest bottom), joined by straight lines: between two points that stays under both
    windows, i.e. under the envelope's own (conservative) window there."""
    xs, his, los = table["shoulders"], table["hi"], table["lo"]
    n = len(xs)
    top = [min(his[max(i - 1, 0):i + 2]) for i in range(n)]
    bottom = [max(los[max(i - 1, 0):i + 2]) for i in range(n)]
    s = min(max(shoulder_us, xs[0]), xs[-1])
    for i in range(n - 1):
        if xs[i] <= s <= xs[i + 1]:
            f = (s - xs[i]) / (xs[i + 1] - xs[i])
            return top[i] + f * (top[i + 1] - top[i]), bottom[i] + f * (bottom[i + 1] - bottom[i])
    return float(top[-1]), float(bottom[-1])


def _height_reach(arm, cal, upper_raw, elbow_bend):
    """2026-10-03 (the author's design): the person's arm drives the two servos CROSSWISE -- raising the arm (hanging ->
    raised, Path B's tilt with its fades) lowers the ELBOW servo across the elbow window the envelope allows at the
    current shoulder pulse (claw up); bending the elbow (straight -> fully bent) raises the SHOULDER servo across the
    measured shoulder range (reach). Hanging + straight = (1500, 1500) = the R start pose. An invalid upper-arm reading
    gives the rest pose whatever the elbow says (never a guess)."""
    if (upper_raw is None or upper_raw[0] is None
            or not all(math.isfinite(c) for c in upper_raw) or math.hypot(*upper_raw) < 1e-6):
        return env.clamp(arm.table, REST_SHOULDER_US, env.REST_ELBOW)
    _base, shoulder_ctrl, _ = pb.ctrl_from_sensors(cal, upper_raw, elbow_bend)
    f_raise = min(max((pb.SHOULDER_REST - shoulder_ctrl) / (pb.SHOULDER_REST - pb.SHOULDER_RAISED), 0.0), 1.0)
    f_bend = 0.0 if not math.isfinite(elbow_bend) else \
        (pb.elbow_request(cal, elbow_bend) - pb.ELBOW_EXTENDED) / (pb.ELBOW_FOLDED - pb.ELBOW_EXTENDED)
    f_bend = min(max(f_bend, 0.0), 1.0)
    s_lo, s_hi = env.shoulder_range(arm.table)
    s, _ = env.clamp(arm.table, s_lo + f_bend * (s_hi - s_lo), env.REST_ELBOW)
    hi, lo = smooth_elbow_window(arm.table, s)
    return env.clamp(arm.table, s, hi - f_raise * (hi - lo))


def default_base_reach(cal):
    """(left, right) arm azimuths (rad) at which the base hits its ends WITHOUT a measured reach: exactly Path B's own
    base mapping (the calibrated LEFT/RIGHT -> +-BASE_SWING, clamped at +-BASE_LIMIT), i.e. ~+-34 deg for 9/13."""
    return cal.az_left * pb.BASE_LIMIT / pb.BASE_SWING, cal.az_right * pb.BASE_LIMIT / pb.BASE_SWING


def saved_base_reach(cal, saved):
    """The measured comfortable reach from shoulder_calibration.json's raw vectors, or None (none is captured yet)."""
    if "base_reach_left_raw" not in saved or "base_reach_right_raw" not in saved:
        return None
    _t, left = cal.decode(saved["base_reach_left_raw"])
    _t, right = cal.decode(saved["base_reach_right_raw"])
    if not (left > 0.1 and right < -0.1):
        raise ValueError(f"base reach not on both sides of FORWARD ({math.degrees(left):.0f}, {math.degrees(right):.0f} deg)")
    return left, right


def base_pulse(cal, upper_raw, reach=None):
    """Base servo pulse (whole us), left = higher (measured 2026-09-28): the whole 500..2500 us is spread over the arm's
    azimuth from `reach` right to `reach` left (2026-10-03, the author's choice: the arm's comfortable reach, so a turn of
    the arm moves the base less while the base keeps its full range). reach=None: default_base_reach (unchanged
    behaviour). Path B's fades still apply (rest near hanging and behind the body); an invalid reading -> rest."""
    left, right = reach if reach is not None else default_base_reach(cal)
    if (upper_raw is None or upper_raw[0] is None
            or not all(math.isfinite(c) for c in upper_raw) or math.hypot(*upper_raw) < 1e-6):
        return REST_BASE_US
    tilt, az = cal.decode(upper_raw)
    pole, front = pb.fades(tilt, az)
    frac = pb.interp_anchors([(right, -1.0), (0.0, 0.0), (left, 1.0)], az, -1.0, 1.0) * pole * front
    half = (BASE_RANGE_US[1] - BASE_RANGE_US[0]) / 2.0
    us = REST_BASE_US + frac * half
    return int(round(min(max(us, BASE_RANGE_US[0]), BASE_RANGE_US[1])))
