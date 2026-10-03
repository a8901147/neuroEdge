"""Model command <-> real servo pulse width for the MEArm (Path B's missing last step).

Path B produces MuJoCo model commands (radians). The real arm takes servo pulse widths. Physically (SESSION_LOG
2026-09-27) both the shoulder and the elbow servo sit at the base and drive their links through parallel linkages, so

    the UPPER ARM's absolute elevation  = f(shoulder pulse)      -- independent of the elbow
    the FOREARM's absolute elevation    = g(elbow pulse)         -- independent of the shoulder

The MuJoCo scene instead uses RELATIVE joints (upper arm = 90deg - shoulder, forearm = 90deg - shoulder - elbow, and the
tool link locked level), verified against the simulator in test_mearm_pulse_map.py. So the two are joined by plain
geometry, and the only things to measure on the real arm are two one-dimensional curves: link elevation (degrees above
horizontal, e.g. from a phone inclinometer laid on the link) against pulse width. Each is a straight line
    elevation = at_1500 + slope * (pulse - 1500)          (slope in degrees per microsecond, either sign).

Pure functions and one small class; no serial, no viewer, no MuJoCo import.
"""
import math

REST_US = 1500
# SG92R: ~180 degrees over ~2000us = 0.09 deg/us. The linkage may scale that; far outside this is a units/typing mistake.
PLAUSIBLE_SLOPE_DEG_PER_US = (0.02, 0.4)


def upper_arm_elevation_deg(shoulder_ctrl):
    return 90.0 - math.degrees(shoulder_ctrl)


def forearm_elevation_deg(shoulder_ctrl, elbow_ctrl):
    return 90.0 - math.degrees(shoulder_ctrl + elbow_ctrl)


def model_ctrl_from_elevations(upper_deg, forearm_deg):
    """(shoulder_ctrl, elbow_ctrl) radians for the two links' absolute elevations."""
    return math.radians(90.0 - upper_deg), math.radians(upper_deg - forearm_deg)


def slope_is_plausible(slope_deg_per_us):
    lo, hi = PLAUSIBLE_SLOPE_DEG_PER_US
    return lo <= abs(slope_deg_per_us) <= hi


def fit_angle_vs_pulse(points):
    """Least-squares elevation = at_1500 + slope*(pulse-1500) through (pulse_us, degrees) points. Refuses fewer than 3
    points, a single repeated pulse, or a non-finite reading."""
    pts = [(float(p), float(a)) for p, a in points]
    if len(pts) < 3:
        raise ValueError("need at least 3 (pulse, angle) points")
    if not all(math.isfinite(p) and math.isfinite(a) for p, a in pts):
        raise ValueError("non-finite reading")
    mx = sum(p for p, _ in pts) / len(pts)
    sxx = sum((p - mx) ** 2 for p, _ in pts)
    if sxx == 0:
        raise ValueError("all readings were taken at the same pulse")
    my = sum(a for _, a in pts) / len(pts)
    slope = sum((p - mx) * (a - my) for p, a in pts) / sxx
    at_1500 = my + slope * (REST_US - mx)
    residual = max(abs(a - (at_1500 + slope * (p - REST_US))) for p, a in pts)
    return {"slope": slope, "at_1500": at_1500, "max_residual": residual, "n": len(pts)}


class ServoAngleMap:
    """Two fitted lines -> conversions between servo pulses, the links' elevations and MuJoCo model commands."""

    def __init__(self, shoulder, elbow):
        for name, fit in (("shoulder", shoulder), ("elbow", elbow)):
            if fit["slope"] == 0 or not math.isfinite(fit["slope"]):
                raise ValueError(f"{name}: a flat or non-finite fit cannot be inverted")
        self.shoulder, self.elbow = shoulder, elbow

    def upper_arm_elevation_deg(self, shoulder_pulse):
        return self.shoulder["at_1500"] + self.shoulder["slope"] * (shoulder_pulse - REST_US)

    def forearm_elevation_deg(self, elbow_pulse):
        return self.elbow["at_1500"] + self.elbow["slope"] * (elbow_pulse - REST_US)

    def model_ctrl_from_pulses(self, shoulder_pulse, elbow_pulse):
        return model_ctrl_from_elevations(self.upper_arm_elevation_deg(shoulder_pulse),
                                          self.forearm_elevation_deg(elbow_pulse))

    def pulses_from_model_ctrl(self, shoulder_ctrl, elbow_ctrl):
        upper = upper_arm_elevation_deg(shoulder_ctrl)
        forearm = forearm_elevation_deg(shoulder_ctrl, elbow_ctrl)
        return (REST_US + (upper - self.shoulder["at_1500"]) / self.shoulder["slope"],
                REST_US + (forearm - self.elbow["at_1500"]) / self.elbow["slope"])

    def reachable_shoulder_ctrl(self, pulse_range):
        """(low, high) shoulder model command the given shoulder pulse range can reach."""
        a, b = (model_ctrl_from_elevations(self.upper_arm_elevation_deg(p), 0.0)[0] for p in pulse_range)
        return min(a, b), max(a, b)

    def shoulder_travel_share(self, pulse_range, model_range):
        """Fraction of the model's shoulder travel a pulse range covers (1.0 = all of it)."""
        lo, hi = self.reachable_shoulder_ctrl(pulse_range)
        return (hi - lo) / (model_range[1] - model_range[0])
