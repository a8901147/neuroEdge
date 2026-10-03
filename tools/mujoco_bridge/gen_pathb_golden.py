"""Writes data/pathb_calibration.csv and data/pathb_golden.csv: the calibration
(2026-09-13 saved vectors) and (upper-arm raw, elbow bend) -> mearm_pathb
ctrl_from_sensors rows that tests/test_mearm_pathb.cpp checks the C++ port
against. Inputs: the 2026-09-24 REAL live log (each row with several elbow
readings) plus special poses.

    python3 tools/mujoco_bridge/gen_pathb_golden.py
"""
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import mearm_pathb as pb  # noqa: E402
import test_mearm_direction as fx  # noqa: E402  (embedded 9/13 calibration + real log)

DATA = HERE.parent.parent / "data"


def _rows():
    cal = pb.make_calibration(fx.SAVED_9_13)
    bends = (fx.STRAIGHT, fx.STRAIGHT + 0.7, fx.FLEXED, 0.0, 3.0)
    special = [fx.HANG, fx.FORWARD, fx.LEFT, fx.RIGHT,
               tuple(-c for c in fx.HANG),                       # upside down: behind the body
               fx._slerp(fx.HANG, fx.FORWARD, 0.5),              # half raised
               fx._slerp(fx.HANG, fx.FORWARD, 0.05),             # inside the pole fade
               fx._sweep_about_hang(fx.FORWARD, 1.8),            # behind-body fade window
               fx._sweep_about_hang(fx.FORWARD, -1.8),
               (0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (0.0, -1.0, 0.0)]
    # tilt inside the pole fade window (8..20deg) at several azimuths: base is
    # tilt-gated there, and only a nonzero azimuth makes the gate visible
    for frac in (0.12, 0.2, 0.28):
        for az in (-0.9, -0.4, 0.4, 0.9):
            special.append(fx._sweep_about_hang(fx._slerp(fx.HANG, fx.FORWARD, frac), az))
    raws = list(fx.REAL_LOG_2026_09_24) + special
    out = []
    for raw in raws:
        for bend in bends:
            b, s, e = pb.ctrl_from_sensors(cal, raw, bend)
            out.append((raw, bend, (b, s, e)))
    return out


def render_calibration():
    s = fx.SAVED_9_13
    vals = list(s["baseline_raw"]) + list(s["forward_raw"]) + list(s["left_twist_raw"]) \
        + list(s["right_twist_raw"]) + [s["zero_elbow"]]
    head = "hang_x,hang_y,hang_z,fwd_x,fwd_y,fwd_z,left_x,left_y,left_z,right_x,right_y,right_z,zero_elbow"
    return head + "\n" + ",".join(f"{v:.9f}" for v in vals) + "\n"


def render_golden():
    lines = ["ax,ay,az,elbow_bend,base,shoulder,elbow"]
    for raw, bend, ctrl in _rows():
        lines.append(",".join(f"{v:.9f}" for v in (*raw, bend, *ctrl)))
    return "\n".join(lines) + "\n"


def write(out_dir=DATA):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "pathb_calibration.csv").write_text(render_calibration())
    (out_dir / "pathb_golden.csv").write_text(render_golden())
    return out_dir


if __name__ == "__main__":
    print(f"wrote pathb_calibration.csv and pathb_golden.csv to {write()}")
