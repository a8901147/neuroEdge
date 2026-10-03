"""Writes data/linkage_golden.csv: (shoulder, elbow_request, projected_elbow)
rows computed by mearm_pathb.project_elbow -- the golden table the C++ port in
include/edgeneuro/control/mearm_linkage.hpp is tested against.

    python3 tools/mujoco_bridge/gen_linkage_golden.py
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import mearm_pathb as pb  # noqa: E402

OUT = HERE.parent.parent / "data" / "linkage_golden.csv"


def render():
    n_s, n_r = 12, 11
    shoulders = [pb.SHOULDER_RAISED + (pb.SHOULDER_REST - pb.SHOULDER_RAISED) * i / (n_s - 1)
                 for i in range(n_s)]
    # from below ELBOW_EXTENDED to above ELBOW_FOLDED, so clamping is exercised
    lo, hi = pb.ELBOW_EXTENDED - 0.5, pb.ELBOW_FOLDED + 0.5
    requests = [lo + (hi - lo) * j / (n_r - 1) for j in range(n_r)]
    lines = ["shoulder,elbow_request,elbow_projected"]
    for s in shoulders:
        for r in requests:
            lines.append(f"{s:.9f},{r:.9f},{pb.project_elbow(s, r):.9f}")
    return "\n".join(lines) + "\n"


def write(path=OUT):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(render())
    return path


if __name__ == "__main__":
    print(f"wrote {write()}")
