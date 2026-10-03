"""Opens mearm_scene.xml in MuJoCo's interactive viewer -- no live sensor
connection, no control logic, just a free-standing look/drag/inspect tool
for the MeArm digital-twin model (see that file's own header comment for
where it came from and what's still unverified about it).

Must run as `mjpython view_mearm.py`, not plain `python3` -- same
launch_passive/main-thread requirement as run_demo.py/run_demo_live.py,
see those files' own comments for why.
"""

import time
from pathlib import Path

import mujoco
import mujoco.viewer

REPO_ROOT = Path(__file__).resolve().parents[2]
SCENE_XML = REPO_ROOT / "tools" / "mujoco_bridge" / "mearm_scene.xml"


def main():
    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            mujoco.mj_step(model, data)
            viewer.sync()
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
