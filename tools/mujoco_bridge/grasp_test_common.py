"""Shared scenario logic for test_grasp_object.py and
test_myoware_grip_replay.py -- reach to the (real-reach-verified, see
arm_hand_scene.xml's pedestal comment) grasp object, drive the grip
actuators from whatever `grip_at_t` callback the caller supplies, then
perform a "lift" (raise the arm a bit) and report whether the object
followed the hand or got left behind. Split out of the two test scripts
because this scenario (XML ball-param overrides, fingertip-centroid
tracking, the lift phase, the held/dropped verdict) is substantial enough
that duplicating it risked the two copies silently diverging -- unlike the
small GRIP_ACTUATORS-style constant dicts this project is fine duplicating
by hand elsewhere.

Not a standalone script -- import from a `mjpython`-run test file.
"""

import math
import tempfile
import time
from pathlib import Path

import mujoco
import mujoco.viewer

REPO_ROOT = Path(__file__).resolve().parents[2]
SCENE_XML = REPO_ROOT / "tools" / "mujoco_bridge" / "arm_hand_scene.xml"

# Identical to run_demo_live.py's GRIP_SCALE/GRIP_ACTUATORS -- kept in sync
# by hand, see that file's comment for the sign/range provenance.
GRIP_SCALE = 0.6
GRIP_ACTUATORS = {
    "left_hand_thumb_1_joint": 1.0472,
    "left_hand_thumb_2_joint": 1.74533,
    "left_hand_middle_0_joint": -1.5708,
    "left_hand_middle_1_joint": -1.74533,
    "left_hand_index_0_joint": -1.5708,
    "left_hand_index_1_joint": -1.74533,
}

# Same reach pose as test_grip_kinematics.py -- a front-LEFT reach (roll is
# a real, comfortable abduction angle, not near-zero) found via a real
# mj_step search, 2026-09-05 (see arm_hand_scene.xml's pedestal comment).
# Puts the closed-fist fingertip centroid right at the (also relocated that
# same day) object's position.
REACH_CTRL = {
    "left_shoulder_pitch_joint": -1.00,
    "left_shoulder_roll_joint": 0.80,
    "left_elbow_joint": 0.90,
}

# The "lift" phase raises the arm by moving shoulder_pitch from REACH_CTRL's
# -1.00 toward this less-flexed value -- far enough to be a real, unambiguous
# movement (not a twitch), well short of the joint's own -3.0892/+1.0472
# range limits.
LIFT_SHOULDER_PITCH = -0.40

DEFAULT_BALL_RADIUS = 0.025
DEFAULT_BALL_FRICTION = "1.0 0.02 0.005"

FINGERTIP_BODIES = ["left_hand_thumb_2_link", "left_hand_middle_1_link", "left_hand_index_1_link"]

SYNC_EVERY_N_STEPS = 20


def load_model(ball_radius=DEFAULT_BALL_RADIUS, ball_friction=DEFAULT_BALL_FRICTION):
    """Loads arm_hand_scene.xml, optionally with the grasp object's radius/
    friction overridden. Substitutes the two exact attribute strings in the
    XML text (confirmed unique in the file) rather than editing the shared
    XML on disk -- lets test_grasp_object.py's --ball-radius/--ball-friction
    flags experiment (per the user's own idea: maybe a smaller/grippier
    ball is what it takes) without mutating the file every other script in
    this directory also loads. Writes to a temp file IN this same directory
    (not /tmp) so the XML's relative meshdir="../../mujoco_menagerie" still
    resolves; the temp file is only needed during from_xml_path's parse, so
    it's deleted immediately after, before this function returns.
    """
    xml_text = SCENE_XML.read_text()
    changed = False
    if ball_radius != DEFAULT_BALL_RADIUS:
        old = f'size="{DEFAULT_BALL_RADIUS}"'
        new = f'size="{ball_radius}"'
        if old not in xml_text:
            raise ValueError(f"expected exactly one {old!r} in {SCENE_XML} (the object sphere) -- "
                              f"file may have changed, update this override logic")
        xml_text = xml_text.replace(old, new)
        changed = True
    if ball_friction != DEFAULT_BALL_FRICTION:
        old = f'friction="{DEFAULT_BALL_FRICTION}"'
        new = f'friction="{ball_friction}"'
        if old not in xml_text:
            raise ValueError(f"expected exactly one {old!r} in {SCENE_XML} (the object sphere) -- "
                              f"file may have changed, update this override logic")
        xml_text = xml_text.replace(old, new)
        changed = True

    if not changed:
        return mujoco.MjModel.from_xml_path(str(SCENE_XML))

    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".xml", dir=str(SCENE_XML.parent), delete=False
    ) as f:
        f.write(xml_text)
        tmp_path = Path(f.name)
    try:
        return mujoco.MjModel.from_xml_path(str(tmp_path))
    finally:
        tmp_path.unlink()


def fingertip_centroid(model, data):
    """Where the fingers actually converge when closed -- the real grasp
    point, not the wrist joint behind them (see arm_hand_scene.xml's
    pedestal comment for why that distinction mattered when relocating the
    object)."""
    pts = [data.xpos[model.body(name).id] for name in FINGERTIP_BODIES]
    return sum(pts) / len(pts)


def step_and_sync(model, data, viewer, seconds, on_tick=None):
    """Steps physics for `seconds` of sim time, syncing the viewer at a
    normal frame rate (not every physics step -- see test_arm_kinematics.py's
    own comment for why that matters on this scene's full mannequin mesh
    count). `on_tick(t)` is called once per physics step, before mj_step,
    with elapsed seconds since this call started -- used by callers to set
    ctrl for that tick.

    `viewer=None` runs headless: no window, no real-time pacing, no
    is_running() check -- just steps physics as fast as possible and still
    calls on_tick/prints. Added 2026-09-05 so these scenarios can be judged
    from the printed log alone (verdict included) without a human watching
    a window -- mujoco.viewer.launch_passive needs `mjpython` and an actual
    display, which an agent driving this via a shell has neither of."""
    n_steps = int(seconds / model.opt.timestep)
    for i in range(n_steps):
        if viewer is not None and not viewer.is_running():
            return False
        if on_tick is not None:
            on_tick(i * model.opt.timestep)
        mujoco.mj_step(model, data)
        if viewer is not None and i % SYNC_EVERY_N_STEPS == 0:
            viewer.sync()
            time.sleep(model.opt.timestep * SYNC_EVERY_N_STEPS)
    return True


def run_grasp_scenario(model, data, viewer, grip_at_t, grip_phase_seconds,
                        lift_seconds=2.0, print_every_seconds=0.3):
    """Runs: settle at REACH_CTRL with grip=0 -> drive grip from
    `grip_at_t(t)` (t in seconds since the grip phase started, returns a
    [0,1] scalar) for `grip_phase_seconds` -> lift (ramp shoulder_pitch to
    LIFT_SHOULDER_PITCH over `lift_seconds`, holding the last commanded grip
    target) -> report.

    Returns a dict: object_start_dist, object_end_dist (fingertip-centroid
    to object-center distance right before vs. after the lift), object_drop
    (how much the object's own height fell during the lift), and held (bool
    verdict: stayed within 0.08m of the fingertips AND didn't fall more than
    0.05m -- both generous relative to the 0.025m default ball radius, so
    "held" means genuinely still in the hand, not just technically nearby).
    """
    grip_ids = {name: model.actuator(name).id for name in GRIP_ACTUATORS}
    object_id = model.body("object").id
    pitch_id = model.actuator("left_shoulder_pitch_joint").id

    for name, val in REACH_CTRL.items():
        data.ctrl[model.actuator(name).id] = val

    print("Settling into reach pose (grip=0)...")
    step_and_sync(model, data, viewer, 1.5)
    centroid = fingertip_centroid(model, data)
    obj_pos = data.xpos[object_id].copy()
    print(f"  fingertip_centroid={centroid.round(3)}  object={obj_pos.round(3)}  "
          f"dist={float(_dist(centroid, obj_pos)):.3f}m")

    last_grip = [0.0]
    last_print = [-999.0]

    def _on_tick_grip(t):
        grip = max(0.0, min(1.0, grip_at_t(t)))
        last_grip[0] = grip
        for name, upper_range in GRIP_ACTUATORS.items():
            data.ctrl[grip_ids[name]] = grip * GRIP_SCALE * upper_range
        if t - last_print[0] >= print_every_seconds:
            c = fingertip_centroid(model, data)
            o = data.xpos[object_id]
            print(f"  [GRIP]  t={t:5.2f}s grip={grip:.3f}  object={o.round(3)}  "
                  f"dist_to_fingertips={float(_dist(c, o)):.3f}m")
            last_print[0] = t

    print(f"\n[GRIP PHASE] driving grip for {grip_phase_seconds:.1f}s...")
    step_and_sync(model, data, viewer, grip_phase_seconds, on_tick=_on_tick_grip)

    centroid_before_lift = fingertip_centroid(model, data)
    obj_before_lift = data.xpos[object_id].copy()
    dist_before_lift = float(_dist(centroid_before_lift, obj_before_lift))
    print(f"\nBefore lift: dist_to_fingertips={dist_before_lift:.3f}m  "
          f"object_height={obj_before_lift[2]:.3f}m")

    pitch_start = data.ctrl[pitch_id]
    last_print[0] = -999.0

    def _on_tick_lift(t):
        frac = min(1.0, t / lift_seconds)
        data.ctrl[pitch_id] = pitch_start + frac * (LIFT_SHOULDER_PITCH - pitch_start)
        for name, upper_range in GRIP_ACTUATORS.items():
            data.ctrl[grip_ids[name]] = last_grip[0] * GRIP_SCALE * upper_range
        if t - last_print[0] >= print_every_seconds:
            c = fingertip_centroid(model, data)
            o = data.xpos[object_id]
            print(f"  [LIFT]  t={t:5.2f}s  object={o.round(3)}  "
                  f"dist_to_fingertips={float(_dist(c, o)):.3f}m")
            last_print[0] = t

    print(f"\n[LIFT PHASE] raising shoulder_pitch to {LIFT_SHOULDER_PITCH:+.2f} over {lift_seconds:.1f}s...")
    step_and_sync(model, data, viewer, lift_seconds, on_tick=_on_tick_lift)
    step_and_sync(model, data, viewer, 1.0)  # settle after the lift finishes

    centroid_after = fingertip_centroid(model, data)
    obj_after = data.xpos[object_id].copy()
    dist_after = float(_dist(centroid_after, obj_after))
    height_drop = float(obj_before_lift[2] - obj_after[2])

    held = dist_after < 0.08 and height_drop < 0.05
    print(f"\nAfter lift: dist_to_fingertips={dist_after:.3f}m  "
          f"object_height={obj_after[2]:.3f}m  height_drop={height_drop:+.3f}m")
    print(f"\nVERDICT: {'HELD' if held else 'DROPPED'} "
          f"(dist_after={dist_after:.3f}m {'<' if dist_after < 0.08 else '>='} 0.08m, "
          f"height_drop={height_drop:+.3f}m {'<' if height_drop < 0.05 else '>='} 0.05m)")

    return {
        "object_start_dist": dist_before_lift,
        "object_end_dist": dist_after,
        "object_drop": height_drop,
        "held": held,
    }


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def add_common_args(parser):
    """--headless/--ball-radius/--ball-friction/--grip-scale/
    --grip-phase-seconds, shared verbatim by test_grasp_object.py and
    test_myoware_grip_replay.py so both take the same flags the same way."""
    parser.add_argument(
        "--headless", action="store_true",
        help="skip mujoco.viewer entirely and just run physics + print the log -- "
             "works under plain python3, no mjpython/display needed. The verdict "
             "line is the same either way.",
    )
    parser.add_argument("--grip-scale", type=float, default=GRIP_SCALE,
                         help=f"overrides GRIP_ACTUATORS's scale (default {GRIP_SCALE}, "
                              f"the same value run_demo_live.py currently uses)")
    parser.add_argument("--ball-radius", type=float, default=DEFAULT_BALL_RADIUS)
    parser.add_argument("--ball-friction", default=DEFAULT_BALL_FRICTION)


def run_with_viewer_or_headless(args, grip_at_t, grip_phase_seconds):
    """Shared main-body logic for test_grasp_object.py/
    test_myoware_grip_replay.py: apply --grip-scale, load the model (with
    --ball-radius/--ball-friction if given), run run_grasp_scenario() either
    under mujoco.viewer.launch_passive (mjpython, interactive) or headless
    (plain python3, --headless), and return its result dict."""
    global GRIP_SCALE
    if args.grip_scale != GRIP_SCALE:
        GRIP_SCALE = args.grip_scale
        print(f"Overriding GRIP_SCALE to {args.grip_scale}")

    model = load_model(ball_radius=args.ball_radius, ball_friction=args.ball_friction)
    data = mujoco.MjData(model)
    print(f"Loaded scene with ball_radius={args.ball_radius}, ball_friction={args.ball_friction!r}, "
          f"grip_scale={GRIP_SCALE}, headless={args.headless}")

    if args.headless:
        return run_grasp_scenario(model, data, None, grip_at_t=grip_at_t,
                                   grip_phase_seconds=grip_phase_seconds)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        result = run_grasp_scenario(model, data, viewer, grip_at_t=grip_at_t,
                                     grip_phase_seconds=grip_phase_seconds)
        print("\nHolding final pose -- close the viewer window to exit.")
        i = 0
        while viewer.is_running():
            mujoco.mj_step(model, data)
            if i % SYNC_EVERY_N_STEPS == 0:
                viewer.sync()
            i += 1
    return result
