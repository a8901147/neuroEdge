#!/usr/bin/env python3
"""Measure the MEArm's feasible (shoulder, elbow) region on the real arm (SESSION_LOG TODO C).

Why: the arm's claw-levelling parallel linkage couples shoulder and elbow, so not every
(shoulder pulse, elbow pulse) pair is reachable -- driving both independently can bind the
linkage (servo stalls, buzzes, or something bends). Each servo's own range was measured with the
OTHER servos at centre; the COMBINATIONS never were. mearm_pathb.py / mearm_linkage.hpp currently
use MeArmPilot's numbers (measured on THEIR unit).

How: only a person can tell that a linkage is binding (ear: buzzing / straining; eye: links stop
or flex, the claw touches the table). So the tool moves ONE servo at a time, one 25us step every
~0.8 s, and YOU PRESS ENTER at the first sign of anything wrong. It immediately backs off a few
steps and records the pulse it was at. Nothing moves faster than that, and nothing leaves each
servo's already-measured range.

Protocol, for each shoulder position (1500 rest, then 1650, 1800, 1350 -- not the extreme ends):
  1. elbow back to its 1500 rest,  2. walk the shoulder there (Enter if it binds by itself),
  3. sweep the elbow UP until you press Enter or it reaches its measured limit, back to 1500,
  4. sweep the elbow DOWN the same way, back to 1500.
Results are saved after every shoulder position, so you can stop (Ctrl+C) at any time.

After every stop it asks WHY (l = the linkage binds / buzzes / strains, c = something touched the base
plate or table, ? = unsure): a collision is the set-up, not the linkage, and is kept out of the window.
Each run writes a NEW file (data/mearm_linkage_<timestamp>.json) and never overwrites an existing one,
so repeats can be merged afterwards to see how repeatable the edges are.

Needs `servo_limit_finder_4ch` flashed + power-cycled (same protocol as servo_pose_4ch.py).

    python3 tools/measure_linkage_region.py                              # default positions
    python3 tools/measure_linkage_region.py --shoulders 1350,1350,1425,1500,1575   # repeats allowed
    python3 tools/measure_linkage_region.py --analyze data/a.json data/b.json      # no hardware: merge + table
"""

import argparse
import json
import select
import sys
import time
from collections import namedtuple
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import servo_pose_4ch as sp  # noqa: E402

BASE, SHOULDER, ELBOW, CLAW = 1, 2, 3, 4
DWELL_S = 0.8            # time between steps, during which Enter is watched for
RETURN_DWELL_S = 0.15    # time between steps when going back to a known position (never a jump)
BACKOFF_STEPS = 3        # steps to retreat after Enter (covers the reaction time)
REST_US = 1500
CLAW_REST_US = sp.REST[4]   # 1300 = open; the arm's chosen rest (SESSION_LOG 2026-09-26)
MAX_ATTEMPTS = 3         # a position the person keeps rejecting is dropped after this many tries
# outward from rest: up first, then back through rest and down. NOT the two ends of the shoulder's
# range (dropped 2026-09-27): at 2100 three attempts gave a downward stop of 850, 725 and 1400 --
# not repeatable, the mechanism was straining -- and at 1200 the shoulder binds by itself (~1250).
DEFAULT_SHOULDERS = (1500, 1650, 1800, 1350)
DATA_DIR = Path(__file__).resolve().parents[1] / "data"
CAUSES = {"": "linkage", "l": "linkage", "c": "collision", "?": "unsure"}

Walk = namedtuple("Walk", "result stopped_at ended_at")


class WalkFailed(RuntimeError):
    pass


def _settle(board, ch, expected, progress, retries):
    """Read the channel back and make it equal `expected`. A serial link that is fed too fast can lose
    steps (the firmware has a single-byte receive register), and nothing on the way tells us: so
    verify, correct, and if it still will not go there, say so instead of carrying on with a wrong
    idea of where the servo is (the first real run did exactly that)."""
    actual = sp.read_state(board, ch)
    for _ in range(retries):
        if actual == expected:
            return actual
        direction, n = sp.plan_steps(actual, expected)
        key = b"+" if direction > 0 else b"-"
        for _ in range(n):
            board.write(key)
            time.sleep(RETURN_DWELL_S)
        actual = sp.read_state(board, ch)
    if actual != expected:
        raise WalkFailed(f"{sp.NAMES[ch - 1]} did not reach {expected} (read back {actual}). Did the serial link drop "
                         f"steps, or is the board not responding?")
    return actual


def walk(board, ch, target_us, stop_requested=None, dwell_s=DWELL_S, backoff_steps=BACKOFF_STEPS, *,
         flush=None, progress=None, retries=3):
    """Walk channel ch toward target_us one 25us step at a time. Never leaves the channel's measured range.

    stop_requested(dwell_s) -> bool is called after every step and waits up to dwell_s for the person;
    True means "stop here": retreat `backoff_steps` (never past where this walk began) and report where
    the stop happened. With stop_requested=None it is a plain return to a known position: no stop
    possible, but STILL paced (RETURN_DWELL_S per step) -- never a jump. Every walk ends by reading the
    real pulse back and correcting it if steps were lost."""
    target, _clamped = sp.clamp_target(ch, target_us)
    if stop_requested is not None and flush:
        flush()                                          # a queued Enter must not count as "stop"
    start = sp.read_state(board, ch)
    direction, n = sp.plan_steps(start, target)
    key, back = (b"+", b"-") if direction > 0 else (b"-", b"+")
    pulse = start
    for taken in range(1, n + 1):
        board.write(key)
        pulse += direction * sp.STEP_US
        if progress:
            progress(ch, pulse)
        stopped = stop_requested(dwell_s) if stop_requested is not None else (time.sleep(RETURN_DWELL_S) or False)
        if stopped:
            true_stop = sp.read_state(board, ch)         # where it REALLY is, not where we counted
            back_n = min(backoff_steps, taken)
            for _ in range(back_n):
                board.write(back)
                time.sleep(RETURN_DWELL_S)
            ended = _settle(board, ch, true_stop - direction * back_n * sp.STEP_US, progress, retries)
            return Walk("stopped", true_stop, ended)
    ended = _settle(board, ch, start + direction * n * sp.STEP_US, progress, retries)
    return Walk("reached", None, ended)


def _measure_position(board, stop_requested, s, dwell_s, log, flush_input, progress, ask_cause=None, elbow_hold=REST_US):
    e_lo, e_hi = sp.RANGES[ELBOW]
    # 1. Every position starts from the SAME pose, approached the same way: elbow to rest first (a shoulder
    # move with the elbow elsewhere could itself bind), then the shoulder back to rest. Backlash / hysteresis
    # would otherwise make a position reached from somewhere else stop at a different pulse, which cannot
    # be told apart from the mechanism simply being unrepeatable.
    # (elbow_hold, 2026-09-28: the elbow waits there instead of 1500 whenever the shoulder moves -- see parse_elbow_hold)
    walk(board, ELBOW, elbow_hold, None, progress=progress)
    walk(board, SHOULDER, REST_US, None, progress=progress)
    rec = {"shoulder": s, "elbow_hold": elbow_hold, "shoulder_bind_at_elbow_1500": None, "elbow_up_bind": None,
           "elbow_down_bind": None, "elbow_up_limit": e_hi, "elbow_down_limit": e_lo,
           "elbow_up_start": None, "elbow_down_start": None,
           "shoulder_bind_cause": None, "elbow_up_cause": None, "elbow_down_cause": None}
    log(f"\n=== shoulder {s} ===  (press Enter AT ONCE on anything abnormal you see or hear)")
    w = walk(board, SHOULDER, s, stop_requested, dwell_s, flush=flush_input, progress=progress)   # 2.
    if w.result == "stopped":
        rec["shoulder_bind_at_elbow_1500"] = w.stopped_at
        log(f"  the shoulder itself bound (elbow at {elbow_hold}): at about {w.stopped_at}. Skipping this position.")
        if ask_cause:
            rec["shoulder_bind_cause"] = ask_cause("shoulder", w.stopped_at)
        walk(board, SHOULDER, REST_US, None, progress=progress)
        return rec
    log("  sweeping the elbow up (increasing pulse)...")
    rec["elbow_up_start"] = sp.read_state(board, ELBOW)
    up = walk(board, ELBOW, e_hi, stop_requested, dwell_s, flush=flush_input, progress=progress)   # 3.
    if up.result == "stopped":
        rec["elbow_up_bind"] = up.stopped_at
        log(f"  stopped going up at {up.stopped_at}.")
        if ask_cause:
            rec["elbow_up_cause"] = ask_cause("elbow_up", up.stopped_at)
    else:
        log(f"  reached {e_hi} with no problem.")
    walk(board, ELBOW, elbow_hold, None, progress=progress)
    log("  sweeping the elbow down (decreasing pulse)...")
    rec["elbow_down_start"] = sp.read_state(board, ELBOW)
    down = walk(board, ELBOW, e_lo, stop_requested, dwell_s, flush=flush_input, progress=progress)  # 4.
    if down.result == "stopped":
        rec["elbow_down_bind"] = down.stopped_at
        log(f"  stopped going down at {down.stopped_at}.")
        if ask_cause:
            rec["elbow_down_cause"] = ask_cause("elbow_down", down.stopped_at)
    else:
        log(f"  reached {e_lo} with no problem.")
    walk(board, ELBOW, elbow_hold, None, progress=progress)
    # back at the rest pose before anything is asked: the servos hold their pulse, and the person may take a while
    walk(board, SHOULDER, REST_US, None, progress=progress)
    return rec


def return_to_rest(board, progress=None):
    """All four servos back to the chosen rest pose, paced: claw and base first, then the elbow BEFORE the
    shoulder (a shoulder move with the elbow elsewhere could bind the linkage)."""
    for ch in sp.MOVE_ORDER:
        walk(board, ch, sp.REST[ch], None, progress=progress)


def safe_finish(board, progress=None, log=print):
    """return_to_rest for every way a run can end (done, Ctrl+C, an error): it must not raise, and if the
    arm cannot be put back it says so instead of leaving the person thinking it was."""
    try:
        log("  arm returning to the rest pose (claw 1300)...")
        return_to_rest(board, progress)
        log("  back at the rest pose.")
    except (WalkFailed, sp.BoardNotAnswering, KeyboardInterrupt, OSError) as exc:
        log(f"  Warning: the arm could not return to the rest pose ({exc}). Check the arm; if needed use servo_pose_4ch.py's "
            f"rest command.")


def measure(board, stop_requested, shoulders=DEFAULT_SHOULDERS, save=lambda records: None,
            dwell_s=DWELL_S, log=print, *, flush_input=None, confirm=None, progress=None,
            max_attempts=MAX_ATTEMPTS, ask_cause=None, rejected_out=None, elbow_hold=REST_US):
    """Run the protocol; returns one record per ACCEPTED shoulder position (rejected attempts go to
    `rejected_out`, if given -- never into the returned records). After each position
    `confirm(record)` (if given) lets the person reject it -- an accidental Enter -- and it is measured
    again, up to max_attempts times, after which it is dropped."""
    records = []
    # Base and claw go to their rest before anything is measured and are never touched again (whatever a
    # previous session or the pose tool left behind is not a starting point). The elbow and shoulder are
    # reset at the start of EVERY position instead (_measure_position).
    walk(board, CLAW, CLAW_REST_US, None, progress=progress)
    walk(board, BASE, REST_US, None, progress=progress)
    for s in shoulders:
        for attempt in range(1, max_attempts + 1):
            rec = _measure_position(board, stop_requested, s, dwell_s, log, flush_input, progress, ask_cause,
                                    elbow_hold)
            if confirm is None or confirm(rec):
                records.append(rec)
                save(records)
                break
            if rejected_out is not None:
                rejected_out.append(dict(rec, attempt=attempt))     # evidence, kept apart from the measurements
                save(records)                                       # ...and written out now, not at the end
            log(f"  re-measuring shoulder {s} (you rejected attempt {attempt}/{max_attempts})")
        else:
            log(f"  shoulder {s} was rejected {max_attempts} times in a row; this position is not recorded.")
    return_to_rest(board, progress)
    return records


def windows(records, margin_steps=3, include_collisions=False):
    """Per shoulder position the elbow window [lo, hi], pulled inward from the recorded stops by
    `margin_steps` (the stop is where you pressed Enter, i.e. already past the first sign of trouble).
    An edge that never bound falls back to the servo's own measured limit. Several records at one
    position (repeats, or several runs) are combined CONSERVATIVELY (the lowest upper edge, the highest
    lower edge) and the spread between repeats is reported: a wide spread means the number is not
    trustworthy. A stop the person labelled a collision (the base plate, the table) is not a linkage edge
    and is left out unless include_collisions; a stop with no label counts as linkage."""
    groups = {}
    for r in records:
        if r["shoulder_bind_at_elbow_1500"] is not None:
            continue
        g = groups.setdefault(r["shoulder"], {"his": [], "los": [], "ups": [], "downs": [], "collisions": []})
        for edge, cause_key, bind_key, limit_key, bucket, edge_list, sign in (
                ("elbow_up", "elbow_up_cause", "elbow_up_bind", "elbow_up_limit", "his", "ups", -1),
                ("elbow_down", "elbow_down_cause", "elbow_down_bind", "elbow_down_limit", "los", "downs", +1)):
            bind = r[bind_key]
            if bind is not None and r.get(cause_key) == "collision":
                g["collisions"].append((edge, bind))
                if not include_collisions:
                    bind = None
            if bind is None:
                g[bucket].append(r[limit_key])
            else:
                g[bucket].append(bind + sign * margin_steps * sp.STEP_US)
                g[edge_list].append(bind)
    out = []
    for shoulder, g in groups.items():
        spread = lambda v: (max(v) - min(v)) if len(v) >= 2 else None
        out.append({"shoulder": shoulder, "lo": max(g["los"]), "hi": min(g["his"]), "n": len(g["his"]),
                    "up_spread": spread(g["ups"]), "down_spread": spread(g["downs"]),
                    "collisions": g["collisions"]})
    return out


def safe_envelope(records, margin_steps=3):
    """What the REAL arm may be commanded to: a stop of ANY cause counts (a claw hitting the base plate is as
    bad for the hardware as a linkage binding), unlike windows() which by default answers the physics question
    "does the linkage couple shoulder and elbow?".

    Returns the elbow windows per shoulder position (collisions included) and the shoulder's own bounds, taken
    from where it stopped by itself with the elbow at rest: the stop NEAREST rest on each side, pulled inward
    by the margin. A bound no stop was ever seen for is the servo's measured range and is flagged untested."""
    lo_lim, hi_lim = sp.RANGES[SHOULDER]
    stops = [r["shoulder_bind_at_elbow_1500"] for r in records if r["shoulder_bind_at_elbow_1500"] is not None]
    below = [x for x in stops if x < REST_US]
    above = [x for x in stops if x > REST_US]
    return {
        "windows": windows(records, margin_steps, include_collisions=True),
        "shoulder_lo": max(below) + margin_steps * sp.STEP_US if below else lo_lim,
        "shoulder_hi": min(above) - margin_steps * sp.STEP_US if above else hi_lim,
        "shoulder_lo_tested": bool(below),
        "shoulder_hi_tested": bool(above),
    }


def load_records(paths):
    """The records of several measurement files, in order. Refuses anything that is not one."""
    records = []
    for path in paths:
        data = json.loads(Path(path).read_text())
        if not isinstance(data, dict) or not isinstance(data.get("records"), list):
            raise ValueError(f"{path} is not a measurement file (no records)")
        records.extend(data["records"])
    return records


def fit_line(points):
    """Least-squares y = intercept + slope*x through (x, y) points; None if fewer than 3."""
    if len(points) < 3:
        return None
    n = len(points)
    mx = sum(x for x, _ in points) / n
    my = sum(y for _, y in points) / n
    sxx = sum((x - mx) ** 2 for x, _ in points)
    if sxx == 0:
        return None
    slope = sum((x - mx) * (y - my) for x, y in points) / sxx
    intercept = my - slope * mx
    return {"slope": slope, "intercept": intercept, "n": n,
            "max_residual": max(abs(y - (intercept + slope * x)) for x, y in points)}


def summarize(records, margin_steps=3, out=print):
    w = windows(records, margin_steps)
    out("\nshoulder  elbow window (safety margin applied)       runs  top spread  bottom spread")
    fmt = lambda v: "   -" if v is None else f"{v:4d}"
    for x in sorted(w, key=lambda x: x["shoulder"]):
        out(f"{x['shoulder']:5d}     {x['lo']:6.0f} ~ {x['hi']:6.0f}   (width {x['hi'] - x['lo']:5.0f})  {x['n']:3d}     "
            f"{fmt(x['up_spread'])}        {fmt(x['down_spread'])}")
    selfs = [r for r in records if r["shoulder_bind_at_elbow_1500"] is not None]
    if selfs:
        label = {"collision": "collision", "linkage": "linkage", "unsure": "unsure"}
        out("The shoulder bound by itself (positions it cannot reach with the elbow at 1500; the shoulder's safe limit):")
        for r in selfs:
            out(f"  aimed for {r['shoulder']} -> stopped at {r['shoulder_bind_at_elbow_1500']}  "
                f"({label.get(r.get('shoulder_bind_cause'), 'cause unlabelled')})")
        stops = [r["shoulder_bind_at_elbow_1500"] for r in selfs]
        if len(stops) >= 2:
            out(f"  these stops repeat within {max(stops) - min(stops)} µs")
    collisions = [(x["shoulder"], e, p) for x in w for e, p in x["collisions"]]
    if collisions:
        out("Stops caused by a collision (not a linkage limit; excluded from the windows above): " +
            ", ".join(f"shoulder {s_} {e} {p}" for s_, e, p in collisions))
    env = safe_envelope(records, margin_steps)
    out("\nSafe envelope (limits for the real arm: a stop of any cause counts; safety margin applied):")
    out(f"  shoulder: {env['shoulder_lo']} ~ {env['shoulder_hi']}"
        + ("" if env["shoulder_lo_tested"] else "   (lower limit untested: only the servo's own range)")
        + ("" if env["shoulder_hi_tested"] else "   (upper limit untested: only the servo's own range)"))
    for x in sorted(env["windows"], key=lambda x: x["shoulder"]):
        out(f"  shoulder {x['shoulder']:5d}: elbow {x['lo']:5.0f} ~ {x['hi']:5.0f}")
    lo_lim, hi_lim = sp.RANGES[ELBOW]
    for name, key in (("top hi", "hi"), ("bottom lo", "lo")):
        f = fit_line([(x["shoulder"], x[key]) for x in w if x[key] not in (lo_lim, hi_lim)])
        if f:
            out(f"{name}: elbow = {f['intercept']:.0f} + {f['slope']:.3f} × shoulder   (max residual {f['max_residual']:.0f} µs, "
                f"{f['n']} points)")
        else:
            out(f"{name}: fewer than 3 real binding points, cannot fit a line yet")


def default_out_path(now=None):
    """A NEW file for every run, so a repeat measurement can never overwrite an earlier one."""
    return DATA_DIR / time.strftime("mearm_linkage_%Y%m%d-%H%M%S.json", now or time.localtime())


def check_output_path(path, overwrite=False):
    if Path(path).exists() and not overwrite:
        raise FileExistsError(f"{path} already exists and will not be overwritten (pass --overwrite, or choose another --out)")


def parse_shoulders(text, elbow_hold=REST_US):
    """'1350,1350,1425' -> (1350, 1350, 1425). Repeats are allowed (that is how repeatability is measured);
    the two ends of the shoulder's range are not (the mechanism strains there: SESSION_LOG 2026-09-27).
    Exception, 2026-09-28: the strain at the TOP end was measured with the elbow at 1500, where the elbow's linkage hits
    the upper arm; with the elbow held lower (elbow_hold < 1500) the shoulder may go up to its own measured limit."""
    lo, hi = sp.RANGES[SHOULDER]
    top = hi if elbow_hold < REST_US else hi - 100
    out = []
    for token in text.split(","):
        token = token.strip()
        if not token.lstrip("-").isdigit():
            raise ValueError(f"'{token}' is not an integer")
        v = int(token)
        if not (lo + 100 < v <= top if elbow_hold < REST_US else lo + 100 < v < top):
            raise ValueError(f"shoulder {v} is too close to the ends of travel (allowed: between {lo + 100} and {top}"
                             f"{'' if elbow_hold < REST_US else ', exclusive; with the elbow out of the way (--elbow-hold below 1500) the upper limit is ' + str(hi)})")
        out.append(v)
    return tuple(out)


ELBOW_HOLD_RANGE = (700, 1500)


def parse_elbow_hold(text):
    """Where the elbow waits while the shoulder moves (default 1500). 2026-09-28 real arm: above shoulder 1800 the elbow's
    linkage hits the upper arm with the elbow at 1500, so the shoulder cannot rise; a LOWER elbow pulse lets it. The
    shoulder always returns to 1500 with the elbow at the hold, so the hold must be inside the elbow window measured at
    shoulder 1500 (500..1500, SESSION_LOG 2026-09-27) and away from the elbow's own end of travel."""
    token = text.strip()
    if not token.isdigit():
        raise ValueError(f"'{token}' is not an integer")
    v = int(token)
    lo, hi = ELBOW_HOLD_RANGE
    if not lo <= v <= hi:
        raise ValueError(f"--elbow-hold {v} is outside the allowed {lo}-{hi} (the elbow's safe range measured at shoulder 1500)")
    return v


def ask_ok(rec, input_fn=input, drain=None, out=print):
    """Show what was recorded for a shoulder position and let the person reject it (an accidental
    Enter is easy to make while watching an arm). True = keep, False = measure this position again."""
    if drain:
        drain()             # a queued Enter must not silently accept a bad result
    if rec["shoulder_bind_at_elbow_1500"] is not None:
        out(f"  record: shoulder {rec['shoulder']} bound by itself, at about {rec['shoulder_bind_at_elbow_1500']}")
    else:
        up = rec["elbow_up_bind"] if rec["elbow_up_bind"] is not None else f"no binding (reached {rec['elbow_up_limit']})"
        down = rec["elbow_down_bind"] if rec["elbow_down_bind"] is not None else f"no binding (reached {rec['elbow_down_limit']})"
        out(f"  record: shoulder {rec['shoulder']}  elbow stopped going up at {up}, going down at {down}")
    return input_fn("  Is this position OK?  Enter=OK   r=re-measure: ").strip().lower() != "r"


def ask_cause_cli(where, pulse, input_fn=input, drain=None, out=print):
    """Why did it stop? A linkage that binds (buzzing, straining, links stopping or flexing) is what is
    being measured; something touching the base plate or the table is the set-up, not the linkage."""
    if drain:
        drain()
    while True:
        answer = input_fn(f"  Why did it stop at {where} {pulse}?  l=linkage binding/buzzing/straining (also Enter)  "
                          f"c=hit the base/table/another part  ?=unsure: ").strip().lower()
        if answer in CAUSES:
            return CAUSES[answer]
        out("  unrecognized, type l, c or ?")


def _stdin_stop(dwell_s):
    """True if the person pressed Enter within dwell_s."""
    ready, _, _ = select.select([sys.stdin], [], [], dwell_s)
    if ready:
        sys.stdin.readline()
        return True
    return False


def _drain_stdin():
    while select.select([sys.stdin], [], [], 0)[0]:
        sys.stdin.readline()


def result_payload(shoulders, records, rejected):
    """What a run writes to its result file. Rejected attempts are saved too, under their own key."""
    return {"measured_at": time.strftime("%Y-%m-%d %H:%M:%S"), "step_us": sp.STEP_US, "dwell_s": DWELL_S,
            "return_dwell_s": RETURN_DWELL_S, "backoff_steps": BACKOFF_STEPS, "claw_rest_us": CLAW_REST_US,
            "shoulders_requested": list(shoulders), "records": records, "rejected_attempts": rejected}


def load_rejected(paths):
    out = []
    for path in paths:
        data = json.loads(Path(path).read_text())
        if isinstance(data, dict):
            out.extend(data.get("rejected_attempts", []))
    return out


def summarize_rejected(rejected, out=print):
    if not rejected:
        return
    show = lambda v: "no binding" if v is None else str(v)
    out("\nRejected attempts (not in the windows or envelope above, but inconsistency at one position is information too):")
    for r in rejected:
        out(f"  shoulder {r['shoulder']} attempt {r.get('attempt', '?')}: elbow up {show(r.get('elbow_up_bind'))}, "
            f"down {show(r.get('elbow_down_bind'))}")


def analyze(paths, margin_steps=3, out=print):
    """No hardware: merge one or more measurement files and print the combined table."""
    records = load_records(paths)
    out(f"{len(records)} record(s) from {len(paths)} file(s)")
    summarize(records, margin_steps, out=out)
    summarize_rejected(load_rejected(paths), out=out)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None)
    parser.add_argument("--cp2102", action="store_true", help="use the CP2102 adapter")
    parser.add_argument("--out", type=Path, default=None,
                        help="result file (default: a NEW data/mearm_linkage_<timestamp>.json; never overwrites)")
    parser.add_argument("--overwrite", action="store_true", help="allow --out to replace an existing file")
    parser.add_argument("--shoulders", default=None,
                        help="comma-separated shoulder pulses, repeats allowed (default: "
                             + ",".join(map(str, DEFAULT_SHOULDERS)) + ")")
    parser.add_argument("--elbow-hold", default="1500",
                        help="elbow pulse while the shoulder moves (default 1500). Lower it (e.g. 1200) to measure "
                             "shoulders above 1800, where the elbow at 1500 makes its linkage hit the upper arm")
    parser.add_argument("--analyze", nargs="+", type=Path, metavar="FILE",
                        help="no hardware: merge these result files and print the combined table")
    args = parser.parse_args()
    if args.analyze:
        analyze(args.analyze)
        return
    try:
        elbow_hold = parse_elbow_hold(args.elbow_hold)
        shoulders = parse_shoulders(args.shoulders, elbow_hold) if args.shoulders else DEFAULT_SHOULDERS
        out_path = args.out or default_out_path()
        check_output_path(out_path, args.overwrite)
    except (ValueError, FileExistsError) as exc:
        sys.exit(str(exc))

    import serial
    from usb_serial_port import autodetect_port
    board = serial.Serial(args.port or autodetect_port(prefer_cp2102=args.cp2102), sp.BAUD, timeout=0.5)
    state = {ch: sp.read_state(board, ch) for ch in sp.RANGES}
    print("Current pulse widths: " + "  ".join(f"{sp.NAMES[ch - 1]}={v}" for ch, v in state.items()))
    print(f"\nShoulder positions: {list(shoulders)}  (the elbow stays at {elbow_hold} while the shoulder moves)\n"
          f"Results go to: {out_path}")
    print(f"\nThis moves the shoulder and elbow (one servo at a time, one 25 µs step every {DWELL_S} s). The claw first "
          f"walks slowly to {CLAW_REST_US} (open)\nand the base to 1500, and neither moves again; at the end (or on an "
          f"interruption) the whole arm returns to the rest pose.\n"
          f"Clear the space around the arm. On anything abnormal (buzzing, straining, a link binding or bending, hitting "
          f"something) press Enter ONCE, AT ONCE:\nthe tool backs off a little and asks why it stopped (l=linkage "
          f"binding/buzzing, c=hit the base or table, ?=unsure).\nCollisions are not linkage coupling and are recorded "
          f"separately. After each position it asks whether it was OK; type r to re-measure after a false press. "
          f"Ctrl+C stops at any time.")
    input("Press Enter to start: ")
    _drain_stdin()

    rejected = []

    def save(records):
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(result_payload(shoulders, records, rejected), indent=2,
                                       ensure_ascii=False) + "\n")

    def progress(ch, pulse):
        print(f"\r  {sp.NAMES[ch - 1]:9s}{pulse:5d} µs   ", end="", flush=True)

    def log(msg):
        print("\r" + " " * 40 + "\r" + msg)

    records = []
    interrupted = False
    try:
        records = measure(board, _stdin_stop, shoulders=shoulders, save=save, log=log, flush_input=_drain_stdin,
                          progress=progress, confirm=lambda rec: ask_ok(rec, drain=_drain_stdin), rejected_out=rejected,
                          elbow_hold=elbow_hold,
                          ask_cause=lambda where, pulse: ask_cause_cli(where, pulse, drain=_drain_stdin))
    except KeyboardInterrupt:
        print("\nInterrupted. The partial results saved so far are in the file.")
        interrupted = True
    except WalkFailed as exc:
        print(f"\nAborted: {exc}\nThe partial results saved so far are in the file.")
        interrupted = True
    if interrupted:
        safe_finish(board, progress=None, log=log)            # a normal run already returned to rest itself
    print(f"\nRaw results saved in {out_path}")
    if records:
        summarize(records)
    summarize_rejected(rejected)
    if records or rejected:
        print(f"\nTo merge several runs and see the repeatability: python3 tools/measure_linkage_region.py --analyze {out_path} "
              f"<other files>")


if __name__ == "__main__":
    main()
