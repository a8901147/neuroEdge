"""Automatic IMU health checks, so a hardware fault is never mistaken for an algorithm problem.

Built from the two real failures of 2026-09-27 (SESSION_LOG), which cost a day of debugging the wrong thing:
  * the upper-arm MPU6050 (0x68) dropping out whenever the arm moved: completions 0, its last reading repeated
    bit for bit, |a| of 0.21 g or 2.2 g;
  * the forearm one (0x69) stuck while its reads still "completed" 246/s: the value frozen at (1.999939, 0, 0),
    one axis at full scale -- so nothing downstream noticed.
A real accelerometer is never bit-for-bit constant (it is noisy) and reads about 1 g of gravity when still.

Checks, per sensor, over the last second: no_data (no/invalid readings), no_response (the firmware's diag line says
0 completed reads), frozen (a long run of bit-identical readings), saturated (an axis pinned at full scale for most
of the window), magnitude (median |a| far from 1 g -- a median, so a moving arm's brief accelerations do not count),
not_enough_data (too few samples to judge yet: never called healthy by default).

Pure Python, no serial/MuJoCo: feed it samples (HealthMonitor.add_sample / add_diag) or poll a
run_demo_live.LatestSample without changing it (LatestSamplePoller).
"""
import math
from collections import deque, namedtuple

WINDOW_S = 1.0
MIN_SAMPLES = 20                 # healthy is ~34 lines/s (2026-10-03: the firmware queues its UART output; 115200 baud
                                 # carries ~34 of its ~346-byte lines per second): below this the RATE is low -> warning
MIN_SAMPLES_TO_JUDGE = 10        # below this there is too little to trust anything -> fault
FROZEN_RUN = 30                  # ~0.3 s of bit-identical readings: never happens with a live sensor
SATURATION_G = 1.99              # the +-2 g range's full scale (32767/16384 = 1.99994)
SATURATED_SHARE = 0.5
MAGNITUDE_OK_G = (0.7, 1.3)
NO_DATA_AFTER_S = 0.5
DIAG_VALID_S = 2.5

SENSORS = {"upper_arm": ("上臂", "0x68"), "forearm": ("前臂", "0x69")}
KIND_TEXT = {
    "no_data": "沒有收到讀數",
    "no_response": "沒有回應(I2C 讀取成功 0 次)",
    "frozen": "讀數凍結(一直是同一個數字)",
    "saturated": "讀數卡在滿刻度",
    "magnitude": "讀數大小不像重力(不是約 1 g)",
    "not_enough_data": "幾乎沒有資料,無法判斷",
    "slow_data": "資料太少(正常每秒約 30 筆,但讀數本身正常)——常見原因:感測器接觸不良(也許是麵包板造成的),匯流排一直在自我恢復",
    "implausible": "讀數不可能來自正常運作的感測器",
    "asleep": "感測器回報自己在睡眠狀態(斷電重開過,資料不會更新)",
    "dropouts": "斷線後又恢復(I2C 沒回應)——接觸不良,也許是麵包板造成的",
    "power_reset": "剛才斷電重開過,韌體已自動重新喚醒",
}
HARDWARE_WARNING_S = 5.0          # a hardware warning stays shown this long after its last event

import re as _re
_DIAG_FIELD = _re.compile(r"(shoulder|elbow)_(nacks|timeouts|asleep_rewakes|pwr_mgmt_1|power_resets)=(\d+)")
_DIAG_KEYS = {"nacks": "nacks", "timeouts": "timeouts", "asleep_rewakes": "rewakes", "power_resets": "power_resets",
              "pwr_mgmt_1": "pwr_mgmt_1"}
_FIRMWARE_NAME = {"shoulder": "upper_arm", "elbow": "forearm"}

Problem = namedtuple("Problem", "sensor kind detail")
# ok/problems: can the data be trusted right now (problems hold the model). warnings: what the hardware reported recently --
# a brief drop-out, a power reset -- which the user must be told about, but which leave the data usable.
Report = namedtuple("Report", "ok problems warnings", defaults=((),))


def parse_diag_line(text):
    """{sensor: {nacks, timeouts, rewakes, power_resets, pwr_mgmt_1}} from the firmware's once-a-second diag line
    (nacks/timeouts per window; rewakes/power_resets cumulative since boot); None if it is not a diag line. Fields an older
    firmware does not send are None."""
    if "diag " not in text or "shoulder_completions=" not in text:
        return None
    out = {name: dict.fromkeys(("nacks", "timeouts", "rewakes", "power_resets", "pwr_mgmt_1")) for name in SENSORS}
    for fw, field, value in _DIAG_FIELD.findall(text):
        v = int(value)
        if field == "pwr_mgmt_1" and v > 0xFF:
            v = None                 # the firmware's 0xFFFFFFFF = "not read yet" (its SLEEP bit is NOT a reading)
        out[_FIRMWARE_NAME[fw]][_DIAG_KEYS[field]] = v
    return out


def _valid(v):
    return v is not None and len(v) == 3 and all(isinstance(c, (int, float)) and math.isfinite(c) for c in v)


PLAUSIBLE_G = (0.3, 3.0)


def plausible(v):
    """Could a live accelerometer have produced this ONE reading? (valid, not an axis pinned at full scale, and a
    magnitude between a free-fall-ish 0.3 g and a hard 3 g shake). The instant guard; HealthMonitor is the loud one."""
    if not _valid(v):
        return False
    if max(abs(c) for c in v) >= SATURATION_G:
        return False
    return PLAUSIBLE_G[0] <= math.sqrt(sum(c * c for c in v)) <= PLAUSIBLE_G[1]


class HealthMonitor:
    def __init__(self):
        self._samples = {name: deque() for name in SENSORS}      # (t, vector or None)
        self._runs = {name: [None, 0] for name in SENSORS}      # last vector, identical-run length
        self._diag = None                                         # (t, {name: completions})
        self._last_t = None
        self.sample_count = 0
        self._hw_prev = None                                      # last cumulative counters
        self._hw_events = {name: {} for name in SENSORS}         # kind -> (time, detail)
        self._pwr = {name: None for name in SENSORS}
        self._watching_since = None                               # set after a gap in OUR polling (see below)

    def add_sample(self, t, upper, fore):
        self.sample_count += 1
        self._last_t = t
        for name, v in (("upper_arm", upper), ("forearm", fore)):
            v = tuple(v) if _valid(v) else None
            q = self._samples[name]
            q.append((t, v))
            while q and q[0][0] < t - WINDOW_S:
                q.popleft()
            run = self._runs[name]
            if v is not None and v == run[0]:
                run[1] += 1
            else:
                run[0], run[1] = v, 1

    def add_hardware(self, t, hw):
        """One parsed diag line (parse_diag_line). Increases in the cumulative counters and any drop-out in this window
        become warnings; a PWR_MGMT_1 with SLEEP set becomes a fault."""
        if hw is None:
            return
        for name, c in hw.items():
            prev = (self._hw_prev or {}).get(name, {})
            lost = (c.get("nacks") or 0) + (c.get("timeouts") or 0)
            woke = 0
            if c.get("rewakes") is not None and prev.get("rewakes") is not None:
                woke = max(0, c["rewakes"] - prev["rewakes"])
            if lost or woke:
                self._hw_events[name]["dropouts"] = (t, f"過去 1 秒沒回應 {lost} 次,重新喚醒 {woke} 次")
            if c.get("power_resets") is not None and prev.get("power_resets") is not None \
                    and c["power_resets"] > prev["power_resets"]:
                self._hw_events[name]["power_reset"] = (t, f"累計 {c['power_resets']} 次")
            self._pwr[name] = c.get("pwr_mgmt_1")
        self._hw_prev = hw

    def observation_gap(self, t):
        """The caller stopped watching for a while (its loop was blocked -- e.g. the MuJoCo viewer opening) and resumes
        at `t`: samples it missed are not missing data, so the rate is not judged until a full window was watched."""
        self._watching_since = t

    def add_diag(self, t, upper_completions, fore_completions):
        self._diag = (t, {"upper_arm": upper_completions, "forearm": fore_completions})

    def report(self, now, ignore=()):
        """ignore: sensor names deliberately absent (run_demo_live --optional-sensors) -- never counted as faulty."""
        problems = []
        slow = []
        if self._last_t is None or now - self._last_t > NO_DATA_AFTER_S:
            for name in SENSORS:
                if name not in ignore:
                    problems.append(Problem(name, "no_data", "no line from the board"))
            return Report(not problems, problems)
        for name in SENSORS:
            if name in ignore:
                continue
            q = [v for t, v in self._samples[name] if t >= now - WINDOW_S]
            good = [v for v in q if v is not None]
            if len(good) < len(q) / 2 or not good:
                problems.append(Problem(name, "no_data", f"{len(q) - len(good)}/{len(q)} readings missing or invalid"))
                continue
            if self._diag is not None and now - self._diag[0] <= DIAG_VALID_S:
                c = self._diag[1][name]
                if c is not None and c == 0:
                    problems.append(Problem(name, "no_response", "0 completed reads in the last diag"))
            if self._runs[name][1] >= FROZEN_RUN and self._runs[name][0] is not None:
                problems.append(Problem(name, "frozen", f"{self._runs[name][1]} identical readings {self._runs[name][0]}"))
            sat = sum(1 for v in good if max(abs(c) for c in v) >= SATURATION_G)
            if sat >= SATURATED_SHARE * len(good):
                problems.append(Problem(name, "saturated", f"{sat}/{len(good)} readings at full scale"))
            mags = sorted(math.sqrt(sum(c * c for c in v)) for v in good)
            median = mags[len(mags) // 2]
            if not MAGNITUDE_OK_G[0] <= median <= MAGNITUDE_OK_G[1]:
                problems.append(Problem(name, "magnitude", f"median |a| = {median:.2f} g"))
            if self._watching_since is not None and now - self._watching_since < WINDOW_S:
                pass                                              # not watched a full window since a gap: no rate verdict
            elif len(good) < MIN_SAMPLES_TO_JUDGE:
                problems.append(Problem(name, "not_enough_data", f"最近 1 秒只收到 {len(good)} 筆"))
            elif len(good) < MIN_SAMPLES:
                # 2026-09-28: live, plausible readings arriving slowly are still usable -- a warning, not a fault
                slow.append(Problem(name, "slow_data", f"最近 1 秒只收到 {len(good)} 筆"))
            pwr = self._pwr.get(name)
            if pwr is not None and pwr & 0x40:
                problems.append(Problem(name, "asleep", f"PWR_MGMT_1 = 0x{pwr:02x}"))
        warnings = [Problem(name, kind, detail)
                    for name in SENSORS if name not in ignore
                    for kind, (t_ev, detail) in sorted(self._hw_events[name].items())
                    if now - t_ev <= HARDWARE_WARNING_S]
        return Report(not problems, problems, slow + warnings)


ANNOUNCE_TOO_LITTLE_DATA_AFTER_S = 2.0


def should_announce(report, elapsed_s=None):
    """Should a waiting tool say something now? Any real problem: yes. Only 'not enough data': not in the first couple of
    seconds (data is still arriving), but after that yes -- a trickle of data is itself a symptom (2026-09-28: the wait
    stayed silent while a failing sensor kept the bus recovering and data came in at a quarter of the normal rate)."""
    if report.ok:
        return False
    if any(p.kind != "not_enough_data" for p in report.problems):
        return True
    return elapsed_s is None or elapsed_s >= ANNOUNCE_TOO_LITTLE_DATA_AFTER_S


def ready_to_start(report, elapsed_s):
    """May a waiting tool start? The data must be trustworthy (report.ok). A merely SLOW rate is accepted only after
    ANNOUNCE_TOO_LITTLE_DATA_AFTER_S: right after start every stream looks slow for a moment."""
    if not report.ok:
        return False
    slow = any(w.kind == "slow_data" for w in report.warnings)
    return not slow or elapsed_s >= ANNOUNCE_TOO_LITTLE_DATA_AFTER_S


def format_warning(report):
    lines = []
    if not report.ok:
        lines.append("⚠ 硬體異常:感測器資料不可信,畫面/手臂的動作會是錯的——請先檢查硬體。")
        for p in report.problems:
            label, addr = SENSORS[p.sensor]
            lines.append(f"   - {label} MPU6050({addr}):{KIND_TEXT[p.kind]}  [{p.detail}]")
    shown = [w for w in report.warnings if w.kind not in QUIET_KINDS]     # a slow rate alone is not worth a notice
    if shown:
        lines.append("⚠ 硬體注意:感測器有狀況(資料仍在更新,但可能不準)——請檢查接線。")
        for w in shown:
            label, addr = SENSORS[w.sensor]
            lines.append(f"   - {label} MPU6050({addr}):{KIND_TEXT[w.kind]}  [{w.detail}]")
    return "\n".join(lines) if lines else "感測器狀態正常。"


def implausible_problems(upper, fore, ignore=()):
    """Problems for the sensors whose ONE current reading fails plausible() (for short windows such as a calibration
    capture, too short for HealthMonitor's one-second view)."""
    out = []
    for name, v in (("upper_arm", upper), ("forearm", fore)):
        if name not in ignore and not plausible(v):
            out.append(Problem(name, "implausible", f"reading {tuple(round(c, 4) for c in v) if _valid(v) else v}"))
    return out


QUIET_KINDS = {"slow_data"}          # the rate alone does not matter for a prototype demo (user, 2026-09-28)


def passes_pre_use_check(report):
    """check_hardware_ready --sensors: passes when the data is right (report.ok). Option A (user, 2026-09-28, breadboard
    prototype): drop-outs that recover and resets the firmware re-woke do NOT fail it -- they are reported as notes
    naming the sensor; a slow rate is only mentioned."""
    return report.ok


class WarningPrinter:
    """Which hardware-warning text to print now (or None). A NEW kind of trouble is said at once; the same trouble
    continuing is summarised every `repeat_s` (30 s: a flaky-but-recovering breadboard link must not flood the screen);
    a slow rate alone is never printed; one line when it all clears. Shared by both MuJoCo paths."""

    def __init__(self, repeat_s=30.0):
        self.repeat_s, self._key, self._next = repeat_s, (), 0.0

    def update(self, report, now):
        shown = [w for w in report.warnings if w.kind not in QUIET_KINDS]
        key = tuple(sorted((w.sensor, w.kind) for w in shown))
        new = set(key) - set(self._key)
        if key and (new or now >= self._next):
            self._key, self._next = key, now + self.repeat_s
            return format_warning(Report(True, [], shown))
        if not key and self._key:
            self._key = ()
            return "✓ 感測器沒有再斷線。"
        self._key = key if key else self._key
        return None


POLL_GAP_S = 0.25   # a longer pause between polls means the caller was blocked, not that the board went quiet


class LatestSamplePoller:
    """Feeds a HealthMonitor from a run_demo_live.LatestSample without changing that class: a new sample is recognised
    by `last_update_monotonic` changing (so polling at 1 kHz a 100 Hz stream does not look like repeats), and the
    firmware's completions are read from its `shoulder_completions` / `elbow_completions` attributes."""

    def __init__(self, latest, monitor):
        self.latest, self.monitor = latest, monitor
        self._seen = None
        self._diag_seen = None
        self._diag_text_seen = None
        self._last_poll = None

    def poll(self, now):
        # 2026-09-28 (v1.1.0 hardware test): the caller's loop can block (viewer opening/dragged) -- that is a gap in the
        # watching, not in the data, and must not read as "almost no data"
        if self._last_poll is not None and now - self._last_poll > POLL_GAP_S:
            self.monitor.observation_gap(now)
        self._last_poll = now
        stamp = getattr(self.latest, "last_update_monotonic", None)
        if stamp is not None and stamp != self._seen:
            self._seen = stamp
            self.monitor.add_sample(now, self.latest.snapshot_shoulder_raw(), self.latest.snapshot_elbow_raw())
        text = getattr(self.latest, "last_diag_text", None)
        if text is not None and text is not self._diag_text_seen:
            self._diag_text_seen = text
            self.monitor.add_hardware(now, parse_diag_line(text))
        diag = (getattr(self.latest, "shoulder_completions", None), getattr(self.latest, "elbow_completions", None))
        if diag != (None, None) and (diag != self._diag_seen or self.monitor._diag is None):
            self._diag_seen = diag
            self.monitor.add_diag(now, *diag)
