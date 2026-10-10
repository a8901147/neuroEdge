"""sensor_health.py: automatic IMU health checks, built from the two real failures of 2026-09-27 (SESSION_LOG):
  * upper arm dropping out: completions 0, the last reading repeated, |a| 0.21 g or 2.2 g;
  * forearm stuck: reads still 'completing' (246/s) but the value frozen at (1.999939, 0, 0) -- one axis at full scale.
A real accelerometer is never bit-for-bit constant (noise) and reads ~1 g of gravity when still."""
import math
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sensor_health as sh  # noqa: E402

UPPER, FORE = "upper_arm", "forearm"


def noisy(v, rng, sigma=0.004):
    return tuple(c + rng.gauss(0, sigma) for c in v)


def feed(mon, n, upper, fore, rng=None, dt=0.01, t0=0.0, upper_completions=250, fore_completions=250):
    """n samples at 100 Hz; a diag line (completions) once a second, like the firmware."""
    rng = rng or random.Random(1)
    t = t0
    for i in range(n):
        u = upper(i) if callable(upper) else noisy(upper, rng)
        f = fore(i) if callable(fore) else noisy(fore, rng)
        mon.add_sample(t, u, f)
        if i % 100 == 0:
            mon.add_diag(t, upper_completions, fore_completions)
        t += dt
    return t


HANG = (0.998, 0.004, 0.243)             # real 9/28 hanging reading, |g| = 1.03
FORE_OK = (0.95, 0.05, 0.30)


class HealthyTest(unittest.TestCase):
    def test_real_noisy_still_readings_are_healthy(self):
        mon = sh.HealthMonitor()
        t = feed(mon, 300, HANG, FORE_OK)
        rep = mon.report(t)
        self.assertTrue(rep.ok, rep.problems)
        self.assertEqual(rep.problems, [])

    def test_a_moving_arm_with_accelerations_is_still_healthy(self):
        # moving: |a| swings well away from 1 g for a moment -- that alone must not raise an alarm
        mon = sh.HealthMonitor()
        rng = random.Random(2)
        swing = lambda i: noisy(tuple(c * (1.0 + 0.6 * math.sin(i / 8.0)) for c in HANG), rng)
        t = feed(mon, 300, swing, FORE_OK)
        self.assertTrue(mon.report(t).ok, mon.report(t).problems)


class FaultTest(unittest.TestCase):
    def problems_for(self, sensor, rep):
        return [p for p in rep.problems if p.sensor == sensor]

    def test_the_real_frozen_forearm_is_caught_even_though_reads_still_complete(self):
        mon = sh.HealthMonitor()
        t = feed(mon, 300, HANG, lambda i: (1.999939, 0.0, 0.0), fore_completions=246)
        rep = mon.report(t)
        self.assertFalse(rep.ok)
        kinds = {p.kind for p in self.problems_for(FORE, rep)}
        self.assertIn("frozen", kinds)
        self.assertIn("saturated", kinds)
        self.assertEqual(self.problems_for(UPPER, rep), [])

    def test_the_real_dropped_upper_arm_is_caught(self):
        mon = sh.HealthMonitor()
        t = feed(mon, 300, lambda i: (0.179, 0.057, 0.093), FORE_OK, upper_completions=0)
        kinds = {p.kind for p in self.problems_for(UPPER, mon.report(t))}
        self.assertIn("no_response", kinds)
        self.assertIn("frozen", kinds)
        self.assertIn("magnitude", kinds)                    # 0.21 g is not gravity

    def test_an_implausible_but_noisy_magnitude_is_caught(self):
        mon = sh.HealthMonitor()
        t = feed(mon, 300, tuple(1.8 * c for c in HANG), FORE_OK)       # ~1.86 g, still, noisy (no axis past +-2 g)
        kinds = {p.kind for p in self.problems_for(UPPER, mon.report(t))}
        self.assertEqual(kinds, {"magnitude"})

    def test_no_data_at_all_is_caught(self):
        mon = sh.HealthMonitor()
        t = feed(mon, 100, HANG, FORE_OK)
        rep = mon.report(t + 2.0)                             # two seconds of silence
        self.assertFalse(rep.ok)
        self.assertIn("no_data", {p.kind for p in rep.problems})

    def test_a_short_silence_is_caught_while_the_window_still_holds_older_samples(self):
        mon = sh.HealthMonitor()
        t = feed(mon, 300, HANG, FORE_OK)
        rep = mon.report(t + 0.7)                             # 0.7 s of silence: 0.3 s of old samples still in the window
        self.assertIn("no_data", {p.kind for p in rep.problems})

    def test_a_missing_vector_is_caught(self):
        mon = sh.HealthMonitor()
        t = feed(mon, 300, HANG, lambda i: None)
        self.assertIn("no_data", {p.kind for p in self.problems_for(FORE, mon.report(t))})

    def test_non_finite_values_are_a_fault_not_a_crash(self):
        mon = sh.HealthMonitor()
        t = feed(mon, 300, lambda i: (math.nan, 0.0, 1.0), FORE_OK)
        rep = mon.report(t)
        self.assertFalse(rep.ok)
        self.assertEqual({p.kind for p in rep.problems if p.sensor == "upper_arm"}, {"no_data"})   # invalid = missing


class TransitionTest(unittest.TestCase):
    def test_recovery_is_reported_and_old_faults_do_not_linger(self):
        mon = sh.HealthMonitor()
        t = feed(mon, 300, HANG, lambda i: (1.999939, 0.0, 0.0))
        self.assertFalse(mon.report(t).ok)
        t = feed(mon, 300, HANG, FORE_OK, t0=t)               # re-plugged
        self.assertTrue(mon.report(t).ok, mon.report(t).problems)

    def test_a_single_repeated_value_is_not_frozen(self):
        # two identical consecutive samples happen (rounding); a freeze is a long run of them
        mon = sh.HealthMonitor()
        rng = random.Random(3)
        vals = [noisy(HANG, rng) for _ in range(300)]
        vals[50] = vals[49]
        vals[297] = vals[298] = vals[299] = vals[296]      # a short repeat right at the END: what the check looks at
        t = feed(mon, 300, lambda i: vals[i], FORE_OK)
        self.assertTrue(mon.report(t).ok, mon.report(t).problems)

    def test_the_first_moments_before_enough_data_are_not_called_healthy(self):
        mon = sh.HealthMonitor()
        t = feed(mon, 5, HANG, FORE_OK)
        rep = mon.report(t)
        self.assertFalse(rep.ok)
        self.assertIn("not_enough_data", {p.kind for p in rep.problems})


class MessageTest(unittest.TestCase):
    def test_the_warning_names_the_sensor_the_problem_and_that_the_data_cannot_be_trusted(self):
        mon = sh.HealthMonitor()
        t = feed(mon, 300, HANG, lambda i: (1.999939, 0.0, 0.0))
        text = sh.format_warning(mon.report(t))
        self.assertIn("forearm", text)
        self.assertIn("frozen", text)
        self.assertIn("cannot be trusted", text)
        self.assertIn("0x69", text)

    def test_a_healthy_report_formats_as_ok(self):
        mon = sh.HealthMonitor()
        t = feed(mon, 300, HANG, FORE_OK)
        self.assertIn("OK", sh.format_warning(mon.report(t)))


class PlausibleTest(unittest.TestCase):
    """The instant, per-sample guard: a reading no live accelerometer can produce is never applied, even before the
    monitor has seen enough of them to raise the (louder) fault."""

    def test_real_readings_including_a_moving_arm_are_plausible(self):
        for v in (HANG, FORE_OK, (0.5, 0.2, 0.7), (1.6, 0.4, 0.9)):
            self.assertTrue(sh.plausible(v), v)

    def test_the_real_faults_are_implausible(self):
        for v in ((1.999939, 0.0, 0.0), (0.179, 0.057, 0.093), (0.0, 0.0, 0.0), (-1.99994, 0.3, 0.1),
                  (math.nan, 0.0, 1.0), (math.inf, 0.0, 1.0), None, (1.0, 2.0)):
            self.assertFalse(sh.plausible(v), v)


# a real diag line from 2026-09-28 (the upper arm had been dropping out: 353 re-wakes since boot)
REAL_DIAG = ("diag shoulder_completions=246 elbow_completions=247 shoulder_nacks=0 shoulder_timeouts=0 elbow_nacks=0 "
             "elbow_timeouts=0 active_reader_state=0 shoulder_wake_result=0 elbow_wake_result=0 shoulder_required=1 "
             "elbow_required=1 bus_recovery_attempts=365 bus_recovery_freed=365 shoulder_asleep_rewakes=353 "
             "elbow_asleep_rewakes=0 shoulder_pwr_mgmt_1=1 elbow_pwr_mgmt_1=1 shoulder_power_resets=0 elbow_power_resets=0")


def diag(nacks=(0, 0), timeouts=(0, 0), rewakes=(353, 0), resets=(0, 0), pwr=(1, 1)):
    return (f"diag shoulder_completions=246 elbow_completions=247 shoulder_nacks={nacks[0]} shoulder_timeouts={timeouts[0]} "
            f"elbow_nacks={nacks[1]} elbow_timeouts={timeouts[1]} active_reader_state=0 shoulder_wake_result=0 "
            f"elbow_wake_result=0 shoulder_required=1 elbow_required=1 bus_recovery_attempts=365 bus_recovery_freed=365 "
            f"shoulder_asleep_rewakes={rewakes[0]} elbow_asleep_rewakes={rewakes[1]} shoulder_pwr_mgmt_1={pwr[0]} "
            f"elbow_pwr_mgmt_1={pwr[1]} shoulder_power_resets={resets[0]} elbow_power_resets={resets[1]}")


class TooLittleDataTest(unittest.TestCase):
    def test_slow_but_live_data_is_a_warning_not_a_fault(self):
        # 2026-09-28 raw view: 26 lines/s, every reading live (|a| 0.99 g, noisy) -- only the RATE was low (a failing
        # upper arm kept stalling the firmware). The first version called that a fault and blocked the preview: too strict.
        mon = sh.HealthMonitor()
        t = feed(mon, 60, HANG, FORE_OK, dt=0.07)                 # ~14 samples/s instead of ~34 (2026-10-03 rate)
        rep = mon.report(t)
        self.assertTrue(rep.ok, rep.problems)
        self.assertEqual({(w.sensor, w.kind) for w in rep.warnings}, {("upper_arm", "slow_data"), ("forearm", "slow_data")})
        # (kept in the report for check_hardware_ready --sensors' note; not printed as a notice -- QuietKindsInWarningTextTest)

    def test_the_real_line_rate_after_the_uart_fix_is_normal_not_slow(self):
        # 2026-10-03: the firmware now queues its UART lines instead of blocking on them; 115200 baud carries ~34 of its
        # ~346-byte lines per second (measured 34.5/s), so that -- not the old assumed 100/s -- is a healthy rate
        mon = sh.HealthMonitor()
        t = feed(mon, 100, HANG, FORE_OK, dt=1.0 / 34.0)
        rep = mon.report(t)
        self.assertTrue(rep.ok, rep.problems)
        self.assertEqual([w for w in rep.warnings if w.kind == "slow_data"], [])

    def test_almost_no_data_is_still_a_fault(self):
        mon = sh.HealthMonitor()
        t = feed(mon, 10, HANG, FORE_OK, dt=0.2)                  # 5 samples/s: too few to trust anything
        self.assertFalse(mon.report(t).ok)

    def test_the_first_moment_after_start_is_not_announced(self):
        mon = sh.HealthMonitor()
        rep = mon.report(feed(mon, 5, HANG, FORE_OK))              # 50 ms in
        self.assertFalse(sh.should_announce(rep, elapsed_s=0.05))
        self.assertTrue(sh.should_announce(rep, elapsed_s=3.0))


class ReadyToStartTest(unittest.TestCase):
    def test_a_slow_rate_is_accepted_only_after_the_first_couple_of_seconds(self):
        mon = sh.HealthMonitor()
        t = feed(mon, 60, HANG, FORE_OK, dt=0.07)
        rep = mon.report(t)
        self.assertFalse(sh.ready_to_start(rep, elapsed_s=0.5))
        self.assertTrue(sh.ready_to_start(rep, elapsed_s=2.5))

    def test_a_fault_is_never_ready_and_healthy_data_is_ready_at_once(self):
        mon = sh.HealthMonitor()
        t = feed(mon, 300, HANG, lambda i: (1.999939, 0.0, 0.0))
        self.assertFalse(sh.ready_to_start(mon.report(t), elapsed_s=99))
        mon = sh.HealthMonitor()
        t = feed(mon, 300, HANG, FORE_OK)
        self.assertTrue(sh.ready_to_start(mon.report(t), elapsed_s=0.0))


class QuietPrototypeTest(unittest.TestCase):
    """The author's priorities for a breadboard prototype (2026-09-28): what matters is that the numbers are right and the
    link is up; how FAST the data comes does not. So a slow rate is not announced, and a flaky-but-recovering link is
    said once and then summarised every 30 s -- while wrong or missing data stays loud (and holds the model)."""

    def report_with(self, warnings, ok=True, problems=()):
        return sh.Report(ok, list(problems), [sh.Problem(*w) for w in warnings])

    def test_a_slow_rate_alone_is_never_printed(self):
        p = sh.WarningPrinter()
        rep = self.report_with([("upper_arm", "slow_data", "20"), ("forearm", "slow_data", "20")])
        self.assertEqual([p.update(rep, t) for t in (0.0, 5.0, 40.0)], [None, None, None])

    def test_dropouts_are_said_once_then_summarised_every_30_seconds_not_every_3(self):
        p = sh.WarningPrinter()
        rep = self.report_with([("upper_arm", "dropouts", "17")])
        printed = [t for t in [i * 0.5 for i in range(0, 130)] if p.update(rep, t)]
        self.assertEqual(printed[0], 0.0)
        self.assertEqual(len(printed), 3)                          # at 0 s, ~30 s, ~60 s -- not 22 times
        self.assertIn("upper arm", p.update(rep, 200.0))

    def test_a_new_kind_of_trouble_is_said_at_once(self):
        p = sh.WarningPrinter()
        p.update(self.report_with([("upper_arm", "dropouts", "17")]), 0.0)
        text = p.update(self.report_with([("upper_arm", "dropouts", "17"), ("forearm", "power_reset", "1")]), 1.0)
        self.assertIsNotNone(text)
        self.assertIn("forearm", text)

    def test_the_pre_use_check_does_not_fail_on_a_slow_rate_alone(self):
        rep = self.report_with([("upper_arm", "slow_data", "20")])
        self.assertTrue(sh.passes_pre_use_check(rep))
        # option A (user, 2026-09-28): drop-outs that recover, with the data still right, pass too -- with a note
        self.assertTrue(sh.passes_pre_use_check(self.report_with([("upper_arm", "dropouts", "16")])))
        self.assertTrue(sh.passes_pre_use_check(self.report_with([("forearm", "power_reset", "1")])))
        self.assertFalse(sh.passes_pre_use_check(self.report_with([], ok=False,
                                                                   problems=[sh.Problem("forearm", "frozen", "")])))


class HardwareSignalTest(unittest.TestCase):
    """What the hardware itself reports (the firmware's diag line, 2026-09-28): an I2C read that got no answer, a device
    re-woken after that, and the device's own PWR_MGMT_1 read back. These say WHY, and they see a brief drop-out that the
    value checks cannot (the data looks fine again a moment later)."""

    def test_the_real_diag_line_parses(self):
        hw = sh.parse_diag_line(REAL_DIAG)
        self.assertEqual(hw["upper_arm"], {"nacks": 0, "timeouts": 0, "rewakes": 353, "power_resets": 0, "pwr_mgmt_1": 1})
        self.assertEqual(hw["forearm"]["rewakes"], 0)

    def test_an_older_firmware_line_without_the_new_fields_still_parses_what_it_has(self):
        old = REAL_DIAG.split(" shoulder_asleep_rewakes=")[0]
        hw = sh.parse_diag_line(old)
        self.assertEqual(hw["upper_arm"]["nacks"], 0)
        self.assertIsNone(hw["upper_arm"]["pwr_mgmt_1"])
        self.assertIsNone(sh.parse_diag_line("tick=1 grip=0.0"))

    def monitor_with(self, lines):
        """Live samples at 100 Hz with one diag line per second, like the real board."""
        mon = sh.HealthMonitor()
        t = feed(mon, 300, HANG, FORE_OK)
        for i, l in enumerate(lines):
            if i:
                t = feed(mon, 100, HANG, FORE_OK, t0=t, rng=random.Random(10 + i))
            mon.add_hardware(t, sh.parse_diag_line(l))
        return mon, t

    def test_a_steady_count_is_not_a_warning_only_an_increase_is(self):
        mon, t = self.monitor_with([diag(rewakes=(353, 0)), diag(rewakes=(353, 0))])
        rep = mon.report(t)
        self.assertTrue(rep.ok)
        self.assertEqual(rep.warnings, [])

    def test_dropouts_are_a_warning_that_names_the_sensor_but_do_not_stop_the_model(self):
        mon, t = self.monitor_with([diag(rewakes=(353, 0)), diag(nacks=(4, 0), rewakes=(357, 0))])
        rep = mon.report(t)
        self.assertTrue(rep.ok)                                    # the data itself is still usable
        self.assertEqual([(w.sensor, w.kind) for w in rep.warnings], [("upper_arm", "dropouts")])
        self.assertIn("4", rep.warnings[0].detail)
        text = sh.format_warning(rep)
        self.assertIn("upper arm", text)
        self.assertIn("dropped out", text)
        self.assertIn("maybe the breadboard", text)

    def test_a_timeout_counts_as_a_dropout_too(self):
        mon, t = self.monitor_with([diag(), diag(timeouts=(0, 2))])
        self.assertEqual([(w.sensor, w.kind) for w in mon.report(t).warnings], [("forearm", "dropouts")])

    def test_a_power_reset_is_reported_as_such(self):
        mon, t = self.monitor_with([diag(resets=(0, 0)), diag(resets=(0, 1))])
        kinds = {(w.sensor, w.kind) for w in mon.report(t).warnings}
        self.assertIn(("forearm", "power_reset"), kinds)
        self.assertIn("restarted", sh.format_warning(mon.report(t)))

    def test_the_firmwares_not_read_yet_value_is_not_mistaken_for_asleep(self):
        # 2026-09-28 bug, seen on the real board: pwr_mgmt_1=4294967295 means "not read yet"; its SLEEP bit is "set"
        hw = sh.parse_diag_line(diag(pwr=(4294967295, 4294967295)))
        self.assertIsNone(hw["upper_arm"]["pwr_mgmt_1"])
        mon, t = self.monitor_with([diag(), diag(pwr=(4294967295, 4294967295))])
        self.assertTrue(mon.report(t).ok, mon.report(t).problems)

    def test_a_sensor_reading_back_asleep_is_a_fault_not_just_a_warning(self):
        mon, t = self.monitor_with([diag(), diag(pwr=(1, 64))])
        rep = mon.report(t)
        self.assertFalse(rep.ok)
        self.assertIn(("forearm", "asleep"), {(p.sensor, p.kind) for p in rep.problems})

    def test_warnings_fade_after_a_few_quiet_seconds(self):
        mon, t = self.monitor_with([diag(), diag(nacks=(3, 0))])
        self.assertTrue(mon.report(t).warnings)
        t2 = feed(mon, 600, HANG, FORE_OK, t0=t)                      # six quiet seconds of live data
        mon.add_hardware(t2, sh.parse_diag_line(diag(nacks=(0, 0))))
        self.assertEqual(mon.report(t2).warnings, [])

    def test_a_sensor_declared_optional_gives_no_hardware_warnings(self):
        mon, t = self.monitor_with([diag(), diag(nacks=(0, 9), resets=(0, 1))])
        self.assertEqual(mon.report(t, ignore={"forearm"}).warnings, [])


class FromLatestSampleTest(unittest.TestCase):
    """Feeding from run_demo_live.LatestSample without touching it: a new sample is recognised by its
    last_update_monotonic changing, so a 1 kHz loop polling a 100 Hz stream does not look 'frozen'."""

    class FakeLatest:
        def __init__(self):
            self.last_update_monotonic = 0.0
            self.raw = HANG
            self.elbow_raw = FORE_OK
            self.shoulder_completions = 250          # plain attributes, exactly as LatestSample has them
            self.elbow_completions = 250
            self.last_diag_text = None

        def snapshot_shoulder_raw(self):
            return self.raw

        def snapshot_elbow_raw(self):
            return self.elbow_raw

    def test_polling_faster_than_the_data_rate_does_not_create_fake_freezes(self):
        latest = self.FakeLatest()
        mon = sh.HealthMonitor()
        poller = sh.LatestSamplePoller(latest, mon)
        rng = random.Random(4)
        now = 0.0
        for i in range(3000):                     # 3 s at 1 kHz polling
            now += 0.001
            if i % 10 == 0:                       # a new 100 Hz sample
                latest.raw, latest.elbow_raw = noisy(HANG, rng), noisy(FORE_OK, rng)
                latest.last_update_monotonic = now
            poller.poll(now)
        self.assertTrue(mon.report(now).ok, mon.report(now).problems)
        self.assertLess(mon.sample_count, 400)    # ~300 real samples, not 3000 repeats


class QuietKindsInWarningTextTest(unittest.TestCase):
    """2026-09-28 v1.1.0 hardware test: run_demo_live printed 'too little data [26 readings]' as a hardware notice at start and inside
    the fault warning, although a slow-but-correct rate does not matter for the demo (user) -- WarningPrinter already
    kept it quiet, format_warning did not."""

    def test_slow_data_alone_is_not_printed_as_a_notice(self):
        rep = sh.Report(True, [], [sh.Problem("upper_arm", "slow_data", "only 26 readings in the last second")])
        self.assertNotIn("too little data", sh.format_warning(rep))
        self.assertNotIn("Hardware note", sh.format_warning(rep))

    def test_a_fault_warning_lists_the_fault_and_real_notices_but_not_the_rate(self):
        rep = sh.Report(False, [sh.Problem("upper_arm", "frozen", "30 identical readings")],
                        [sh.Problem("upper_arm", "slow_data", "only 24 readings in the last second"),
                         sh.Problem("upper_arm", "dropouts", "in the last second: 705 missed reads, 1 re-wakes")])
        text = sh.format_warning(rep)
        self.assertIn(sh.KIND_TEXT["frozen"], text)
        self.assertIn(sh.KIND_TEXT["dropouts"], text)
        self.assertNotIn(sh.KIND_TEXT["slow_data"], text)


class PollingGapTest(unittest.TestCase):
    """Real false alarm, 2026-09-28 (v1.1.0 hardware test): while the MuJoCo viewer window was opening the main loop did
    not poll for a few seconds, so the monitor saw only 2 samples in its 1 s window and held the arm for 'almost no
    data' -- while the firmware's own diag line said completions=255 nacks=0. A gap in OUR polling is not a gap in the
    data: rate judgements wait until a full window has been watched again. Real absence is still caught."""

    def run_with_gap(self, rate_hz_after, resume_s):
        latest = FromLatestSampleTest.FakeLatest()
        mon = sh.HealthMonitor()
        poller = sh.LatestSamplePoller(latest, mon)
        rng = random.Random(6)
        now = 0.0
        period_after = 1.0 / rate_hz_after
        next_sample = 0.0
        for i in range(int((2.0 + 2.0 + resume_s) * 1000)):
            now = (i + 1) * 0.001
            rate_period = 0.01 if now < 2.0 else period_after
            if now >= next_sample:                        # the board keeps sending, gap or not
                latest.raw, latest.elbow_raw = noisy(HANG, rng), noisy(FORE_OK, rng)
                latest.last_update_monotonic = now
                next_sample = now + rate_period
            if not 2.0 <= now < 4.0:                      # 2 s without a single poll (the viewer opening)
                poller.poll(now)
        return mon.report(now)

    def test_a_pause_in_polling_is_not_reported_as_missing_data(self):
        for resume_s in (0.05, 0.3, 0.9):
            with self.subTest(resume_s=resume_s):
                rep = self.run_with_gap(100, resume_s)
                self.assertTrue(rep.ok, rep.problems)
                self.assertEqual([w for w in rep.warnings if w.kind == "slow_data"], [])

    def test_really_too_little_data_is_still_caught_once_a_full_window_is_watched(self):
        rep = self.run_with_gap(5, 1.5)
        self.assertEqual({(p.sensor, p.kind) for p in rep.problems},
                         {("upper_arm", "not_enough_data"), ("forearm", "not_enough_data")})


class PollerHardwareTest(unittest.TestCase):
    def test_the_poller_feeds_the_raw_diag_text_once_per_new_line(self):
        latest = FromLatestSampleTest.FakeLatest()
        mon = sh.HealthMonitor()
        poller = sh.LatestSamplePoller(latest, mon)
        rng = random.Random(5)
        now = 0.0
        for i in range(3000):
            now += 0.001
            if i % 10 == 0:
                latest.raw, latest.elbow_raw = noisy(HANG, rng), noisy(FORE_OK, rng)
                latest.last_update_monotonic = now
            if i == 1000:
                latest.last_diag_text = diag(rewakes=(353, 0))
            if i == 2000:
                latest.last_diag_text = diag(nacks=(5, 0), rewakes=(358, 0))
            poller.poll(now)
        rep = mon.report(now)
        self.assertTrue(rep.ok)
        self.assertEqual([(w.sensor, w.kind) for w in rep.warnings], [("upper_arm", "dropouts")])


    def test_one_diag_line_is_counted_once_so_its_warning_still_fades(self):
        # polling at 1 kHz sees the same diag text ~1000 times; re-feeding it would keep the warning alive forever
        latest = FromLatestSampleTest.FakeLatest()
        mon = sh.HealthMonitor()
        poller = sh.LatestSamplePoller(latest, mon)
        rng = random.Random(6)
        now = 0.0
        latest.last_diag_text = diag(rewakes=(353, 0))
        for i in range(9000):                                   # 9 s; the drop-out line arrives at 1 s, then nothing new
            now += 0.001
            if i % 10 == 0:
                latest.raw, latest.elbow_raw = noisy(HANG, rng), noisy(FORE_OK, rng)
                latest.last_update_monotonic = now
            if i == 1000:
                latest.last_diag_text = diag(nacks=(5, 0), rewakes=(358, 0))
            poller.poll(now)
        self.assertEqual(mon.report(now).warnings, [])

if __name__ == "__main__":
    unittest.main()
