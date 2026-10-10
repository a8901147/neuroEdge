"""The public documents (README.md, PRD.md, firmware/README.md) must describe the system as it is implemented.

Each test reads a claim from a document AND the fact from the code, and compares them, so changing either side alone
fails here: retuning a constant means updating the sentence that states it, and editing a sentence means checking the
code. Design-level claims (what feeds the servos, the servo update rate) are checked against the firmware's structure.
Pure stdlib, no hardware. Added 2026-10-10 after a design-level audit found documents describing an older design."""
import json
import math
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
def prose(path):
    """A document with every run of whitespace collapsed to one space, so re-wrapping a paragraph never breaks a match."""
    return " ".join((ROOT / path).read_text().split())


README = prose("README.md")
PRD = prose("PRD.md")
FW_README = prose("firmware/README.md")
PHASE3 = (ROOT / "firmware/src/phase3_control_loop_main.cpp").read_text()
INPUT_FILTER = (ROOT / "include/edgeneuro/control/mearm_input_filter.hpp").read_text()
SERVO_MAPS = (ROOT / "include/edgeneuro/control/mearm_servo_maps.hpp").read_text()
IMU_HEALTH = (ROOT / "include/edgeneuro/control/imu_health.hpp").read_text()
DRIVE = (ROOT / "include/edgeneuro/control/mearm_drive.hpp").read_text()
PATHB = (ROOT / "include/edgeneuro/control/mearm_pathb.hpp").read_text()
MEARM_REAL = (ROOT / "include/edgeneuro/control/mearm_real.hpp").read_text()
RUN_DEMO_LIVE = (ROOT / "tools/mujoco_bridge/run_demo_live.py").read_text()
CAL_LINK = (ROOT / "tools/mujoco_bridge/mearm_calibration_link.py").read_text()

NUM = r"([0-9]+(?:\.[0-9]+)?)"
CORE_HZ = 16_000_000          # HSI, never reconfigured (checked below)


def doc(text, pattern, group=1):
    """The number a document states; fails if the sentence carrying it is gone."""
    m = re.search(pattern, text)
    if m is None:
        raise AssertionError(f"document no longer contains: {pattern}")
    return float(m.group(group))


def code(text, pattern, group=1):
    m = re.search(pattern, text)
    if m is None:
        raise AssertionError(f"code no longer contains: {pattern}")
    return float(m.group(group))


def phase3_servo_block():
    """The firmware's per-servo-cycle block: from the IMU health update to the four CCR writes."""
    start = PHASE3.index("imu_health.update(")
    end = PHASE3.index("TIM3->CCR4", start)
    return PHASE3[start:end]


class ParametersStatedInTheDocsTest(unittest.TestCase):
    def test_one_euro_filter(self):
        cutoff = code(INPUT_FILTER, rf"kFilterMinCutoffHz = {NUM}f")
        beta = code(INPUT_FILTER, rf"kFilterBeta = {NUM}f")
        self.assertEqual(doc(README, rf"1€ filter \({NUM} Hz min cutoff"), cutoff)
        self.assertEqual(doc(README, rf"min cutoff, β {NUM}\)"), beta)
        self.assertEqual(doc(PRD, rf"1€ filter on the raw vectors \({NUM} Hz at rest"), cutoff)

    def test_grip_debounce_and_emg_smoothing(self):
        on = code(PHASE3, rf"kOnDuration = {NUM}f")
        off = code(PHASE3, rf"kOffDuration = {NUM}f")
        alpha = code(PHASE3, rf"kEmgSmoothingAlpha = {NUM}f")
        self.assertEqual(on, off)                                  # every document states one value for both
        self.assertEqual(doc(README, rf"{NUM} ms on/off debounce") / 1000, on)
        self.assertEqual(doc(PRD, rf"grip closes after {NUM} ms above") / 1000, on)
        self.assertEqual(doc(PRD, rf"opens after {NUM} ms below") / 1000, off)
        self.assertEqual(doc(PRD, rf"2 thresholds, {NUM} ms debounce"), on * 1000)
        self.assertEqual(doc(FW_README, rf"{NUM} s debounce each way"), on)
        self.assertEqual(doc(PRD, rf"EMA, α = {NUM}"), alpha)
        self.assertEqual(doc(FW_README, rf"EMA \(α = {NUM}\)"), alpha)

    def test_base_steadiness(self):
        self.assertEqual(doc(PRD, rf"{NUM} µs hysteresis on the base"),
                         code(INPUT_FILTER, rf"kBaseHysteresisUs = {NUM}u"))
        self.assertEqual(doc(PRD, rf"slow-follow rule \({NUM} µs/s\)"),
                         code(INPUT_FILTER, rf"kRaiseFollowUsPerS = {NUM}f"))

    def test_start_up_ramp_and_start_pose(self):
        self.assertEqual(doc(PRD, rf"slow ramp \({NUM} µs/s\)"), code(SERVO_MAPS, rf"kStartRateUsPerS = {NUM}f"))
        rest = r"base/shoulder/elbow {0} µs, claw {0} µs".format(NUM)
        for joint, group in (("Base", 1), ("Shoulder", 1), ("Elbow", 1), ("Claw", 2)):
            with self.subTest(joint):
                self.assertEqual(doc(FW_README, rest, group), code(SERVO_MAPS, rf"k{joint}RestUs = {NUM}u"))

    def test_servo_limits(self):
        stated = re.search(rf"base {NUM}–{NUM} µs .*?shoulder {NUM}–{NUM}, elbow {NUM}–{NUM},\s*claw {NUM}–{NUM}",
                           FW_README, re.S)
        self.assertIsNotNone(stated, "firmware/README no longer states the four servo limits")
        values = [float(v) for v in stated.groups()]
        for i, joint in enumerate(("Base", "Shoulder", "Elbow", "Claw")):
            with self.subTest(joint):
                self.assertEqual(values[2 * i], code(SERVO_MAPS, rf"k{joint}LoUs = {NUM}u"))
                self.assertEqual(values[2 * i + 1], code(SERVO_MAPS, rf"k{joint}HiUs = {NUM}u"))
        self.assertEqual(doc(PRD, rf"azimuth over the full {NUM}–", 1), values[0])

    def test_emg_calibration(self):
        k = code(RUN_DEMO_LIVE, rf"(?m)^EMG_THRESHOLD_K = {NUM}$")
        self.assertEqual(doc(PRD, rf"relaxed mean \+ {NUM}·std"), k)
        self.assertIn("halfway between the relaxed mean and the grip threshold", PRD)
        self.assertEqual(code(RUN_DEMO_LIVE, rf"(?m)^EMG_RELEASE_FRACTION = {NUM}$"), 0.5)
        self.assertEqual(doc(PRD, rf"records {NUM} s relaxed"), code(RUN_DEMO_LIVE, rf"(?m)^EMG_RECORD_SECONDS = {NUM}$"))
        self.assertEqual(doc(PRD, rf"uses the last {NUM} s of each"),
                         code(RUN_DEMO_LIVE, rf"(?m)^EMG_SETTLE_TAIL_SECONDS = {NUM}$"))

    def test_firmware_imu_health_check(self):
        # "a magnitude outside 0.3–3 g or unchanged for 0.3 s" -- the firmware check (ImuHealth), run every servo cycle
        lo = code(IMU_HEALTH, rf"mag2 >= {NUM}f \* {NUM}f")
        hi = code(IMU_HEALTH, rf"mag2 <= {NUM}f \* {NUM}f")
        frozen_s = code(IMU_HEALTH, rf"kFrozenUpdates = {NUM};") * code(PHASE3, rf"kServoStepDt = {NUM}f \* 0\.001f") / 1000
        for text in (README, PRD):
            with self.subTest(doc=text[:20]):
                self.assertEqual(doc(text, rf"magnitude outside {NUM}–"), lo)
                self.assertEqual(doc(text, rf"magnitude outside [0-9.]+–{NUM} g"), hi)
                self.assertAlmostEqual(doc(text, rf"unchanged for {NUM} s"), frozen_s)

    def test_calibration_message_pacing(self):
        self.assertEqual(code(CAL_LINK, rf"(?m)^CHUNK_BYTES = {NUM}$"), 1)
        pause_ms = code(CAL_LINK, rf"(?m)^CHUNK_PAUSE_S = {NUM}$") * 1000
        self.assertAlmostEqual(doc(FW_README, rf"one byte every {NUM} ms"), pause_ms)
        self.assertAlmostEqual(doc(PRD, rf"\({NUM} ms per byte\)"), pause_ms)

    def test_power_check_and_transmit_queue(self):
        self.assertIn("`PWR_MGMT_1` is read back once a second", FW_README)
        self.assertRegex(PHASE3, r"HealthSchedule shoulder_power_schedule\(1000u\)")   # 1000 ticks at 1 kHz
        self.assertRegex(PHASE3, r"HealthSchedule elbow_power_schedule\(1000u\)")
        self.assertEqual(doc(FW_README, rf"`TxRing<{NUM}>`"), code(PHASE3, rf"edgeneuro::TxRing<{NUM}> g_tx"))


class TestCountsStatedInTheDocsTest(unittest.TestCase):
    def test_catch2_and_python_test_counts(self):
        cases = sum(len(re.findall(r"^\s*(?:TEMPLATE_)?TEST_CASE\s*\(", p.read_text(), re.M))
                    for p in (ROOT / "tests").glob("*.cpp"))
        functions = sum(len(re.findall(r"^\s+def test_", p.read_text(), re.M))
                        for p in [*(ROOT / "tools").glob("*.py"), *(ROOT / "tools/mujoco_bridge").glob("*.py")])
        for text in (README, PRD):
            with self.subTest(doc=text[:20]):
                self.assertEqual(doc(text, rf"{NUM} Catch2 (?:test )?cases"), cases)
                self.assertEqual(doc(text, rf"{NUM} Python test functions"), functions)


class RatesStatedInTheDocsTest(unittest.TestCase):
    def test_the_core_clock_is_never_reconfigured(self):
        # every rate below is derived from the 16 MHz HSI the docs state; no PLL or prescaler is ever written
        self.assertNotRegex(PHASE3, r"RCC->(PLLCFGR|CFGR)\s*[|&]?=")
        self.assertIn("16 MHz HSI", FW_README)

    def test_emg_is_sampled_at_1_khz(self):
        psc, arr = code(PHASE3, rf"TIM2->PSC = {NUM}u"), code(PHASE3, rf"TIM2->ARR = {NUM}u")
        self.assertEqual(CORE_HZ / (psc + 1) / (arr + 1), 1000)
        self.assertIn("EMG sampled at 1 kHz", README)

    def test_servo_commands_are_computed_at_100_hz_and_applied_at_the_50_hz_pwm_frame(self):
        # computed: the servo block runs on every 10th 1 kHz tick
        self.assertRegex(PHASE3, r"if \(tick_count % 10u == 0u\) \{")
        # applied: TIM3 period, with output-compare and auto-reload preload, so a new CCR takes effect only at the update
        psc, arr = code(PHASE3, rf"TIM3->PSC = {NUM}u"), code(PHASE3, rf"TIM3->ARR = {NUM}u")
        pwm_hz = CORE_HZ / (psc + 1) / (arr + 1)
        self.assertEqual(pwm_hz, 50)
        self.assertRegex(PHASE3, r"TIM3->CCMR1 \|= TIM_CCMR1_OC1PE \| TIM_CCMR1_OC2PE;")
        self.assertRegex(PHASE3, r"TIM3->CCMR2 \|= TIM_CCMR2_OC3PE \| TIM_CCMR2_OC4PE;")
        self.assertRegex(PHASE3, r"TIM3->CR1 \|= TIM_CR1_ARPE;")
        for text in (README, FW_README):
            with self.subTest(doc=text[:20]):
                self.assertIn("100 Hz", text)
                self.assertRegex(text, r"each 20 ms PWM frame|next 20 ms PWM update")
                self.assertRegex(text, r"new command at 50 Hz")

    def test_i2c_and_uart_rates(self):
        # standard-mode I2C: CCR = PCLK1 / (2 * f_scl) (RM0368 18.6.8); USART: BRR = f / baud, OVER8 = 0 (19.3.4)
        ccr = int(re.search(r"kI2cCcr100k = 0x([0-9A-Fa-f]+)u", PHASE3).group(1), 16)
        self.assertRegex(PHASE3, r"I2C1->CCR = kI2cCcr100k;")
        self.assertEqual(CORE_HZ / (2 * ccr), 100_000)
        self.assertIn("I2C1 SCL / SDA, 100 kHz", FW_README)
        brr = int(re.search(r"USART2->BRR = 0x([0-9A-Fa-f]+)u;", PHASE3).group(1), 16)
        self.assertLess(abs(CORE_HZ / brr - 115200) / 115200, 0.001)
        self.assertIn("115200 8N1", FW_README)

    def test_mpu6050_low_pass_is_the_5_hz_setting(self):
        # MPU6050 register map (RM-MPU-6000A-00) 4.3: DLPF_CFG = 6 is the 5 Hz accelerometer/gyro bandwidth
        self.assertEqual(code(PHASE3, r"kDlpfCfg6 = 0x0([0-9])u"), 6)
        self.assertRegex(PHASE3, r"mpu6050_write_reg_blocking\(kShoulderImuAddr, kMpu6050ConfigReg, kDlpfCfg6\)")
        self.assertRegex(PHASE3, r"mpu6050_write_reg_blocking\(kElbowImuAddr, kMpu6050ConfigReg, kDlpfCfg6\)")
        self.assertIn("DLPF 5 Hz", PRD)
        self.assertIn("DLPF at 5 Hz", README)


class DesignStatedInTheDocsTest(unittest.TestCase):
    def test_the_servos_are_driven_from_the_accelerometer_vectors_only(self):
        # docs: "Only the IMUs' accelerometers (gravity direction) are used for control; the gyroscopes are read, but no
        # control uses them" and the complementary filter is diagnostic only
        block = phase3_servo_block()
        for unused in ("raw_gx", "raw_gy", "raw_gz", "shoulder_filter", "ComplementaryFilter"):
            with self.subTest(unused):
                self.assertNotIn(unused, block)
        self.assertRegex(block, r"arm_filter\.update\(\{shoulder_raw_ax,\s*shoulder_raw_ay,\s*shoulder_raw_az\},\s*"
                                r"\{elbow_raw_ax,\s*elbow_raw_ay,\s*elbow_raw_az\}")
        self.assertIn("no control uses them", README)

    def test_the_claw_follows_the_slew_limited_grip_setpoint(self):
        # docs: EMG -> EMA -> GripStateMachine -> slew-rate limit -> claw
        self.assertRegex(PHASE3, r"grip\.update\(emg_ema, kDtPerTick\)")
        self.assertRegex(PHASE3, r"sp = setpoint\.update\(grip\.is_gripping\(\) \? 1\.0f : 0\.0f, kDtPerTick\)")
        self.assertRegex(phase3_servo_block(), r"drive::command\(\s*arm_cal,\s*filtered\.upper,\s*filtered\.elbow_bend,\s*sp,")

    def test_homing_runs_even_during_an_imu_fault(self):
        # docs: R homes the arm "even during an IMU fault"; R9 names this as its one exception. Homing::apply returns the
        # start pose with hold = false, whatever the sensors say.
        self.assertRegex(DRIVE, r"return \{kBaseRestUs, kShoulderRestUs, kElbowRestUs, kClawRestUs, false\};")
        self.assertIn("even during an IMU fault", README)
        self.assertIn("also during an IMU fault", PRD)
        self.assertIn("except that homing (`R`) still walks to the start pose", PRD)
        self.assertIn("Runs even during an IMU fault", FW_README)

    def test_base_ends_without_a_measured_reach(self):
        # docs: the base reaches its ends at 1.2x the calibrated LEFT/RIGHT azimuth; a measured reach is accepted but
        # nothing in tools/ writes one (only the consumers read base_reach_*_raw)
        self.assertRegex(MEARM_REAL, r"cal\.az_left \* pathb::kBaseLimit / pathb::kBaseSwing")
        ratio = code(PATHB, rf"kBaseLimit = {NUM}f") / code(PATHB, rf"kBaseSwing = {NUM}f")
        self.assertEqual(round(ratio, 1), doc(PRD, rf"reaching\s+the ends at {NUM}× the calibrated LEFT/RIGHT azimuth"))
        writers = [p for p in (ROOT / "tools").rglob("*.py") if not p.name.startswith("test_")
                   and re.search(r"[\"']base_reach_left_raw[\"']\s*:", p.read_text())]
        self.assertEqual(writers, [], "a tool now captures the base reach: update PRD 5")
        self.assertIn("no tool captures one yet", PRD)

    def test_only_the_imus_are_health_checked(self):
        # docs: "Only the IMUs are checked; a loose electrode can open or close the claw" -- if an EMG check is ever
        # added, this test fails as a reminder to update that limitation
        self.assertNotRegex(PHASE3, r"(?i)emg\w*health|emg\w*fault|emg\w*plausib")
        self.assertIn("EMG not health-checked", PRD)


class MeasurementsStatedInTheDocsTest(unittest.TestCase):
    """Numbers the documents derive from committed raw recordings, recomputed from those recordings."""

    # data/arm_motion_20261004-*.json: the phases that are demo motions, and the ones held still at rest
    MOTION = ("raise_forward", "raise_left_front", "raise_right_front", "check_diagonal", "relaxed_task", "grip_task")
    REST = ("relaxed_still", "grip_firm_still")

    @classmethod
    def setUpClass(cls):
        cls.phases = {}
        for path in sorted((ROOT / "data").glob("arm_motion_20261004-*.json")):
            cls.phases.update(json.loads(path.read_text())["phases"])

    def p95_deviation(self, phase, sensor):
        dev = sorted(abs(math.sqrt(sum(c * c for c in row[sensor])) - 1.0) for row in self.phases[phase])
        return dev[int(0.95 * len(dev))]

    def test_accelerometer_deviation_from_1_g(self):
        stated = r"0\.06–0\.09 g on the upper arm and\s+0\.10–0\.18 g on the forearm \(95th percentile;\s+about 0\.03 g at rest\)"
        for text in (README, PRD):
            self.assertRegex(text, stated)
        for sensor, (lo, hi) in (("upper_arm", (0.06, 0.09)), ("forearm", (0.10, 0.18))):
            p95 = [self.p95_deviation(phase, sensor) for phase in self.MOTION]
            with self.subTest(sensor):
                self.assertEqual((round(min(p95), 2), round(max(p95), 2)), (lo, hi))
                for phase in self.REST:
                    self.assertAlmostEqual(self.p95_deviation(phase, sensor), 0.03, delta=0.005)
        peak = max(abs(math.sqrt(sum(c * c for c in row["upper_arm"])) - 1.0)
                   for phase in self.MOTION for row in self.phases[phase])
        self.assertGreater(peak, 1.0)                                        # "briefly by more than 1 g"

    def test_emg_motion_artifact_peaks(self):
        # docs: a relaxed hand moving through the demo reached 3848, a firm still grip only 1318 -- each the peak of the
        # 10 ms window midpoint (emg_min + emg_max) / 2 that capture_arm_motion.emg_events replays
        def peak(phase):
            return max((r["emg"][0] + r["emg"][1]) / 2 for r in self.phases[phase] if r.get("emg"))
        self.assertEqual(round(peak("relaxed_task")), doc(README, rf"reached {NUM} ADC counts"))
        self.assertEqual(round(peak("grip_firm_still")), doc(README, rf"a firm grip held still only {NUM}"))
        self.assertEqual(round(peak("relaxed_task")), doc(PRD, rf"arm motion alone reached {NUM} ADC counts"))
        self.assertEqual(round(peak("grip_firm_still")), doc(PRD, rf"a firm still grip \({NUM}\)"))


if __name__ == "__main__":
    unittest.main()
