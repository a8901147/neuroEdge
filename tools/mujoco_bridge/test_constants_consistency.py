"""Cross-file consistency of constants that are written down in more than one
place and must agree, but nothing else ties together:

  mearm_scene.xml  <->  run_demo_live.py MEARM_*_CTRL_RANGE  <->  mearm_pathb.py
  run_demo_live.py human-side sensor ranges  <->  firmware mearm_servo_maps.hpp
  firmware phase3_control_loop_main.cpp: which map drives which TIM3 channel

Editing one copy and forgetting another fails here instead of on the arm.
Hardware-free; parses the C++ sources as text.
"""

import re
import sys
import unittest
from pathlib import Path

import mujoco

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
sys.path.insert(0, str(HERE))
import mearm_pathb as pb  # noqa: E402
import run_demo_live as rdl  # noqa: E402

HEADER = (ROOT / "include/edgeneuro/control/mearm_servo_maps.hpp").read_text()
PHASE3 = (ROOT / "firmware/src/phase3_control_loop_main.cpp").read_text()
LIMIT_FINDER = (ROOT / "firmware/src/servo_limit_finder_4ch_main.c").read_text()


def _header_constants():
    """{name: int} of the `constexpr unsigned kXxxUs = N` pulse constants."""
    return {m[1]: int(m[2]) for m in re.finditer(r"constexpr unsigned (\w+)\s*=\s*(\d+)u", HEADER)} | {
        m[1]: int(m[2]) for m in re.finditer(r"(\w+Us)\s*=\s*(\d+)u", HEADER)}


def firmware_map(name):
    """(value_min, value_max, pulse_min, pulse_max) of `<name>_map()` in the header
    (pulse ends may be literals or the header's own named constants)."""
    m = re.search(name + r"_map\(\)\s*\{[^\n]*\n\s*static (?:const|constexpr) ServoAngleMap m\(\s*"
                  r"([-\d.]+)f,\s*([-\d.]+)f,\s*(\w+),\s*(\w+)\)", HEADER)
    assert m, f"could not parse {name}_map() in mearm_servo_maps.hpp"
    consts = _header_constants()
    pulse = lambda tok: int(tok.rstrip("u")) if tok.rstrip("u").isdigit() else consts[tok]
    return float(m[1]), float(m[2]), pulse(m[3]), pulse(m[4])


class SceneAgreesWithPythonTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = mujoco.MjModel.from_xml_path(str(rdl.MEARM_SCENE_XML))

    def ctrlrange(self, name):
        return tuple(self.model.actuator(name).ctrlrange)

    def test_run_demo_live_ctrl_ranges_equal_the_scene_actuators(self):
        for name, rng in (("base", rdl.MEARM_BASE_CTRL_RANGE), ("shoulder", rdl.MEARM_SHOULDER_CTRL_RANGE),
                          ("elbow", rdl.MEARM_ELBOW_CTRL_RANGE), ("claw", rdl.MEARM_CLAW_CTRL_RANGE)):
            with self.subTest(name):
                for a, b in zip(self.ctrlrange(name), rng):
                    self.assertAlmostEqual(a, b, places=6)

    def test_actuator_ctrlrange_equals_its_joint_range(self):
        for name in ("base", "shoulder", "elbow", "claw"):
            with self.subTest(name):
                for a, b in zip(self.ctrlrange(name), self.model.joint(name).range):
                    self.assertAlmostEqual(a, b, places=6)

    def test_pathb_endpoints_are_the_scene_actuator_limits(self):
        for name, (lo, hi) in (("shoulder", (pb.SHOULDER_RAISED, pb.SHOULDER_REST)),
                               ("elbow", (pb.ELBOW_EXTENDED, pb.ELBOW_FOLDED)),
                               ("base", (-pb.BASE_LIMIT, pb.BASE_LIMIT))):
            with self.subTest(name):
                rng = self.ctrlrange(name)
                self.assertAlmostEqual(min(lo, hi), rng[0], places=6)
                self.assertAlmostEqual(max(lo, hi), rng[1], places=6)

    def test_pathb_rest_pose_is_inside_every_actuator_range(self):
        for name, v in zip(("base", "shoulder", "elbow"), pb.REST_CTRL):
            with self.subTest(name):
                lo, hi = self.ctrlrange(name)
                self.assertTrue(lo <= v <= hi, f"{name}: {v} outside {lo}..{hi}")


class LimitFinderStartsAtTheChosenRestPoseTest(unittest.TestCase):
    """servo_limit_finder_4ch is the firmware the pose / linkage-measurement tools run against. It
    must boot every channel at the SAME rest pose phase3_control_loop uses (base/shoulder/elbow
    1500us, claw 1300us = open) -- it originally booted all four at 1475, so every measurement
    session began from a different pose than the arm's chosen start."""

    def header_rest(self):
        return [int(m[2]) for m in sorted(
            re.finditer(r"constexpr unsigned k(Base|Shoulder|Elbow|Claw)RestUs = (\d+)u", HEADER),
            key=lambda m: ("Base", "Shoulder", "Elbow", "Claw").index(m[1]))]

    def test_the_rest_table_equals_the_shared_rest_pulses(self):
        m = re.search(r"REST_PULSE_US\[4\]\s*=\s*\{\s*(\d+)u,\s*(\d+)u,\s*(\d+)u,\s*(\d+)u\s*\}", LIMIT_FINDER)
        self.assertIsNotNone(m, "REST_PULSE_US[4] table not found")
        self.assertEqual([int(g) for g in m.groups()], self.header_rest())
        self.assertEqual(self.header_rest(), [1500, 1500, 1500, 1300])

    def test_each_channels_first_pulse_and_starting_state_come_from_that_table(self):
        for ch in range(4):
            with self.subTest(ch):
                self.assertRegex(LIMIT_FINDER, rf"TIM3->CCR{ch + 1}\s*=\s*REST_PULSE_US\[{ch}\];")
        self.assertRegex(LIMIT_FINDER, r"uint32_t pulse_us\[4\]\s*=\s*\{\s*REST_PULSE_US\[0\],\s*REST_PULSE_US\[1\],"
                                       r"\s*REST_PULSE_US\[2\],\s*REST_PULSE_US\[3\]")

    def test_no_channel_still_boots_at_the_old_common_1475(self):
        self.assertNotIn("SERVO_CENTER_PULSE_US", LIMIT_FINDER)

    def test_every_rest_pulse_is_inside_that_channels_measured_range(self):
        ranges = [(500, 2500), (1200, 2100), (500, 1850), (1300, 1500)]
        for rest, (lo, hi) in zip(self.header_rest(), ranges):
            self.assertTrue(lo <= rest <= hi, (rest, lo, hi))


class FirmwareAgreesWithPythonTest(unittest.TestCase):
    def test_human_side_value_ranges_match_run_demo_live(self):
        # same sensor quantities feed the simulator and the firmware maps
        for fw, rng in (("base", rdl.SHOULDER_ROLL_RANGE), ("shoulder", rdl.SHOULDER_PITCH_RANGE),
                        ("elbow", rdl.ELBOW_RANGE), ("claw", (0.0, 1.0))):
            with self.subTest(fw):
                vmin, vmax, _, _ = firmware_map(fw)
                self.assertAlmostEqual(vmin, rng[0], places=3)
                self.assertAlmostEqual(vmax, rng[1], places=3)

    def test_each_channel_is_driven_by_its_own_ramp_fed_by_the_drive_command_in_the_documented_order(self):
        # channel order (user-defined, bottom-to-top): CCR1 base, CCR2 shoulder, CCR3 elbow, CCR4 claw. The values come from
        # edgeneuro::mearm::drive::command (host-tested in tests/test_mearm_drive.cpp), not from ad-hoc maps in main()
        # (2026-09-28: shoulder and elbow step TOGETHER through mearm::joint_step, so every tick stays inside the envelope)
        expected = {1: ("ramps\\.base", "base"), 4: ("ramps\\.claw", "claw")}
        for ch, (ramp, field) in expected.items():
            with self.subTest(ch):
                self.assertRegex(PHASE3, rf"TIM3->CCR{ch}\s*=\s*{ramp}\.step\(\s*static_cast<float>\(\s*cmd\.{field}\s*\)")
        # R (2026-10-03): received over UART, handed to Homing, and the command the servos get passes through it
        self.assertRegex(PHASE3, r"if \(b == \(uint8_t\)'R'\) \{\s*(//[^\n]*\n\s*)*g_home_requested = true;")
        self.assertRegex(PHASE3, r"homing\.request\(ramps\);")
        self.assertRegex(PHASE3, r"const auto cmd = homing\.apply\(")
        self.assertRegex(PHASE3, r"(\w+)\s*=\s*edgeneuro::mearm::joint_step\(\s*ramps\.shoulder,\s*ramps\.elbow,\s*"
                                 r"static_cast<float>\(\s*cmd\.shoulder\s*\),\s*static_cast<float>\(\s*cmd\.elbow\s*\)")
        self.assertRegex(PHASE3, r"TIM3->CCR2\s*=\s*se\.shoulder;")
        self.assertRegex(PHASE3, r"TIM3->CCR3\s*=\s*se\.elbow;")

    def test_the_command_uses_the_compiled_in_calibration_the_firmwares_own_raw_vector_and_elbow_reading(self):
        self.assertRegex(PHASE3, r"edgeneuro::mearm::make_compiled_calibration\(arm_cal_storage\)")
        # (2026-10-03: through the input filter -- the raw vectors in, the filtered vector and bend to the command)
        self.assertRegex(PHASE3, r"arm_filter\.update\(\{shoulder_raw_ax,\s*shoulder_raw_ay,\s*shoulder_raw_az\},\s*"
                                 r"\{elbow_raw_ax,\s*elbow_raw_ay,\s*elbow_raw_az\},\s*kServoStepDt\)")
        self.assertRegex(PHASE3, r"edgeneuro::mearm::drive::command\(\s*arm_cal,\s*filtered\.upper,\s*filtered\.elbow_bend,"
                                 r"\s*sp,\s*imu_health\.ok\(\),\s*&base_reach\)")
        self.assertRegex(PHASE3, r"edgeneuro::mearm::make_compiled_base_reach\(\*arm_cal, base_reach\)")
        self.assertRegex(PHASE3, r"sensed\.base = base_hold\.apply\(sensed\.base, edgeneuro::mearm::kBaseHysteresisUs\);")
        # (wrapped in drive::only(..., EDGENEURO_SERVO_MASK) since 2026-09-28; the default mask drives all four)
        self.assertRegex(PHASE3, r"edgeneuro::mearm::drive::only\(")
        self.assertRegex(PHASE3, r"#define EDGENEURO_SERVO_MASK 0xFu")

    def test_the_imu_health_check_is_fed_every_servo_cycle_and_a_fault_writes_nothing(self):
        # 2026-09-28: a failed sensor must stop the real arm -- the four CCR writes only happen when not holding
        self.assertRegex(PHASE3, r"imu_health\.update\(\s*\{\s*shoulder_raw_ax,\s*shoulder_raw_ay,\s*shoulder_raw_az\s*\},"
                                 r"\s*\{\s*elbow_raw_ax,\s*elbow_raw_ay,\s*elbow_raw_az\s*\}\s*\)")
        block = PHASE3[PHASE3.index("if (!cmd.hold) {"):]
        block = block[:block.index("}")]
        for ch in (1, 2, 3, 4):
            self.assertIn(f"TIM3->CCR{ch}", block)
        # optional sensors (the O<bits> command) are not health-checked
        self.assertRegex(PHASE3, r"edgeneuro::ImuHealth imu_health\(\s*g_require_shoulder_imu,\s*g_require_elbow_imu\s*\)")

    def test_the_old_per_joint_maps_no_longer_drive_the_servos(self):
        # the old maps drove shoulder/elbow independently from the firmware's own pitch/roll: what the 2026-09-26 review
        # found unsafe (opposite elbow direction, no calibration, no linkage/collision envelope)
        for old in ("kBasePulseMap.pulse_us", "kShoulderPulseMap.pulse_us", "kElbowPulseMap.pulse_us",
                    "kClawPulseMap.pulse_us"):
            self.assertNotIn(old, PHASE3)

    def test_each_ramp_is_built_from_its_own_factory(self):
        for ramp in ("base", "shoulder", "elbow", "claw"):
            with self.subTest(ramp):
                self.assertRegex(PHASE3, rf"edgeneuro::mearm::{ramp}_ramp\(\)")
        # (2026-10-03: grouped as ramps.base/.shoulder/.elbow/.claw, in this order, so mearm::drive::Homing can re-arm them)
        self.assertRegex(PHASE3, r"\}\s*ramps\{edgeneuro::mearm::base_ramp\(\),\s*edgeneuro::mearm::shoulder_ramp\(\),\s*"
                                 r"edgeneuro::mearm::elbow_ramp\(\),\s*edgeneuro::mearm::claw_ramp\(\)\}")

    def test_the_ramp_time_step_matches_the_cadence_of_the_block_that_calls_it(self):
        # the servo block runs on every 10th 1 kHz tick; a 1-tick step would make the walk 10x too fast
        self.assertRegex(PHASE3, r"kServoStepDt\s*=\s*10\.0f\s*\*\s*0\.001f")
        self.assertRegex(PHASE3, r"if \(tick_count % 10u == 0u\) \{")



class Mpu6050PowerCheckWiringTest(unittest.TestCase):
    """2026-09-28: the firmware reads each MPU6050's own PWR_MGMT_1 once a second. The data read everything depends on must
    stay exactly as it was (0x3B, 14 bytes), and the health read must use the verified side-effect-free block."""

    def test_the_data_read_defaults_are_unchanged(self):
        self.assertRegex(PHASE3, r"static constexpr uint8_t kImuRegAddr = 0x3Bu;")
        self.assertRegex(PHASE3, r"static constexpr uint32_t kImuReadLen = 14u;")
        self.assertRegex(PHASE3, r"void begin\(uint32_t start_tick, uint8_t reg = kImuRegAddr, uint32_t len = kImuReadLen\)")
        self.assertRegex(PHASE3, r"active_reader\.begin\(tick_count\);")          # the normal read passes no reg/len

    def test_the_health_read_uses_the_verified_block_and_is_never_decoded_as_motion_data(self):
        self.assertRegex(PHASE3, r"active_reader\.begin\(tick_count, edgeneuro::mpu6050::kHealthReadStartReg, "
                                 r"edgeneuro::mpu6050::kHealthReadLen\)")
        self.assertRegex(PHASE3, r"if \(completed && power_read_in_flight\) \{")
        self.assertRegex(PHASE3, r"\} else if \(completed\) \{")

    def test_a_reset_sensor_is_woken_with_the_same_value_as_at_boot(self):
        header = (ROOT / "include/edgeneuro/control/mpu6050_power_check.hpp").read_text()
        self.assertRegex(header, r"constexpr uint8_t kPwrMgmt1Awake = 0x01u;")
        for addr in ("kShoulderImuAddr", "kElbowImuAddr"):
            self.assertRegex(PHASE3, rf"mpu6050_write_reg_blocking\({addr}, 0x6Bu, 0x01u\)")   # the boot wake-up
        self.assertRegex(PHASE3, r"mpu6050_write_reg_blocking\(addr, 0x6Bu, edgeneuro::mpu6050::kPwrMgmt1Awake\)")

    def test_the_diag_line_reports_the_hardware_state(self):
        for field in ("shoulder_pwr_mgmt_1=", "elbow_pwr_mgmt_1=", "shoulder_power_resets=", "elbow_power_resets="):
            self.assertIn(field, PHASE3)

if __name__ == "__main__":
    unittest.main()
