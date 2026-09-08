#!/usr/bin/env python3
from opendbc.testing import parameterized_class
import unittest
import numpy as np

from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.hyundai_common import HyundaiButtonBase, HyundaiLongitudinalBase

# All combinations of radar/camera-SCC and gas/hybrid/EV cars
ALL_GAS_EV_HYBRID_COMBOS = [
  # Radar SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 0, "SAFETY_PARAM": 0},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS},
  # Camera SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS | HyundaiSafetyFlags.CAMERA_SCC},
]


class TestHyundaiCanfdBase(HyundaiButtonBase, common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0]]
  STANDSTILL_THRESHOLD = 12  # 0.375 kph
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}

  MAX_RATE_UP = 2
  MAX_RATE_DOWN = 3
  MAX_TORQUE_LOOKUP = [0], [270]

  MAX_RT_DELTA = 112

  DRIVER_TORQUE_ALLOWANCE = 250
  DRIVER_TORQUE_FACTOR = 2

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 89
  MAX_INVALID_STEERING_FRAMES = 2

  PT_BUS = 0
  SCC_BUS = 2
  STEER_BUS = 0
  STEER_MSG = ""
  GAS_MSG = ("", "")
  BUTTONS_TX_BUS = 1

  def _torque_driver_msg(self, torque):
    values = {"MDPS_StrTqSnsrVal": torque}
    return self.packer.make_can_msg_safety("MDPS", self.PT_BUS, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"StrTqReqVal": torque, "ActToiSta": steer_req}
    return self.packer.make_can_msg_safety(self.STEER_MSG, self.STEER_BUS, values)

  def _speed_msg(self, speed):
    values = {f"WHL_Spd{pos}Val": speed * 0.03125 for pos in ["FL", "FR", "RL", "RR"]}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", self.PT_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"DriverBraking": brake}
    return self.packer.make_can_msg_safety("TCS", self.PT_BUS, values)

  def _user_gas_msg(self, gas):
    values = {self.GAS_MSG[1]: gas}
    return self.packer.make_can_msg_safety(self.GAS_MSG[0], self.PT_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACCMode": 1 if enable else 0}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.SCC_BUS, values)

  def _button_msg(self, buttons, main_button=0, bus=None):
    if bus is None:
      bus = self.PT_BUS
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", bus, values)

  def _acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.SCC_BUS, values)

  def _lkas_button_msg(self, enabled):
    values = {"LDA_BTN": enabled}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", self.PT_BUS, values)

  def _main_cruise_button_msg(self, enabled):
    return self._button_msg(0, enabled)


class TestHyundaiCanfdLFASteeringBase(TestHyundaiCanfdBase):

  TX_MSGS = [[0x12A, 0], [0x1A0, 1], [0x1CF, 0], [0x1E0, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0)}  # LFA, LFAHDA_CLUSTER
  FWD_BLACKLISTED_ADDRS = {2: [0x12A, 0x1E0]}

  STEER_MSG = "LFA"
  BUTTONS_TX_BUS = 2
  SAFETY_PARAM: int

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ in ("TestHyundaiCanfdLFASteering", "TestHyundaiCanfdLFASteeringAltButtons"):
      cls.packer = None
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.SAFETY_PARAM)
    self.safety.init_tests()


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteering(TestHyundaiCanfdLFASteeringBase):
  pass


class TestHyundaiCanfdLFASteeringAltButtonsBase(TestHyundaiCanfdLFASteeringBase):

  SAFETY_PARAM: int

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_ALT_BUTTONS | self.SAFETY_PARAM)
    self.safety.init_tests()

  def _button_msg(self, buttons, main_button=0, bus=1):
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS, values)

  def _lkas_button_msg(self, enabled):
    values = {"LDA_BTN": enabled}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS, values)

  def _acc_cancel_msg(self, cancel, accel=0):
    values = {"ACCMode": 4 if cancel else 0, "aReqRaw": accel, "aReqValue": accel}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def test_button_sends(self):
    """
      No button send allowed with alt buttons.
    """
    for enabled in (True, False):
      for btn in range(8):
        self.safety.set_controls_allowed(enabled)
        self.assertFalse(self._tx(self._button_msg(btn)))

  def test_acc_cancel(self):
    # FIXME: the CANFD_ALT_BUTTONS cars are the only ones that use SCC_CONTROL to cancel, why can't we use buttons?
    for enabled in (True, False):
      self.safety.set_controls_allowed(enabled)
      self.assertTrue(self._tx(self._acc_cancel_msg(True)))
      self.assertFalse(self._tx(self._acc_cancel_msg(True, accel=1)))
      self.assertFalse(self._tx(self._acc_cancel_msg(False)))


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringAltButtons(TestHyundaiCanfdLFASteeringAltButtonsBase):
  pass


class TestHyundaiCanfdLKASteeringEV(TestHyundaiCanfdBase):

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4)}  # LKAS, CAM_0x2A4
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS)
    self.safety.init_tests()


# TODO: Handle ICE and HEV configurations once we see cars that use the new messages
class TestHyundaiCanfdLKASteeringAltEV(TestHyundaiCanfdBase):

  TX_MSGS = [[0x110, 0], [0x1CF, 1], [0x362, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x110, 0x362)}  # LKAS_ALT, CAM_0x362
  FWD_BLACKLISTED_ADDRS = {2: [0x110, 0x362]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS_ALT"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS |
                                 HyundaiSafetyFlags.CANFD_LKA_STEER_MSG_ALT)
    self.safety.init_tests()


class TestHyundaiCanfdLKASteeringLongEV(HyundaiLongitudinalBase, TestHyundaiCanfdLKASteeringEV):

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0], [0x51, 0], [0x730, 1], [0x12a, 1], [0x160, 1],
             [0x1e0, 1], [0x1a0, 1], [0x1ea, 1], [0x200, 1], [0x345, 1], [0x1da, 1]]

  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4), 1: (0x1a0,)}  # LKAS, CAM_0x2A4, SCC_CONTROL

  DISABLED_ECU_UDS_MSG = (0x730, 1)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 1)

  STEER_MSG = "LFA"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")
  STEER_BUS = 1

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG |
                                 HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.EV_GAS)
    self.safety.init_tests()

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def _tx_acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)


# Tests longitudinal for ICE, hybrid, EV cars with LFA steering
class TestHyundaiCanfdLFASteeringLongBase(HyundaiLongitudinalBase, TestHyundaiCanfdLFASteeringBase):

  FWD_BLACKLISTED_ADDRS = {2: [0x12a, 0x1e0, 0x1a0, 0x160]}

  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0, 0x1a0, 0x160)}  # LFA, LFAHDA_CLUSTER, SCC_CONTROL, ADRV_0x160

  DISABLED_ECU_UDS_MSG = (0x7D0, 0)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 0)

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLongBase":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.LONG | self.SAFETY_PARAM)
    self.safety.init_tests()

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def _tx_acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  # no knockout
  def test_tester_present_allowed(self):
    pass


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringLong(TestHyundaiCanfdLFASteeringLongBase):
  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLong":
      cls.safety = None
      raise unittest.SkipTest


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringLongAltButtons(TestHyundaiCanfdLFASteeringLongBase, TestHyundaiCanfdLFASteeringAltButtonsBase):
  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLongAltButtons":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.CANFD_ALT_BUTTONS | self.SAFETY_PARAM)
    self.safety.init_tests()

  def test_acc_cancel(self):
    # Alt buttons does not use SCC_CONTROL to cancel if longitudinal
    pass


class HyundaiCanfdDynamicTorqueBase:
  MAX_TORQUE_LOOKUP = [11., 13., 17.], [384, 350, 270]
  DYNAMIC_MAX_TORQUE = True
  STANDSTILL_THRESHOLD = 12 * 0.03125 / 3.6
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")
  SAFETY_PARAM = HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CAMERA_SCC

  def setUp(self):
    super().setUp()
    param = self.safety.get_current_safety_param() | HyundaiSafetyFlags.CANFD_DYNAMIC_TORQUE
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, param)
    self.safety.init_tests()

  def _speed_msg(self, speed):
    # Existing Hyundai tests use raw wheel-speed counts; dynamic tests use m/s.
    return super()._speed_msg(speed * 3.6 / 0.03125)

  @staticmethod
  def _controller_speed_for_test(speed):
    wheel_count = round(speed * 3.6 / 0.03125)
    v_ego_raw = float(np.float32(wheel_count * 0.03125 / 3.6))
    return float(np.ceil(v_ego_raw * 1000.)) / 1000.

  def _get_max_torque(self, speed):
    # Match the DBC wheel-speed quantization, Float32 CarState field, and the
    # controller's conservative upper 0.001 m/s bin.
    controller_speed = self._controller_speed_for_test(speed)
    return int(float(np.interp(controller_speed, self.MAX_TORQUE_LOOKUP[0], self.MAX_TORQUE_LOOKUP[1])) + 0.5)

  @staticmethod
  def _get_rate_limits(speed):
    controller_speed = HyundaiCanfdDynamicTorqueBase._controller_speed_for_test(speed)
    rate_up = int(float(np.interp(controller_speed, [11., 13.], [10., 2.])) + 0.5)
    rate_down = int(float(np.interp(controller_speed, [11., 13.], [10., 3.])) + 0.5)
    return rate_up, rate_down

  # The common safety tests assume fixed rate and RT limits. Exercise the same
  # invariants against every section of this speed-dependent profile instead.
  def test_steer_safety_check(self):
    for speed in self._torque_speed_range:
      self._reset_safety_hooks()
      self.safety.init_tests()
      self._reset_speed_measurement(speed)
      max_torque = self._get_max_torque(speed)
      for enabled in (False, True):
        for torque in (-max_torque - 1, -max_torque, 0, max_torque, max_torque + 1):
          self.safety.set_controls_allowed(enabled)
          self._set_prev_torque(torque)
          expected = torque == 0 or (enabled and abs(torque) <= max_torque)
          self.assertEqual(self._tx(self._torque_cmd_msg(torque)), expected, (speed, enabled, torque))

  def test_non_realtime_limit_up(self):
    for speed in (0., 11., 11.5, 12., 12.5, 13., 17., 30.):
      rate_up, _ = self._get_rate_limits(speed)
      self._reset_safety_hooks()
      self.safety.init_tests()
      self._reset_speed_measurement(speed)
      self._reset_torque_driver_measurement(0)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._set_prev_torque(sign * 100)
        self.assertTrue(self._tx(self._torque_cmd_msg(sign * (100 + rate_up))), (speed, rate_up))
        self.safety.set_controls_allowed(True)
        self._set_prev_torque(sign * 100)
        self.assertFalse(self._tx(self._torque_cmd_msg(sign * (101 + rate_up))), (speed, rate_up))

  def test_against_torque_driver(self):
    for speed in (0., 12., 13., 17., 30.):
      max_torque = self._get_max_torque(speed)
      _, rate_down = self._get_rate_limits(speed)
      self._reset_safety_hooks()
      self.safety.init_tests()
      self._reset_speed_measurement(speed)
      for sign in (-1, 1):
        for driver_torque in (self.DRIVER_TORQUE_ALLOWANCE, self.DRIVER_TORQUE_ALLOWANCE + 1):
          self._reset_torque_driver_measurement(-driver_torque * sign)
          self.safety.set_controls_allowed(True)
          self._set_prev_torque(max_torque * sign)
          self.assertEqual(self._tx(self._torque_cmd_msg(max_torque * sign)),
                           driver_torque == self.DRIVER_TORQUE_ALLOWANCE, (speed, driver_torque))

        opposing_driver = int(max_torque / self.DRIVER_TORQUE_FACTOR + self.DRIVER_TORQUE_ALLOWANCE + 1)
        self._reset_torque_driver_measurement(-opposing_driver * sign)
        self.safety.set_controls_allowed(True)
        self._set_prev_torque(max_torque * sign)
        self.assertTrue(self._tx(self._torque_cmd_msg((max_torque - rate_down) * sign)), (speed, rate_down))
        self.safety.set_controls_allowed(True)
        self._set_prev_torque(max_torque * sign)
        self.assertFalse(self._tx(self._torque_cmd_msg((max_torque - rate_down + 1) * sign)), (speed, rate_down))

  def test_realtime_limits(self):
    for speed, max_rt_delta in ((0., 270), (17., 112), (30., 112)):
      for sign in (-1, 1):
        self._reset_safety_hooks()
        self.safety.init_tests()
        self._reset_speed_measurement(speed)
        self._reset_torque_driver_measurement(0)
        self.safety.set_controls_allowed(True)
        self._set_prev_torque(0)
        for torque in range(max_rt_delta + 1):
          self.assertTrue(self._tx(self._torque_cmd_msg(sign * torque)), (speed, torque))
        self.assertFalse(self._tx(self._torque_cmd_msg(sign * (max_rt_delta + 1))), (speed, max_rt_delta))

  def test_dynamic_torque_boundaries(self):
    for speed, maximum in ((0., 384), (11., 384), (12., 367), (12.5, 359), (13., 350), (13.1, 348), (13.4, 342),
                           (14., 330), (15., 310), (16., 290), (17., 270), (18., 270), (30., 270)):
      self._reset_speed_measurement(speed)
      for sign in (-1, 1):
        for torque in (maximum, maximum + 1):
          self.safety.set_controls_allowed(True)
          self._set_prev_torque(sign * torque)
          assert self._tx(self._torque_cmd_msg(sign * torque)) == (torque == maximum)

  def test_dynamic_torque_flag_reset(self):
    param = self.safety.get_current_safety_param() & ~HyundaiSafetyFlags.CANFD_DYNAMIC_TORQUE
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, param)
    self.safety.init_tests()
    for speed in (0., 11., 13., 17.):
      self._reset_speed_measurement(speed)
      for torque in (-384, -271, -270, 270, 271, 384):
        self.safety.set_controls_allowed(True)
        self._set_prev_torque(torque)
        assert self._tx(self._torque_cmd_msg(torque)) == (abs(torque) <= 270)

  def test_dynamic_torque_with_mads(self):
    # sunnypilot can allow lateral control while ACC is disengaged. The same
    # speed-dependent ceiling must still be enforced in that state.
    self.safety.set_mads_params(True, False, False)
    for speed, maximum in ((0., 384), (12., 367), (15., 310), (17., 270)):
      self._reset_speed_measurement(speed)
      for sign in (-1, 1):
        for torque in (maximum, maximum + 1):
          self.safety.set_controls_allowed(False)
          self.safety.set_controls_allowed_lateral(True)
          self._set_prev_torque(sign * torque)
          assert self._tx(self._torque_cmd_msg(sign * torque)) == (torque == maximum)
      self.safety.set_controls_allowed_lateral(False)
      self._set_prev_torque(1)
      assert not self._tx(self._torque_cmd_msg(1))

  def test_dynamic_torque_wheel_speed_quantization(self):
    # Four independently quantized wheels can average to quarter-count speeds.
    for raw_sum in range(5900, 7900):
      counts = [raw_sum // 4 + (i < raw_sum % 4) for i in range(4)]
      values = {f"WHL_Spd{pos}Val": count * 0.03125 for pos, count in zip(("FL", "FR", "RL", "RR"), counts, strict=True)}
      for _ in range(common.MAX_SAMPLE_VALS):
        self._rx(self.packer.make_can_msg_safety("WHEEL_SPEEDS", self.PT_BUS, values))
      speed = sum(values.values()) / 4 / 3.6
      # CarState.vEgoRaw is serialized as Float32 before the controller reads it.
      controller_speed = float(np.ceil(float(np.float32(speed)) * 1000.)) / 1000.
      torque = int(float(np.interp(controller_speed, self.MAX_TORQUE_LOOKUP[0], self.MAX_TORQUE_LOOKUP[1])) + 0.5)
      self.safety.set_controls_allowed(True)
      self._set_prev_torque(torque)
      assert self._tx(self._torque_cmd_msg(torque)), (speed, torque, self.safety.get_vehicle_speed_min())


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdDynamicTorqueLFA(HyundaiCanfdDynamicTorqueBase, TestHyundaiCanfdLFASteeringBase):
  pass


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdDynamicTorqueLFAAltButtons(HyundaiCanfdDynamicTorqueBase, TestHyundaiCanfdLFASteeringAltButtonsBase):
  pass


class TestHyundaiCanfdDynamicTorqueLKAS(HyundaiCanfdDynamicTorqueBase, TestHyundaiCanfdLKASteeringEV):
  pass


class TestHyundaiCanfdDynamicTorqueLKASAlt(HyundaiCanfdDynamicTorqueBase, TestHyundaiCanfdLKASteeringAltEV):
  pass


class TestHyundaiCanfdDynamicTorqueLKASLong(HyundaiCanfdDynamicTorqueBase, TestHyundaiCanfdLKASteeringLongEV):
  pass


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdDynamicTorqueLFALong(HyundaiCanfdDynamicTorqueBase, TestHyundaiCanfdLFASteeringLongBase):
  pass


if __name__ == "__main__":
  unittest.main()
