import unittest
from unittest.mock import patch

from opendbc.can import CANParser
from opendbc.car import gen_empty_fingerprint, get_safety_config
from opendbc.car.car_helpers import get_car
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, CANFD_CAR, CarControllerParams, HyundaiFlags, HyundaiSafetyFlags
from opendbc.car.structs import CarControl, CarControlSP, CarParams
from opendbc.sunnypilot.car.hyundai.torque import configure_low_speed_torque, supports_low_speed_torque
from opendbc.sunnypilot.car.interfaces import setup_interfaces


class TestHkgLowSpeedTorque(unittest.TestCase):
  def test_scope_and_reset(self):
    for model in CAR:
      for msg in (None, 0x50, 0x110):
        with self.subTest(model=model, steering_msg=msg):
          fingerprint = gen_empty_fingerprint()
          if msg is not None:
            fingerprint[CanBus(None, fingerprint).CAM][msg] = 16
          CP = CarInterface.get_params(model, fingerprint, [], False, False, False)
          CP_SP = CarInterface.get_params_sp(CP, model, fingerprint, [], False, False, False)
          stock = CP.to_dict()
          stock_limits = vars(CarControllerParams(CP))
          self.assertEqual(supports_low_speed_torque(CP), model in CANFD_CAR)

          for setting in (None, "1", "0", True, False):
            enabled = setting in (True, "1") and model in CANFD_CAR
            params_list = None if setting is None else [{"HkgLowSpeedTorque": setting}]
            setup_interfaces(CarInterface, CP, CP_SP, params_list)
            self.assertEqual(bool(CP.flags & HyundaiFlags.CANFD_DYNAMIC_TORQUE), enabled)
            self.assertEqual(bool(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_TORQUE), enabled)
            if enabled:
              limits = vars(CarControllerParams(CP))
              self.assertEqual(limits["STEER_MAX"], 310)
              self.assertEqual(limits["STEER_MAX_LOOKUP"], ([9., 13., 17.], [310, 310, 270]))
              self.assertEqual({k: v for k, v in limits.items() if k not in ("STEER_MAX", "STEER_MAX_LOOKUP")},
                               {k: v for k, v in stock_limits.items() if k != "STEER_MAX"})
            else:
              self.assertEqual(CP.to_dict(), stock)
              self.assertEqual(vars(CarControllerParams(CP)), stock_limits)

  def test_reject_incompatible_configuration(self):
    self.assertFalse(supports_low_speed_torque(None))
    for change in ("angle", "dashcam", "alternate_limits", "alternate_limits_2", "no_canfd", "unknown_platform", "no_safety", "wrong_safety"):
      with self.subTest(change=change):
        CP = CarInterface.get_non_essential_params(CAR.KIA_EV6)
        configure_low_speed_torque(CP, True)
        if change == "angle":
          CP.steerControlType = CarParams.SteerControlType.angle
        elif change == "dashcam":
          CP.dashcamOnly = True
        elif change == "alternate_limits":
          CP.flags |= HyundaiFlags.ALT_LIMITS.value
        elif change == "alternate_limits_2":
          CP.flags |= HyundaiFlags.ALT_LIMITS_2.value
        elif change == "no_canfd":
          CP.flags &= ~HyundaiFlags.CANFD.value
        elif change == "unknown_platform":
          CP.carFingerprint = "UNRECOGNIZED_HKG"
        elif change == "no_safety":
          CP.safetyConfigs = []
        else:
          CP.safetyConfigs = [get_safety_config(CarParams.SafetyModel.hyundai)]
        self.assertFalse(supports_low_speed_torque(CP))
        configure_low_speed_torque(CP, True)
        self.assertFalse(CP.flags & HyundaiFlags.CANFD_DYNAMIC_TORQUE)
        self.assertTrue(all(not c.safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_TORQUE for c in CP.safetyConfigs))

  def test_other_brands_unchanged(self):
    for brand in ("toyota", "tesla", "mock"):
      CP = CarInterface.get_non_essential_params(CAR.KIA_EV6)
      CP.brand = brand
      # Brand-specific bit positions can overlap. Never clear another brand's flags.
      CP.flags |= HyundaiFlags.CANFD_DYNAMIC_TORQUE.value
      CP.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CANFD_DYNAMIC_TORQUE.value
      stock = CP.to_dict()
      self.assertFalse(supports_low_speed_torque(CP))
      configure_low_speed_torque(CP, True)
      self.assertEqual(CP.to_dict(), stock)

  def test_multi_panda(self):
    fingerprint = gen_empty_fingerprint()
    fingerprint[4][0x130] = 16
    CP = CarInterface.get_params(CAR.KIA_EV6, fingerprint, [], False, False, False)
    self.assertEqual(len(CP.safetyConfigs), 2)
    configure_low_speed_torque(CP, True)
    self.assertEqual(CP.safetyConfigs[0].safetyModel, CarParams.SafetyModel.noOutput)
    self.assertEqual(CP.safetyConfigs[0].safetyParam, 0)
    self.assertTrue(CP.safetyConfigs[1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_TORQUE)

  def test_get_car_initializes_controller_after_setting(self):
    # Exercise the production initialization path, including a new drive with the toggle off.
    for model in CANFD_CAR:
      for enabled in (True, False):
        with self.subTest(model=model, enabled=enabled):
          fingerprint = gen_empty_fingerprint()
          result = (model, fingerprint, "0" * 17, [], CarParams.FingerprintSource.can, True)
          with patch("opendbc.car.car_helpers.fingerprint", return_value=result):
            CI = get_car(None, None, None, False, False, init_params_list_sp=[{"HkgLowSpeedTorque": enabled}])
          self.assertEqual(CI.CC.params.STEER_MAX, 310 if enabled else 270)
          parser = CANParser("hyundai_canfd_generated", [("LFA", 100)], CI.CC.CAN.ECAN)
          CC, CC_SP = CarControl(enabled=True, latActive=True), CarControlSP()
          for speed, maximum in ((0., 310 if enabled else 270), (15., 290 if enabled else 270), (17., 270)):
            CI.CS.out.vEgoRaw = speed
            for sign in (-1, 1):
              CC.actuators.torque = sign
              CI.CC.apply_torque_last = sign * maximum
              timestamp = CI.CC.frame * 10_000_000
              actuators, msgs = CI.apply(CC.as_reader(), CC_SP, timestamp)
              parser.update([timestamp, msgs])
              self.assertEqual(actuators.torqueOutputCan, sign * maximum)
              self.assertEqual(parser.vl["LFA"]["StrTqReqVal"], sign * maximum)
