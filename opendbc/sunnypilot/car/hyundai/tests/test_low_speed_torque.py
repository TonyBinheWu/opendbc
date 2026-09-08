import unittest
from unittest.mock import patch

from opendbc.can import CANParser
from opendbc.car import gen_empty_fingerprint, get_safety_config
from opendbc.car.car_helpers import get_car
from opendbc.car.hyundai.carcontroller import get_steer_max, get_steer_rate_limits
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, CANFD_CAR, CarControllerParams, HyundaiFlags, HyundaiSafetyFlags
from opendbc.car.structs import CarControl, CarControlSP, CarParams
from opendbc.sunnypilot.car.hyundai.torque import configure_low_speed_torque, supports_low_speed_torque
from opendbc.sunnypilot.car.interfaces import setup_interfaces


class TestHkgLowSpeedTorque(unittest.TestCase):
  def test_one_setting_enables_torque_and_creep_lane_change(self):
    for model in CAR:
      with self.subTest(model=model):
        CP = CarInterface.get_non_essential_params(model)
        supported = model in CANFD_CAR

        configure_low_speed_torque(CP, True)
        self.assertEqual(bool(CP.flags & HyundaiFlags.CANFD_DYNAMIC_TORQUE), supported)
        self.assertEqual(bool(CP.flags & HyundaiFlags.CANFD_CREEP_LANE_CHANGE), supported)
        self.assertEqual(bool(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_TORQUE), supported)
        # Creep lane change is now an openpilot behavior flag, not a separate Panda torque allowance.
        self.assertTrue(all(not c.safetyParam & HyundaiSafetyFlags.CANFD_CREEP_LANE_CHANGE for c in CP.safetyConfigs))

        configure_low_speed_torque(CP, False)
        self.assertFalse(CP.flags & HyundaiFlags.CANFD_DYNAMIC_TORQUE)
        self.assertFalse(CP.flags & HyundaiFlags.CANFD_CREEP_LANE_CHANGE)
        self.assertTrue(all(not c.safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_TORQUE for c in CP.safetyConfigs))

  def test_torque_and_rate_curves(self):
    CP = CarInterface.get_non_essential_params(CAR.KIA_EV6)
    configure_low_speed_torque(CP, True)
    params = CarControllerParams(CP)

    self.assertEqual(params.STEER_MAX_LOOKUP, ([11., 13., 17.], [384, 350, 270]))
    self.assertEqual(params.STEER_DELTA_UP_LOOKUP, ([11., 13.], [10, 2]))
    self.assertEqual(params.STEER_DELTA_DOWN_LOOKUP, ([11., 13.], [10, 3]))
    for speed, expected in ((0., 384), (11., 384), (12., 367), (12.5, 359),
                            (13., 350), (14., 330), (15., 310), (16., 290),
                            (17., 270), (30., 270)):
      with self.subTest(kind="torque", speed=speed):
        self.assertEqual(get_steer_max(params, CP.flags, speed), expected)

    for speed, expected in ((0., (10, 10)), (11., (10, 10)), (11.5, (8, 8)),
                            (12., (6, 7)), (12.5, (4, 5)), (13., (2, 3)),
                            (17., (2, 3)), (30., (2, 3))):
      with self.subTest(kind="rate", speed=speed):
        self.assertEqual(get_steer_rate_limits(params, CP.flags, speed), expected)

    CP.flags &= ~HyundaiFlags.CANFD_DYNAMIC_TORQUE.value
    stock_params = CarControllerParams(CP)
    for speed in (0., 11., 13., 17., 30.):
      self.assertEqual(get_steer_max(stock_params, CP.flags, speed), 270)
      self.assertEqual(get_steer_rate_limits(stock_params, CP.flags, speed), (2, 3))

  def test_scope_and_reset(self):
    dynamic_keys = {"STEER_MAX_LOOKUP", "STEER_DELTA_UP_LOOKUP", "STEER_DELTA_DOWN_LOOKUP"}
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
            self.assertEqual(bool(CP.flags & HyundaiFlags.CANFD_CREEP_LANE_CHANGE), enabled)
            self.assertEqual(bool(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_TORQUE), enabled)
            self.assertFalse(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_CREEP_LANE_CHANGE)
            if enabled:
              limits = vars(CarControllerParams(CP))
              self.assertEqual(limits["STEER_MAX"], 384)
              self.assertEqual(limits["STEER_MAX_LOOKUP"], ([11., 13., 17.], [384, 350, 270]))
              self.assertEqual(limits["STEER_DELTA_UP_LOOKUP"], ([11., 13.], [10, 2]))
              self.assertEqual(limits["STEER_DELTA_DOWN_LOOKUP"], ([11., 13.], [10, 3]))
              self.assertEqual({k: v for k, v in limits.items() if k not in dynamic_keys | {"STEER_MAX"}},
                               {k: v for k, v in stock_limits.items() if k != "STEER_MAX"})
            else:
              self.assertEqual(CP.to_dict(), stock)
              self.assertEqual(vars(CarControllerParams(CP)), stock_limits)

  def test_reject_incompatible_configuration(self):
    self.assertFalse(supports_low_speed_torque(None))
    for change in ("angle", "dashcam", "alternate_limits", "alternate_limits_2", "no_canfd", "unknown_platform", "no_safety", "wrong_safety"):
      with self.subTest(change=change):
        CP = CarInterface.get_non_essential_params(CAR.KIA_EV6)
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
        self.assertFalse(CP.flags & HyundaiFlags.CANFD_CREEP_LANE_CHANGE)
        self.assertTrue(all(not c.safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_TORQUE for c in CP.safetyConfigs))
        self.assertTrue(all(not c.safetyParam & HyundaiSafetyFlags.CANFD_CREEP_LANE_CHANGE for c in CP.safetyConfigs))

  def test_other_brands_unchanged(self):
    for brand in ("toyota", "tesla", "mock"):
      CP = CarInterface.get_non_essential_params(CAR.KIA_EV6)
      CP.brand = brand
      CP.flags |= (HyundaiFlags.CANFD_DYNAMIC_TORQUE | HyundaiFlags.CANFD_CREEP_LANE_CHANGE).value
      CP.safetyConfigs[-1].safetyParam |= (
        HyundaiSafetyFlags.CANFD_DYNAMIC_TORQUE | HyundaiSafetyFlags.CANFD_CREEP_LANE_CHANGE).value
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
    self.assertFalse(CP.safetyConfigs[1].safetyParam & HyundaiSafetyFlags.CANFD_CREEP_LANE_CHANGE)

  def test_get_car_initializes_unified_profile(self):
    fingerprint = gen_empty_fingerprint()
    result = (CAR.KIA_EV6, fingerprint, "0" * 17, [], CarParams.FingerprintSource.can, True)
    with patch("opendbc.car.car_helpers.fingerprint", return_value=result):
      CI = get_car(None, None, None, False, False, init_params_list_sp=[{"HkgLowSpeedTorque": True}])

    self.assertEqual(CI.CC.params.STEER_MAX, 384)
    self.assertTrue(CI.CP.flags & HyundaiFlags.CANFD_CREEP_LANE_CHANGE)
    parser = CANParser("hyundai_canfd_generated", [("LFA", 100)], CI.CC.CAN.ECAN)
    CC = CarControl(enabled=True, latActive=True)
    CC.actuators.torque = 1.
    for speed, active, maximum in ((0., False, 384), (0., True, 384), (12., False, 367),
                                   (13., True, 350), (15., False, 310), (17., True, 270), (30., False, 270)):
      with self.subTest(speed=speed, active=active):
        CI.CS.out.vEgoRaw = speed
        CI.CC.apply_torque_last = maximum
        CC_SP = CarControlSP(creepLaneChangeActive=active)
        timestamp = CI.CC.frame * 10_000_000
        actuators, msgs = CI.apply(CC.as_reader(), CC_SP, timestamp)
        parser.update([timestamp, msgs])
        self.assertEqual(actuators.torqueOutputCan, maximum)
        self.assertEqual(parser.vl["LFA"]["StrTqReqVal"], maximum)

  def test_legacy_creep_setting_is_ignored(self):
    CP = CarInterface.get_non_essential_params(CAR.KIA_EV6)
    CP_SP = CarInterface.get_params_sp(CP, CAR.KIA_EV6, gen_empty_fingerprint(), [], False, False, False)
    setup_interfaces(CarInterface, CP, CP_SP, [{"HkgCreepLaneChange": True}])
    self.assertFalse(CP.flags & HyundaiFlags.CANFD_DYNAMIC_TORQUE)
    self.assertFalse(CP.flags & HyundaiFlags.CANFD_CREEP_LANE_CHANGE)


if __name__ == "__main__":
  unittest.main()
