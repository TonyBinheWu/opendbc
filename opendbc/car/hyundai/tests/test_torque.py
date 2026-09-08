import unittest

import numpy as np

from opendbc.can import CANParser
from opendbc.car import gen_empty_fingerprint
from opendbc.car.structs import CarControl, CarControlSP
from opendbc.car.hyundai.carcontroller import get_steer_max
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR
from opendbc.sunnypilot.car.interfaces import setup_interfaces
from opendbc.testing import parameterized_class


@parameterized_class([{"CAR_MODEL": c} for c in (CAR.KIA_EV6, CAR.HYUNDAI_IONIQ_5, CAR.GENESIS_GV60_EV_1ST_GEN)])
class TestHyundaiCanfdTorque(unittest.TestCase):
  CAR_MODEL = CAR.KIA_EV6

  def setUp(self):
    CP = CarInterface.get_params(self.CAR_MODEL, gen_empty_fingerprint(), [], False, False, False)
    CP_SP = CarInterface.get_params_sp(CP, self.CAR_MODEL, gen_empty_fingerprint(), [], False, False, False)
    setup_interfaces(CarInterface, CP, CP_SP, [{"HkgLowSpeedTorque": "1"}])
    self.CI = CarInterface(CP, CP_SP)
    self.CC = CarControl(enabled=True, latActive=True)
    self.CC_SP = CarControlSP()
    self.CC.actuators.torque = 1.
    self.parser = CANParser("hyundai_canfd_generated", [("LFA", 100)], 0)

  def update(self):
    timestamp = self.CI.CC.frame * 10_000_000
    actuators, msgs = self.CI.apply(self.CC.as_reader(), self.CC_SP, timestamp)
    self.parser.update([timestamp, msgs])
    assert self.parser.vl["LFA"]["StrTqReqVal"] == actuators.torqueOutputCan
    return actuators

  def test_curve_and_feedback(self):
    # vEgo deliberately differs: the curve must use unfiltered wheel speed.
    self.CI.CS.out.vEgo = 50.
    for speed, maximum in ((0., 384), (11., 384), (12., 367), (12.5, 359), (13., 350), (13.1, 348), (13.4, 342),
                           (14., 330), (15., 310), (16., 290), (16.9, 272), (17., 270), (30., 270)):
      for request in (-1., -0.5, 0.5, 1.):
        with self.subTest(speed=speed, request=request):
          self.CI.CS.out.vEgoRaw = speed
          self.CC.actuators.torque = request
          expected = round(request * maximum)
          self.CI.CC.apply_torque_last = expected
          actuators = self.update()
          assert actuators.torqueOutputCan == expected
          self.assertAlmostEqual(actuators.torque, expected / maximum, places=6)

  def test_driver_torque_limit_uses_dynamic_maximum(self):
    for speed, maximum in ((0., 384), (12., 367), (13., 350), (15., 310), (17., 270)):
      for sign in (-1, 1):
        self.CI.CS.out.vEgoRaw = speed
        self.CI.CS.out.steeringTorque = -sign * 260
        self.CC.actuators.torque = sign
        self.CI.CC.apply_torque_last = sign * maximum
        for _ in range(10):
          actuators = self.update()
        assert actuators.torqueOutputCan == sign * (maximum - 20)

  def test_rates_and_disengagement(self):
    for request in (1., -1.):
      self.CC.actuators.torque = request
      for _ in range(400):
        previous = self.CI.CC.apply_torque_last
        actuators = self.update()
        current = actuators.torqueOutputCan
        if current * previous >= 0 and abs(current) > abs(previous):
          assert abs(current - previous) <= 10
        else:
          assert abs(current - previous) <= 10
      assert current == request * 384

    self.CC.latActive = False
    assert self.update().torqueOutputCan == 0
    assert self.parser.vl["LFA"]["ActToiSta"] == 0

  def test_speed_transition(self):
    self.CI.CC.apply_torque_last = 384
    for step in range(601):
      speed = 11. + step / 100
      self.CI.CS.out.vEgoRaw = speed
      previous = self.CI.CC.apply_torque_last
      actuators = self.update()
      expected_max = get_steer_max(self.CI.CC.params, self.CI.CP.flags, float(np.float32(speed)))
      assert 0 <= previous - actuators.torqueOutputCan <= 10
      assert actuators.torqueOutputCan == expected_max, (speed, previous, actuators.torqueOutputCan, expected_max)
    assert actuators.torqueOutputCan == 270

  def test_rate_transition(self):
    self.CC.actuators.torque = 1.
    for speed, expected_up in ((0., 10), (11., 10), (11.5, 8), (12., 6), (12.5, 4), (13., 2), (17., 2)):
      with self.subTest(speed=speed):
        self.CI.CS.out.vEgoRaw = speed
        self.CI.CC.apply_torque_last = 100
        assert self.update().torqueOutputCan == 100 + expected_up

  def test_high_angle_fault_avoidance(self):
    self.CI.CS.out.steeringAngleDeg = 85.
    for frame in range(92):
      self.update()
      assert self.parser.vl["LFA"]["ActToiSta"] == (frame not in (89, 90))
