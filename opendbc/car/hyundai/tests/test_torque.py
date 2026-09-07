import unittest

from opendbc.can import CANParser
from opendbc.car import gen_empty_fingerprint
from opendbc.car.structs import CarControl, CarControlSP
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
    for speed, maximum in ((0., 310), (9., 310), (13., 310), (13.1, 309), (13.4, 306),
                           (14., 300), (15., 290), (16., 280), (16.9, 271), (17., 270), (30., 270)):
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
    for speed, maximum in ((13., 310), (15., 290), (17., 270)):
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
          assert abs(current - previous) <= 2
        else:
          assert abs(current - previous) <= 3
      assert current == request * 310

    self.CC.latActive = False
    assert self.update().torqueOutputCan == 0
    assert self.parser.vl["LFA"]["ActToiSta"] == 0

  def test_speed_transition(self):
    self.CI.CC.apply_torque_last = 310
    for step in range(401):
      self.CI.CS.out.vEgoRaw = 13. + step / 100
      previous = self.CI.CC.apply_torque_last
      actuators = self.update()
      assert 0 <= previous - actuators.torqueOutputCan <= 3
      if step % 10 == 0:
        assert actuators.torqueOutputCan == 310 - step // 10
    assert actuators.torqueOutputCan == 270

  def test_high_angle_fault_avoidance(self):
    self.CI.CS.out.steeringAngleDeg = 85.
    for frame in range(92):
      self.update()
      assert self.parser.vl["LFA"]["ActToiSta"] == (frame not in (89, 90))
