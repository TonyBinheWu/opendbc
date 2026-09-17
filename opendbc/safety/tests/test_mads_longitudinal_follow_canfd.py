#!/usr/bin/env python3

from opendbc.safety import ALTERNATIVE_EXPERIENCE
from opendbc.safety.tests.test_hyundai_canfd import TestHyundaiCanfdLKASteeringLongEV


class TestMadsLongitudinalFollowCanfd(TestHyundaiCanfdLKASteeringLongEV):
  """Safety-layer checks for MADS lead-distance brake-only longitudinal authority."""

  def setUp(self):
    super().setUp()
    self.safety.set_controls_allowed(False)
    self.safety.mads_apply_alternative_experience(
      ALTERNATIVE_EXPERIENCE.ENABLE_MADS | ALTERNATIVE_EXPERIENCE.MADS_LONGITUDINAL_FOLLOW
    )
    self.safety.set_heartbeat_engaged_mads(True)
    self.safety.set_controls_allowed_lateral(True)

  def test_follow_allows_decel_only(self):
    self.assertTrue(self._tx(self._accel_msg(0.0)))
    self.assertTrue(self._tx(self._accel_msg(-0.1)))
    self.assertTrue(self._tx(self._accel_msg(-3.5)))

    self.assertFalse(self._tx(self._accel_msg(0.01)))
    self.assertFalse(self._tx(self._accel_msg(1.0)))
    self.assertFalse(self._tx(self._accel_msg(-3.51)))

  def test_follow_requires_mads_lateral_authority(self):
    self.safety.set_controls_allowed_lateral(False)
    self.assertFalse(self._tx(self._accel_msg(-0.1)))
    self.assertTrue(self._tx(self._accel_msg(0.0)))

  def test_follow_requires_mads_heartbeat(self):
    self.safety.set_heartbeat_engaged_mads(False)
    self.assertFalse(self._tx(self._accel_msg(-0.1)))
    self.assertTrue(self._tx(self._accel_msg(0.0)))

  def test_follow_yields_to_driver_gas(self):
    self._rx(self._user_gas_msg(True))
    self.assertFalse(self._tx(self._accel_msg(-0.1)))
    self.assertTrue(self._tx(self._accel_msg(0.0)))

  def test_follow_yields_to_driver_brake(self):
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self._tx(self._accel_msg(-0.1)))
    self.assertTrue(self._tx(self._accel_msg(0.0)))
