import unittest

from opendbc.sunnypilot.car.hyundai.torque import (
  NaturalSteeringTorqueShaper,
  get_natural_steering_deadband,
  get_natural_steering_rate,
  get_natural_steering_reversal_confirm_frames,
)


class TestNaturalSteeringTorqueShaper(unittest.TestCase):
  def test_neutral_speed_demand_curve(self):
    steer_max = 400
    # 11 m/s is the neutral profile: exponent=1 and rate scale=1.
    self.assertAlmostEqual(get_natural_steering_rate(0, steer_max, 11.0), 1.5)
    self.assertAlmostEqual(get_natural_steering_rate(60, steer_max, 11.0), 2.0)
    self.assertAlmostEqual(get_natural_steering_rate(120, steer_max, 11.0), 3.0)
    self.assertAlmostEqual(get_natural_steering_rate(200, steer_max, 11.0), 4.0)
    self.assertAlmostEqual(get_natural_steering_rate(300, steer_max, 11.0), 6.0)
    self.assertAlmostEqual(get_natural_steering_rate(400, steer_max, 11.0), 8.0)

  def test_speed_adaptive_nonlinearity(self):
    steer_max = 384
    low_speed = get_natural_steering_rate(384, steer_max, 0.0)
    city_speed = get_natural_steering_rate(384, steer_max, 11.0)
    highway_speed = get_natural_steering_rate(384, steer_max, 35.0)

    self.assertAlmostEqual(low_speed, 10.0)
    self.assertAlmostEqual(city_speed, 8.0)
    self.assertAlmostEqual(highway_speed, 3.6)
    self.assertGreater(low_speed, city_speed)
    self.assertGreater(city_speed, highway_speed)

  def test_speed_profile_is_continuous_around_breakpoint(self):
    before = get_natural_steering_rate(200, 384, 10.99)
    after = get_natural_steering_rate(200, 384, 11.01)
    self.assertLess(abs(before - after), 0.1)

  def test_high_speed_deadband_suppresses_micro_correction(self):
    shaper = NaturalSteeringTorqueShaper()
    deadband = get_natural_steering_deadband(270, 35.0)
    self.assertGreater(deadband, 3.0)

    current = shaper.update(3, 0, 270, True, 35.0)
    self.assertEqual(current, 0)
    self.assertEqual(shaper.phase, "micro_hold")

  def test_low_speed_deadband_remains_transparent(self):
    shaper = NaturalSteeringTorqueShaper()
    self.assertLess(get_natural_steering_deadband(384, 0.0), 1.0)

    current = shaper.update(3, 0, 384, True, 0.0)
    self.assertEqual(current, 1)
    self.assertEqual(shaper.phase, "turn_in")

  def test_turn_in_is_progressive_and_bounded(self):
    shaper = NaturalSteeringTorqueShaper()
    current = 0
    deltas = []
    for _ in range(20):
      next_torque = shaper.update(384, current, 384, True, 0.0)
      deltas.append(next_torque - current)
      current = next_torque

    self.assertEqual(deltas[0], 1)
    self.assertTrue(all(0 < delta <= 10 for delta in deltas))
    self.assertTrue(all(a <= b for a, b in zip(deltas[:9], deltas[1:10])))
    self.assertEqual(shaper.phase, "turn_in")

  def test_small_city_corrections_use_low_rate(self):
    shaper = NaturalSteeringTorqueShaper()
    current = 0
    for _ in range(40):
      next_torque = shaper.update(50, current, 384, True, 11.0)
      self.assertLessEqual(abs(next_torque - current), 2)
      current = next_torque
      if current == 50:
        break
    self.assertEqual(current, 50)

  def test_hold_settles_slew_rate(self):
    shaper = NaturalSteeringTorqueShaper()
    current = 0
    for _ in range(10):
      current = shaper.update(200, current, 384, True, 11.0)
    shaper.update(current, current, 384, True, 11.0)
    self.assertEqual(shaper.phase, "hold")

  def test_unwind_is_progressive(self):
    shaper = NaturalSteeringTorqueShaper()
    current = 200
    values = []
    for _ in range(60):
      current = shaper.update(0, current, 384, True, 11.0)
      values.append(current)
      if current == 0:
        break
    self.assertEqual(values[-1], 0)
    self.assertTrue(all(a >= b >= 0 for a, b in zip(values, values[1:])))
    deltas = [200 - values[0], *[a - b for a, b in zip(values, values[1:])]]
    self.assertTrue(all(delta <= 6 for delta in deltas))

  def test_reversal_confirmation_grows_with_speed(self):
    low = get_natural_steering_reversal_confirm_frames(0.0)
    city = get_natural_steering_reversal_confirm_frames(11.0)
    highway = get_natural_steering_reversal_confirm_frames(35.0)
    self.assertEqual(low, 3)
    self.assertEqual(city, 5)
    self.assertEqual(highway, 12)
    self.assertLess(low, city)
    self.assertLess(city, highway)

  def test_transient_reversal_is_debounced(self):
    shaper = NaturalSteeringTorqueShaper()
    current = 120
    confirm_frames = get_natural_steering_reversal_confirm_frames(11.0)

    for _ in range(confirm_frames - 1):
      current = shaper.update(-120, current, 384, True, 11.0)
      self.assertGreaterEqual(current, 0)
      self.assertEqual(shaper.phase, "reversal_confirm")

    current = shaper.update(120, current, 384, True, 11.0)
    self.assertGreaterEqual(current, 0)
    self.assertEqual(shaper.reversal_frames, 0)
    self.assertEqual(shaper.reversal_sign, 0)

  def test_persistent_reversal_holds_zero_until_confirmed(self):
    shaper = NaturalSteeringTorqueShaper()
    current = 12
    values = []
    confirm_frames = get_natural_steering_reversal_confirm_frames(11.0)

    for _ in range(confirm_frames):
      current = shaper.update(-120, current, 384, True, 11.0)
      values.append(current)
      self.assertGreaterEqual(current, 0)

    for _ in range(30):
      current = shaper.update(-120, current, 384, True, 11.0)
      values.append(current)
      if current < 0:
        break

    first_negative = next(i for i, value in enumerate(values) if value < 0)
    self.assertGreaterEqual(first_negative, confirm_frames)
    self.assertIn(0, values[:first_negative])

  def test_highway_reversal_is_more_damped(self):
    low = NaturalSteeringTorqueShaper()
    high = NaturalSteeringTorqueShaper()
    low_torque = 120
    high_torque = 120

    for _ in range(6):
      low_torque = low.update(-120, low_torque, 384, True, 0.0)
      high_torque = high.update(-120, high_torque, 270, True, 35.0)

    # Low speed is allowed to unload faster; highway retains more of the
    # original-side torque and has a longer reversal confirmation window.
    self.assertLess(low_torque, high_torque)
    self.assertGreater(high.reversal_frames, 0)

  def test_inactive_resets(self):
    shaper = NaturalSteeringTorqueShaper()
    current = shaper.update(300, 0, 384, True, 11.0)
    self.assertNotEqual(current, 0)
    shaper.update(-300, current, 384, True, 11.0)
    self.assertNotEqual(shaper.reversal_sign, 0)

    self.assertEqual(shaper.update(300, current, 384, False, 11.0), 0)
    self.assertEqual(shaper.phase, "inactive")
    self.assertEqual(shaper.slew_rate, 0.0)
    self.assertEqual(shaper.reversal_frames, 0)
    self.assertEqual(shaper.reversal_sign, 0)

  def test_never_exceeds_existing_maximum(self):
    shaper = NaturalSteeringTorqueShaper()
    current = 0
    for _ in range(150):
      current = shaper.update(999, current, 350, True, 0.0)
      self.assertLessEqual(abs(current), 350)


if __name__ == "__main__":
  unittest.main()
