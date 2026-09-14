import unittest

from opendbc.sunnypilot.car.hyundai.torque import NaturalSteeringTorqueShaper, get_natural_steering_rate


class TestNaturalSteeringTorqueShaper(unittest.TestCase):
  def test_demand_rate_table(self):
    steer_max = 400
    self.assertEqual(get_natural_steering_rate(0, steer_max), 3)
    self.assertEqual(get_natural_steering_rate(60, steer_max), 3)
    self.assertEqual(get_natural_steering_rate(120, steer_max), 4)
    self.assertEqual(get_natural_steering_rate(200, steer_max), 6)
    self.assertEqual(get_natural_steering_rate(300, steer_max), 8)
    self.assertEqual(get_natural_steering_rate(400, steer_max), 10)

  def test_turn_in_is_progressive_and_bounded(self):
    shaper = NaturalSteeringTorqueShaper()
    current = 0
    deltas = []
    for _ in range(20):
      next_torque = shaper.update(384, current, 384, True)
      deltas.append(next_torque - current)
      current = next_torque

    self.assertEqual(deltas[0], 1)
    self.assertTrue(all(0 < delta <= 10 for delta in deltas))
    self.assertTrue(all(a <= b for a, b in zip(deltas[:9], deltas[1:10])))
    self.assertEqual(shaper.phase, "turn_in")

  def test_small_corrections_use_low_rate(self):
    shaper = NaturalSteeringTorqueShaper()
    current = 0
    for _ in range(20):
      next_torque = shaper.update(50, current, 384, True)
      self.assertLessEqual(abs(next_torque - current), 3)
      current = next_torque
      if current == 50:
        break
    self.assertEqual(current, 50)

  def test_hold_settles_slew_rate(self):
    shaper = NaturalSteeringTorqueShaper()
    current = 0
    for _ in range(10):
      current = shaper.update(200, current, 384, True)
    shaper.update(current, current, 384, True)
    self.assertEqual(shaper.phase, "hold")

  def test_unwind_is_progressive(self):
    shaper = NaturalSteeringTorqueShaper()
    current = 200
    values = []
    for _ in range(30):
      current = shaper.update(0, current, 384, True)
      values.append(current)
      if current == 0:
        break
    self.assertEqual(values[-1], 0)
    self.assertTrue(all(a >= b >= 0 for a, b in zip(values, values[1:])))

  def test_reversal_crosses_zero_before_opposite_torque(self):
    shaper = NaturalSteeringTorqueShaper()
    current = 180
    values = []
    for _ in range(80):
      current = shaper.update(-180, current, 384, True)
      values.append(current)
      if current < 0:
        break

    first_negative = next(i for i, value in enumerate(values) if value < 0)
    self.assertIn(0, values[:first_negative])

  def test_inactive_resets(self):
    shaper = NaturalSteeringTorqueShaper()
    current = shaper.update(300, 0, 384, True)
    self.assertNotEqual(current, 0)
    self.assertEqual(shaper.update(300, current, 384, False), 0)
    self.assertEqual(shaper.phase, "inactive")
    self.assertEqual(shaper.slew_rate, 0.0)

  def test_never_exceeds_existing_maximum(self):
    shaper = NaturalSteeringTorqueShaper()
    current = 0
    for _ in range(100):
      current = shaper.update(999, current, 350, True)
      self.assertLessEqual(abs(current), 350)


if __name__ == "__main__":
  unittest.main()
