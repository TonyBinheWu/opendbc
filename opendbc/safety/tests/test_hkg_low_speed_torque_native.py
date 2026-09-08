"""Run the unified HKG low-speed controller and real C safety hooks against encoded CAN inputs."""
import ast
import binascii
from enum import IntFlag
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np
from opendbc.safety.tests.libsafety import libsafety_py


class Flags(IntFlag):
  CANFD_DYNAMIC_TORQUE = 1 << 27


def controller_functions():
  source = Path(__file__).resolve().parents[2] / "car/hyundai/carcontroller.py"
  tree = ast.parse(source.read_text())
  nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in ("get_steer_max", "get_steer_rate_limits")]
  scope = {"np": np, "HyundaiFlags": Flags}
  exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), "exec"), scope)
  return scope["get_steer_max"], scope["get_steer_rate_limits"]


class TestHkgLowSpeedTorqueNative(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.safety = libsafety_py.libsafety
    cls.steer_max, cls.steer_rates = (staticmethod(f) for f in controller_functions())
    cls.params = SimpleNamespace(
      STEER_MAX=384,
      STEER_DELTA_UP=2,
      STEER_DELTA_DOWN=3,
      STEER_MAX_LOOKUP=([11., 13., 17.], [384, 350, 270]),
      STEER_DELTA_UP_LOOKUP=([11., 13.], [10, 2]),
      STEER_DELTA_DOWN_LOOKUP=([11., 13.], [10, 3]),
    )

  def configure(self, dynamic=True, lka=False, legacy_creep_bit=False):
    self.pt_bus = 1 if lka else 0
    self.steer_addr = 0x50 if lka else 0x12A
    self.counter = 0
    param = 1 | (16 if lka else 8) | (1024 if dynamic else 0) | (2048 if legacy_creep_bit else 0)
    self.assertEqual(self.safety.set_safety_hooks(28, param), 0)
    self.safety.init_tests()
    self.safety.set_timer(100)

  def packet(self, address, bus, data):
    if len(data) == 8:
      data[1] = (self.counter % 16) << 4
    else:
      data[2] = self.counter % 256
    self.counter += 1
    if address == 0x1CF:  # CRUISE_BUTTONS has a nibble counter but no checksum.
      return libsafety_py.make_CANPacket(address, bus, data)
    crc = binascii.crc_hqx(bytes(data[2:]) + address.to_bytes(2, "little"), 0)
    crc ^= {8: 0x5F29, 16: 0x041D, 24: 0x819D, 32: 0x9F5B}[len(data)]
    data[:2] = crc.to_bytes(2, "little")
    return libsafety_py.make_CANPacket(address, bus, data)

  def speed(self, kph):
    count = round(kph / 0.03125)
    return self.speed_counts([count] * 4)

  def speed_counts(self, counts):
    for _ in range(8):
      data = bytearray(24)
      for offset, count in zip((8, 10, 12, 14), counts, strict=True):
        data[offset:offset + 2] = count.to_bytes(2, "little")
      self.safety.safety_rx_hook(self.packet(0xA0, self.pt_bus, data))
    return sum(counts) / 4 * 0.03125 / 3.6

  def tx(self, torque, previous=None):
    self.safety.set_controls_allowed(True)
    self.safety.set_torque_driver(0, 0)
    self.safety.set_desired_torque_last(torque if previous is None else previous)
    self.safety.set_rt_torque_last(torque)
    data = bytearray(16)
    raw = torque + 1024
    data[5] = (raw & 0x7F) << 1
    data[6] = ((raw >> 7) & 0xF) | 0x10
    return self.safety.safety_tx_hook(self.packet(self.steer_addr, 0, data))

  def tx_live(self, torque):
    self.safety.set_controls_allowed(True)
    self.safety.set_torque_driver(0, 0)
    data = bytearray(16)
    raw = torque + 1024
    data[5] = (raw & 0x7F) << 1
    data[6] = ((raw >> 7) & 0xF) | 0x10
    return self.safety.safety_tx_hook(self.packet(self.steer_addr, 0, data))

  def test_boundaries_and_controller_agreement(self):
    flags = Flags.CANFD_DYNAMIC_TORQUE
    for lka in (False, True):
      for kph in (0., 20., 39.6, 43.2, 46.8, 50.4, 54., 57.6, 61.2, 72., 108.):
        with self.subTest(lka=lka, kph=kph):
          self.configure(lka=lka)
          speed = float(np.float32(self.speed(kph)))
          maximum = self.steer_max(self.params, flags, speed)
          for sign in (-1, 1):
            self.assertTrue(self.tx(sign * maximum), (speed, maximum))
            self.assertFalse(self.tx(sign * (maximum + 1)), (speed, maximum))

  def test_rate_boundaries_and_controller_agreement(self):
    flags = Flags.CANFD_DYNAMIC_TORQUE
    for kph in (0., 39.6, 41.4, 43.2, 45., 46.8, 61.2, 108.):
      with self.subTest(kph=kph):
        self.configure()
        speed = float(np.float32(self.speed(kph)))
        rate_up, _ = self.steer_rates(self.params, flags, speed)
        self.assertTrue(self.tx(100 + rate_up, previous=100), (speed, rate_up))
        self.assertFalse(self.tx(100 + rate_up + 1, previous=100), (speed, rate_up))

  def test_low_speed_rate_reaches_2022_max_with_rt_limit(self):
    self.configure()
    self.speed(20.)
    for frame, torque in enumerate(range(10, 385, 10)):
      torque = min(torque, 384)
      self.safety.set_timer(100 + frame * 10_000)
      self.assertTrue(self.tx_live(torque), (frame, torque))
    self.assertTrue(self.tx_live(384))
    self.assertFalse(self.tx(385))

  def test_rt_window_survives_rate_transition(self):
    self.configure()
    self.speed(20.)
    for frame, torque in enumerate(range(10, 201, 10)):
      self.safety.set_timer(100 + frame * 10_000)
      self.assertTrue(self.tx_live(torque), (frame, torque))

    self.speed(46.8)
    for frame, torque in enumerate(range(202, 215, 2), start=20):
      self.safety.set_timer(100 + frame * 10_000)
      self.assertTrue(self.tx_live(torque), (frame, torque))

  def test_speed_transition_requires_monotonic_wind_down(self):
    self.configure()
    self.speed(39.)
    self.assertTrue(self.tx(384))

    speed = float(np.float32(self.speed(43.2)))
    maximum = self.steer_max(self.params, Flags.CANFD_DYNAMIC_TORQUE, speed)
    _, rate_down = self.steer_rates(self.params, Flags.CANFD_DYNAMIC_TORQUE, speed)
    self.assertLess(maximum, 384)
    self.assertTrue(self.tx(384 - rate_down, previous=384))
    self.assertFalse(self.tx(384, previous=384))
    self.assertFalse(self.tx(385, previous=384))

  def test_all_quantized_wheel_speeds_in_both_tapers(self):
    self.configure()
    flags = Flags.CANFD_DYNAMIC_TORQUE
    for raw_sum in range(int(10 * 3.6 * 32 * 4), int(18 * 3.6 * 32 * 4) + 1):
      counts = [raw_sum // 4 + (i < raw_sum % 4) for i in range(4)]
      speed = float(np.float32(self.speed_counts(counts)))
      maximum = self.steer_max(self.params, flags, speed)
      self.assertTrue(self.tx(maximum), (speed, maximum))

  def test_disabled_and_legacy_creep_bit_stay_at_stock_limits(self):
    stock = SimpleNamespace(STEER_MAX=270, STEER_DELTA_UP=2, STEER_DELTA_DOWN=3)
    for legacy_bit in (False, True):
      for kph in (0., 20., 39.6, 46.8, 61.2, 108.):
        with self.subTest(legacy_bit=legacy_bit, kph=kph):
          self.configure(dynamic=False, legacy_creep_bit=legacy_bit)
          speed = float(np.float32(self.speed(kph)))
          self.assertEqual(self.steer_max(stock, Flags(0), speed), 270)
          self.assertEqual(self.steer_rates(stock, Flags(0), speed), (2, 3))
          self.assertTrue(self.tx(270))
          self.assertFalse(self.tx(271))

  def test_high_speed_remains_270(self):
    for kph in (61.2, 80., 100., 130., 180.):
      self.configure()
      self.speed(kph)
      for sign in (-1, 1):
        self.assertTrue(self.tx(sign * 270))
        self.assertFalse(self.tx(sign * 271))


if __name__ == "__main__":
  unittest.main()
