"""Run the real C safety hooks against encoded CAN inputs, without a vehicle."""
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
  CANFD_CREEP_LANE_CHANGE = 1 << 28


def controller_function():
  source = Path(__file__).resolve().parents[2] / 'car/hyundai/carcontroller.py'
  tree = ast.parse(source.read_text())
  nodes = [n for n in tree.body if (isinstance(n, ast.FunctionDef) and n.name == 'get_steer_max') or
           (isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id.startswith('CREEP_LANE_CHANGE_') for t in n.targets))]
  scope = {'np': np, 'HyundaiFlags': Flags, 'CV': SimpleNamespace(KPH_TO_MS=1 / 3.6)}
  exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), 'exec'), scope)
  return scope['get_steer_max']


class TestHkgCreepCurveNative(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.safety = libsafety_py.libsafety
    cls.steer_max = staticmethod(controller_function())

  def configure(self, dynamic=False, enabled=True, lka=False):
    self.dynamic = dynamic
    self.pt_bus = 1 if lka else 0
    self.steer_addr = 0x50 if lka else 0x12A
    self.counter = 0
    param = 1 | (16 if lka else 8) | (1024 if dynamic else 0) | (2048 if enabled else 0)
    self.assertEqual(self.safety.set_safety_hooks(28, param), 0)
    self.safety.init_tests()
    self.safety.set_timer(100)

  def packet(self, address, bus, data):
    if address == 0x413:  # BLINKERS has no checksum/counter fields in its DBC.
      return libsafety_py.make_CANPacket(address, bus, data)
    if len(data) == 8:
      data[1] = (self.counter % 16) << 4
    else:
      data[2] = self.counter % 256
    self.counter += 1
    crc = binascii.crc_hqx(bytes(data[2:]) + address.to_bytes(2, 'little'), 0)
    crc ^= {8: 0x5F29, 16: 0x041D, 24: 0x819D, 32: 0x9F5B}[len(data)]
    data[:2] = crc.to_bytes(2, 'little')
    return libsafety_py.make_CANPacket(address, bus, data)

  def speed(self, kph):
    count = round(kph / 0.03125)
    return self.speed_counts([count] * 4)

  def speed_counts(self, counts):
    for _ in range(8):
      data = bytearray(24)
      for offset, count in zip((8, 10, 12, 14), counts, strict=True):
        data[offset:offset + 2] = count.to_bytes(2, 'little')
      self.safety.safety_rx_hook(self.packet(0xA0, self.pt_bus, data))
    return sum(counts) / 4 * 0.03125 / 3.6

  def blink(self, left=True, right=False):
    data = bytearray(8)
    data[2] = (int(left) << 4) | (int(right) << 6)
    self.safety.safety_rx_hook(self.packet(0x413, self.pt_bus, data))

  def brake(self, pressed):
    data = bytearray(24)
    data[10] = int(pressed) << 1
    self.safety.safety_rx_hook(self.packet(0x175, self.pt_bus, data))

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

  def test_boundaries_and_controller_agreement(self):
    for dynamic in (False, True):
      for lka in (False, True):
        for kph, expected in ((0, 400), (20, 400), (20.5, 400), (21, 400),
                              (25.5, 375 if dynamic else 335), (30, 350 if dynamic else 270),
                              (31, 350 if dynamic else 270)):
          with self.subTest(dynamic=dynamic, lka=lka, kph=kph):
            self.configure(dynamic, lka=lka)
            speed = self.speed(kph)
            self.blink()
            flags = Flags.CANFD_CREEP_LANE_CHANGE | (Flags.CANFD_DYNAMIC_TORQUE if dynamic else 0)
            params = SimpleNamespace(STEER_MAX=350 if dynamic else 270, STEER_MAX_LOOKUP=([9., 13., 17.], [350, 350, 270]))
            self.assertEqual(self.steer_max(params, flags, float(np.float32(speed)), True), expected)
            for sign in (-1, 1):
              self.assertTrue(self.tx(sign * expected))
              self.assertFalse(self.tx(sign * (expected + 1)))

  def test_all_quantized_wheel_speeds_in_taper(self):
    for dynamic in (False, True):
      self.configure(dynamic)
      params = SimpleNamespace(STEER_MAX=350 if dynamic else 270, STEER_MAX_LOOKUP=([9., 13., 17.], [350, 350, 270]))
      flags = Flags.CANFD_CREEP_LANE_CHANGE | (Flags.CANFD_DYNAMIC_TORQUE if dynamic else 0)
      for raw_sum in range(21 * 32 * 4, 30 * 32 * 4 + 1):
        counts = [raw_sum // 4 + (i < raw_sum % 4) for i in range(4)]
        speed = self.speed_counts(counts)
        self.blink()
        maximum = self.steer_max(params, flags, float(np.float32(speed)), True)
        self.assertTrue(self.tx(maximum), (dynamic, speed, maximum))

  def test_existing_prerequisites_still_required_at_twenty(self):
    for reason in ('disabled', 'no_signal', 'hazards', 'brake', 'stale'):
      with self.subTest(reason=reason):
        self.configure(enabled=reason != 'disabled')
        self.speed(20)
        if reason != 'no_signal':
          self.blink(right=reason == 'hazards')
        if reason == 'brake':
          self.brake(True)
        if reason == 'stale':
          self.safety.set_timer(1_200_101)
        self.assertFalse(self.tx(400))

  def test_wind_down_above_thirty_cannot_hold_or_increase(self):
    self.configure()
    self.speed(20)
    self.blink()
    self.assertTrue(self.tx(400))
    self.speed(31)
    self.assertTrue(self.tx(397, previous=400))
    self.assertFalse(self.tx(397, previous=397))
    self.assertFalse(self.tx(399, previous=397))

  def test_wrong_bus_or_length_cannot_authorize_creep(self):
    for bus, length in ((2, 8), (0, 16)):
      self.configure()
      self.speed(20)
      data = bytearray(length)
      data[2] = 1 << 4
      self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x413, bus, data))
      self.assertFalse(self.tx(400))

  def test_normal_high_speed_curve_unchanged(self):
    for dynamic in (False, True):
      for kph, maximum in ((46.8, 350 if dynamic else 270), (54, 310 if dynamic else 270), (61.2, 270)):
        self.configure(dynamic)
        self.speed(kph)
        self.blink()
        for sign in (-1, 1):
          self.assertTrue(self.tx(sign * maximum))
          self.assertFalse(self.tx(sign * (maximum + 1)))


if __name__ == '__main__':
  unittest.main()
