import unittest

from opendbc.car.hyundai.values import HyundaiSafetyFlags as Flags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.common import CANPackerSafety, make_msg
from opendbc.safety.tests.libsafety import libsafety_py


class TestHyundaiCanfdBlinkers(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.packer = CANPackerSafety('hyundai_canfd_generated')

  def init(self, flags=Flags.CANFD_LKA_STEER_MSG | Flags.CANFD_ENABLE_BLINKERS):
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, flags)
    self.safety.init_tests()

  def test_flag_model_and_bus_isolation(self):
    for flags in [0, Flags.CANFD_LKA_STEER_MSG, Flags.CANFD_ENABLE_BLINKERS,
                  Flags.CANFD_LKA_STEER_MSG | Flags.CANFD_ENABLE_BLINKERS,
                  Flags.CANFD_LKA_STEER_MSG | Flags.CANFD_LKA_STEER_MSG_ALT | Flags.CANFD_ENABLE_BLINKERS,
                  Flags.CANFD_LKA_STEER_MSG | Flags.LONG | Flags.CANFD_ENABLE_BLINKERS]:
      for bus in range(3):
        self.init(flags)
        self.safety.set_controls_allowed(True)
        allowed = bool(flags & Flags.CANFD_ENABLE_BLINKERS and flags & Flags.CANFD_LKA_STEER_MSG and bus == 1)
        for name in ('SPAS1', 'SPAS2'):
          self.assertEqual(bool(self.safety.safety_tx_hook(self.packer.make_can_msg_safety(name, bus, {}))), allowed)

  def test_only_left_right_cancel_and_only_engaged(self):
    for active in (False, True):
      for lateral in (False, True):
        for signal in range(8):
          self.init()
          self.safety.set_controls_allowed(active)
          self.safety.set_controls_allowed_lateral(lateral)
          msg = self.packer.make_can_msg_safety('SPAS2', 1, {'BLINKER_CONTROL': signal})
          self.assertEqual(bool(self.safety.safety_tx_hook(msg)), signal == 0 or ((active or lateral) and signal in (3, 4)))

  def test_all_other_payload_bits_are_blocked(self):
    self.init()
    self.safety.set_controls_allowed(True)
    for addr, size in [(0x165, 24), (0x16A, 32)]:
      for index in range(3, size):
        for bit in range(8):
          if addr == 0x16A and index == 16 and bit >= 5:
            continue
          data = bytearray(size)
          data[index] = 1 << bit
          self.assertFalse(self.safety.safety_tx_hook(make_msg(1, addr, dat=data)), (addr, index, bit))

  def test_uds_only_tester_present_and_wrong_length(self):
    self.init()
    for dat, allowed in [(b'\x02\x3e\x80\x00\x00\x00\x00\x00', True),
                         (b'\x02\x10\x03\x00\x00\x00\x00\x00', False),
                         (b'\x03\x28\x80\x01\x00\x00\x00\x00', False), (bytes(8), False)]:
      for bus in range(3):
        self.assertEqual(bool(self.safety.safety_tx_hook(make_msg(bus, 0x7B1, dat=dat))), allowed and bus == 1)
    for addr, length in [(0x165, 32), (0x16A, 24), (0x7B1, 16)]:
      self.assertFalse(self.safety.safety_tx_hook(make_msg(1, addr, length)))

  def test_flag_resets_on_next_drive(self):
    self.init()
    msg = self.packer.make_can_msg_safety('SPAS1', 1, {})
    self.assertTrue(self.safety.safety_tx_hook(msg))
    self.init(Flags.CANFD_LKA_STEER_MSG)
    self.assertFalse(self.safety.safety_tx_hook(msg))


if __name__ == '__main__':
  unittest.main()
