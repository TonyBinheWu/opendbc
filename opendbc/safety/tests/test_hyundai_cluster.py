"""Exercise production C hooks, including packet consumption and forwarding."""

import itertools
import unittest

from opendbc.can import CANPacker
from opendbc.car.hyundai.formosa_cluster import MESSAGES, overlay
from opendbc.safety.tests.libsafety import libsafety_py


class TestHyundaiClusterSync(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.packer = CANPacker("hyundai_canfd_generated")
    self.safety.set_safety_hooks(28, 8 | 16384)
    self.safety.init_tests()
    self.safety.set_timer(1000)

  def packet(self, addr, bus, raw=None, changes=None):
    msg = self.packer.dbc.addr_to_msg[addr]
    data = raw if raw is not None else bytes((i * 17 + 13) % 256 for i in range(msg.size))
    data = overlay(self.packer.dbc, addr, data, changes or {})
    return libsafety_py.make_CANPacket(addr, bus, data)

  def data(self, packet):
    return bytes(libsafety_py.ffi.buffer(packet[0].data, self.packer.dbc.addr_to_msg[packet[0].addr].size))

  def consume(self, packet):
    accepted = self.safety.safety_tx_hook(packet)
    return bool(accepted and self.safety.safety_tx_consumed_hook(packet))

  def test_every_cluster_uses_original_counter_crc_and_protected_bits(self):
    for addr in MESSAGES:
      self.safety.set_safety_hooks(28, 8 | 16384)
      self.safety.init_tests()
      self.safety.set_timer(1000)
      host = self.packet(addr, 0, changes={"COUNTER": 10})
      self.assertTrue(self.consume(host))
      original = self.packet(addr, 2, raw=bytes([0xA5] * self.packer.dbc.addr_to_msg[addr].size), changes={"COUNTER": 249})
      before = self.data(original)
      self.assertEqual(self.safety.safety_fwd_packet_hook(2, original), 0)
      after = self.data(original)
      self.assertEqual(after[2], 249)
      # Use independently computed DBC signal masks, not the C byte masks.
      fields = {
        0x161: (
          "LCA_LEFT_ARROW",
          "LCA_RIGHT_ARROW",
          "CENTERLINE",
          "LANELINE_LEFT",
          "LANELINE_RIGHT",
          "SETSPEED_HUD",
          "DISTANCE",
          "SETSPEED_SPEED",
          "SETSPEED",
          "LFA_ICON",
        ),
        0x162: ("LEAD", "LEAD_ALT", "LEAD_LEFT", "LEAD_RIGHT", "LEAD_LEFT_REAR_STATUS", "LEAD_RIGHT_REAR_STATUS"),
        0x1E0: ("HDA_ICON", "LFA_ICON"),
        0x1EA: (),
        0x200: (),
      }[addr]
      expected = bytearray(before)
      from opendbc.can.packer import set_value
      from opendbc.can.parser import get_raw_value

      for key in fields:
        sig = self.packer.dbc.addr_to_msg[addr].sigs[key]
        set_value(expected, sig, get_raw_value(self.data(host), sig))
      expected = overlay(self.packer.dbc, addr, expected, {})
      self.assertEqual(after, expected)
      if addr in (0x1EA, 0x200):
        self.assertEqual(after, before)

  def test_timeout_boundary_wrap_and_missing_host_preserve_original(self):
    for elapsed in (0, 149999, 150000, 150001, 1_000_000):
      for start in (1000, 0xFFFFFF00):
        self.safety.set_safety_hooks(28, 8 | 16384)
        self.safety.init_tests()
        self.safety.set_timer(start)
        self.assertTrue(self.consume(self.packet(0x161, 0)))
        self.safety.set_timer((start + elapsed) & 0xFFFFFFFF)
        source = self.packet(0x161, 2, changes={"COUNTER": 70, "LFA_ICON": 0})
        before = self.data(source)
        self.assertEqual(self.safety.safety_fwd_packet_hook(2, source), 0)
        if elapsed >= 150000:
          self.assertEqual(self.data(source), before)
    self.safety.set_safety_hooks(28, 8 | 16384)
    source = self.packet(0x161, 2)
    before = self.data(source)
    self.assertEqual(self.safety.safety_fwd_packet_hook(2, source), 0)
    self.assertEqual(self.data(source), before)

  def test_bad_host_crc_wrong_bus_length_and_control_are_not_consumed(self):
    host = self.packet(0x161, 0)
    host[0].data[4] ^= 1
    self.assertFalse(self.consume(host))
    self.assertFalse(self.safety.safety_tx_consumed_hook(host))
    for addr in MESSAGES:
      for bus in (1, 2):
        self.assertFalse(self.consume(self.packet(addr, bus)))
      short = libsafety_py.make_CANPacket(addr, 0, b"\0" * 4)
      self.assertFalse(self.consume(short))
    control = self.packer.make_can_msg("LFA", 0, {"StrTqReqVal": 0, "ActToiSta": 0})
    control = libsafety_py.make_CANPacket(control[0], control[2], control[1])
    self.assertFalse(self.safety.safety_tx_consumed_hook(control))

  def test_bad_original_not_repaired_and_invalidates_cache(self):
    self.assertTrue(self.consume(self.packet(0x161, 0)))
    original = self.packet(0x161, 2)
    original[0].data[5] ^= 1
    before = self.data(original)
    self.assertEqual(self.safety.safety_fwd_packet_hook(2, original), 0)
    self.assertEqual(self.data(original), before)
    original = self.packet(0x161, 2, changes={"COUNTER": 17})
    before = self.data(original)
    self.safety.safety_fwd_packet_hook(2, original)
    self.assertEqual(self.data(original), before)

  def test_relay_fault_and_reset_clear_cache(self):
    self.assertTrue(self.consume(self.packet(0x161, 0)))
    self.safety.set_relay_malfunction(True)
    self.assertFalse(self.consume(self.packet(0x161, 0)))
    self.assertEqual(self.safety.safety_fwd_packet_hook(2, self.packet(0x161, 2)), -1)
    self.safety.set_safety_hooks(28, 8 | 16384)
    source = self.packet(0x161, 2)
    before = self.data(source)
    self.safety.safety_fwd_packet_hook(2, source)
    self.assertEqual(self.data(source), before)

  def test_mode_does_not_activate_for_radar_or_lka_harness(self):
    for camera, lka, enabled in itertools.product((False, True), repeat=3):
      param = (8 if camera else 0) | (16 if lka else 0) | (16384 if enabled else 0)
      self.safety.set_safety_hooks(28, param)
      self.safety.init_tests()
      self.assertEqual(self.consume(self.packet(0x161, 0)), camera and not lka and enabled)

  def test_slow_host_and_fast_oem_do_not_accumulate_history(self):
    self.assertTrue(self.consume(self.packet(0x161, 0, changes={"LFA_ICON": 1})))
    for index in range(60):
      self.safety.set_timer(1000 + index * 1000)
      if index == 30:
        self.assertTrue(self.consume(self.packet(0x161, 0, changes={"LFA_ICON": 2})))
      packet = self.packet(0x161, 2, changes={"COUNTER": index})
      self.assertEqual(self.safety.safety_fwd_packet_hook(2, packet), 0)
      self.assertEqual(self.data(packet)[28] & 15, 1 if index < 30 else 2)
      self.assertEqual(self.data(packet)[2], index)

  def test_packet_forwarding_api_preserves_legacy_modes(self):
    for param in (0, 8, 4, 12, 16, 20, 16 | 128, 8 | 2048):
      self.safety.set_safety_hooks(28, param)
      self.safety.init_tests()
      for addr in (0x50, 0x110, 0x12A, 0x1A0, 0xEA, 0x161, 0x162, 0x1E0, 0x1EA, 0x200):
        for bus in (0, 1, 2):
          packet = libsafety_py.make_CANPacket(addr, bus, bytes(range(32)))
          before = bytes(libsafety_py.ffi.buffer(packet[0].data, 32))
          expected = self.safety.safety_fwd_hook(bus, addr)
          self.assertEqual(self.safety.safety_fwd_packet_hook(bus, packet), expected)
          self.assertEqual(bytes(libsafety_py.ffi.buffer(packet[0].data, 32)), before)

  def test_active_display_mode_preserves_control_tx_decisions(self):
    for param in (8, 12):
      for torque in range(-400, 401, 40):
        values = {"StrTqReqVal": torque, "ActToiSta": 1}
        addr, data, bus = self.packer.make_can_msg("LFA", 0, values)
        results = []
        for mode in (param, param | 16384):
          self.safety.set_safety_hooks(28, mode)
          self.safety.init_tests()
          self.safety.set_controls_allowed(True)
          packet = libsafety_py.make_CANPacket(addr, bus, data)
          results.append(self.safety.safety_tx_hook(packet))
          self.assertFalse(self.safety.safety_tx_consumed_hook(packet))
        self.assertEqual(*results)
