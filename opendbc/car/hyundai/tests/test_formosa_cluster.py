import itertools
import time
import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker, CANParser
from opendbc.car import structs, get_safety_config
from opendbc.car.hyundai.formosa_cluster import ClusterDisplay, DISPLAY_FLAG, SAFETY_FLAG, MESSAGES, configure, overlay
from opendbc.car.hyundai.values import HyundaiFlags, HyundaiSafetyFlags


class TestFormosaCluster(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker("hyundai_canfd_generated")
    self.parser = CANParser("hyundai_canfd_generated", [], 2)
    self.parser.track_raw_messages(MESSAGES.values())
    self.cp = structs.CarParams(
      brand="hyundai",
      flags=int(HyundaiFlags.CANFD | HyundaiFlags.CANFD_CAMERA_SCC | HyundaiFlags.FORMOSA_CLUSTER_AVAILABLE),
      safetyConfigs=[get_safety_config(structs.CarParams.SafetyModel.hyundaiCanfd, int(HyundaiSafetyFlags.CAMERA_SCC))],
    )
    configure(self.cp, True)
    hud = SimpleNamespace(leftLaneVisible=True, rightLaneVisible=True, leftLaneDepart=False, rightLaneDepart=False, leadDistanceBars=3)
    self.cc = SimpleNamespace(latActive=True, longActive=False, leftBlinker=True, rightBlinker=False, hudControl=hud)
    self.cs = SimpleNamespace(is_metric=True, out=SimpleNamespace(leftBlindspot=False, rightBlindspot=False, vCruiseCluster=80))
    self.now = time.monotonic_ns()

  def receive(self, name, values=None, bus=2, at=None):
    packet = self.packer.make_can_msg(name, bus, values or {})
    self.parser.update([(self.now if at is None else at, [packet])])
    return packet

  def test_explicit_mode_support_and_no_flag_collisions(self):
    for flag in HyundaiFlags:
      if flag.name != "FORMOSA_CLUSTER":
        self.assertFalse(flag.value & DISPLAY_FLAG)
    for flag in HyundaiSafetyFlags:
      if flag.name != "FORMOSA_CLUSTER":
        self.assertFalse(flag.value & SAFETY_FLAG)
    for canfd, camera, lka, enabled in itertools.product((False, True), repeat=4):
      self.cp.flags = int(
        (HyundaiFlags.CANFD if canfd else 0)
        | (HyundaiFlags.CANFD_CAMERA_SCC if camera else 0)
        | (HyundaiFlags.CANFD_LKA_STEER_MSG if lka else 0)
        | HyundaiFlags.FORMOSA_CLUSTER_AVAILABLE
      )
      configure(self.cp, enabled)
      expected = canfd and camera and not lka and enabled
      self.assertEqual(bool(self.cp.flags & DISPLAY_FLAG), expected)
      self.assertEqual(bool(self.cp.safetyConfigs[-1].safetyParam & SAFETY_FLAG), expected)
    self.cp.dashcamOnly = True
    configure(self.cp, True)
    self.assertFalse(self.cp.flags & DISPLAY_FLAG)

  def test_raw_observer_is_optional_bus_scoped_and_does_not_gate_driving(self):
    self.receive("CCNC_0x161", bus=0)
    self.assertFalse(self.parser.raw_messages)
    self.receive("CCNC_0x161")
    self.assertIn(0x161, self.parser.raw_messages)
    self.assertEqual(self.parser.addresses, set())
    self.parser.update([(self.now, [(0x161, b"bad", 2)])])
    self.assertNotIn(0x161, self.parser.raw_messages)

  def test_faults_sounds_unknown_bits_geometry_and_counter_preserved(self):
    for name in MESSAGES.values():
      msg = self.packer.dbc.name_to_msg[name]
      # Include unmodelled bits: rebuilding a whole message from decoded fields would lose these.
      raw = bytes((index * 37 + 19) % 256 for index in range(msg.size))
      raw = overlay(self.packer.dbc, msg.address, raw, {"COUNTER": 255})
      self.parser.update([(self.now, [(msg.address, raw, 2)])])
    packets = ClusterDisplay().messages(self.parser, self.cp, self.cc, self.cs, True, self.now)
    self.assertEqual(len(packets), 5)
    decoded = CANParser("hyundai_canfd_generated", [(name, 0) for name in MESSAGES.values()], 0)
    for address, data, bus in packets:
      self.assertEqual(bus, 0)
      self.assertEqual(data[2], 255)
      original = self.parser.raw_messages[address][1]
      decoded.update([(self.now, [(address, data, bus)])])
      sigs = decoded.dbc.addr_to_msg[address].sigs
      protected = [
        key
        for key in sigs
        if key.startswith(("FAULT_", "ALERTS_", "SOUNDS_", "ZEROS_")) or key in ("LANELINE_CURVATURE", "LANELINE_LEFT_POSITION", "LANELINE_RIGHT_POSITION")
      ]
      original_parser = CANParser("hyundai_canfd_generated", [(MESSAGES[address], 0)], 2)
      original_parser.update([(self.now, [(address, original, 2)])])
      for key in protected:
        self.assertEqual(decoded.vl[address][key], original_parser.vl[address][key])
      if address in (0x1EA, 0x200):
        self.assertEqual(data, original)
    self.assertEqual(decoded.vl[0x161]["LFA_ICON"], 2)
    self.assertEqual(decoded.vl[0x161]["LCA_LEFT_ARROW"], 2)

  def test_stale_bad_checksum_wrong_size_and_duplicate_snapshot_not_published(self):
    address, data, bus = self.receive("CCNC_0x161")
    display = ClusterDisplay()
    self.assertEqual(len(display.messages(self.parser, self.cp, self.cc, self.cs, True, self.now)), 1)
    self.assertFalse(display.messages(self.parser, self.cp, self.cc, self.cs, True, self.now))
    self.assertFalse(ClusterDisplay().messages(self.parser, self.cp, self.cc, self.cs, True, self.now + 150_000_000))
    self.parser.raw_messages[address] = self.now, b"bad"
    self.assertFalse(ClusterDisplay().messages(self.parser, self.cp, self.cc, self.cs, True, self.now))
    bad = bytearray(data)
    bad[-1] ^= 1
    self.parser.raw_messages[address] = self.now, bytes(bad)
    self.assertFalse(ClusterDisplay().messages(self.parser, self.cp, self.cc, self.cs, True, self.now))

  def test_box_conversion_does_not_invent_objects_or_modify_distance(self):
    self.receive("CCNC_0x162", {"LEAD": 1, "LEAD_DISTANCE": 17.3, "LEAD_LEFT": 0, "LEAD_LEFT_DISTANCE": 7, "LEAD_RIGHT": 6, "LEAD_RIGHT_DISTANCE": 6})
    packets = ClusterDisplay().messages(self.parser, self.cp, self.cc, self.cs, True, self.now)
    decoded = CANParser("hyundai_canfd_generated", [("CCNC_0x162", 0)], 0)
    decoded.update([(self.now, packets)])
    self.assertEqual(decoded.vl[0x162]["LEAD"], 3)
    self.assertAlmostEqual(decoded.vl[0x162]["LEAD_DISTANCE"], 17.3)
    self.assertEqual(decoded.vl[0x162]["LEAD_LEFT"], 0)
    self.assertEqual(decoded.vl[0x162]["LEAD_RIGHT"], 6)

  def test_off_and_unsupported_modes_emit_nothing(self):
    self.receive("CCNC_0x161")
    configure(self.cp, False)
    self.assertFalse(ClusterDisplay().messages(self.parser, self.cp, self.cc, self.cs, True, self.now))

  def test_camera_source_capability_requires_both_observed_messages(self):
    from opendbc.car import gen_empty_fingerprint
    from opendbc.car.hyundai.interface import CarInterface
    from opendbc.car.hyundai.values import CAR

    for first, second in itertools.product((False, True), repeat=2):
      fingerprint = gen_empty_fingerprint()
      if first:
        fingerprint[2][0x161] = 32
      if second:
        fingerprint[2][0x162] = 32
      cp = CarInterface.get_params(CAR.HYUNDAI_IONIQ_5, fingerprint, [], False, False, False)
      configure(cp, True)
      self.assertEqual(bool(cp.flags & DISPLAY_FLAG), first and second)

  def test_real_controller_retains_steering_and_adds_only_display_packets(self):
    from opendbc.car.hyundai.carcontroller import CarController
    from opendbc.car.hyundai.hyundaicanfd import CanBus

    self.receive("CCNC_0x161")
    controller = CarController.__new__(CarController)
    controller.CP = self.cp
    controller.CAN = CanBus(self.cp)
    controller.packer = self.packer
    controller.formosa_cluster = ClusterDisplay()
    controller.frame = 0
    controller.lkas_icon = 1
    controller.lfa_icon = 1
    controller.last_button_frame = 0
    controller.cancel_counter = 0
    self.cs.formosa_parser = self.parser
    self.cc.enabled = True
    self.cc.cruiseControl = SimpleNamespace(cancel=False, resume=False)
    actual = controller.create_canfd_msgs(True, 0, 80, 0, False, self.cc.hudControl, self.cs, self.cc)
    self.assertEqual([msg[0] for msg in actual], [0x12A, 0x161])
    steering = actual[0]
    configure(self.cp, False)
    legacy = controller.create_canfd_msgs(True, 0, 80, 0, False, self.cc.hudControl, self.cs, self.cc)
    self.assertEqual([msg[0] for msg in legacy], [0x12A, 0x1E0])
    # Counter differs between calls; every steering-command data bit remains identical.
    self.assertEqual(steering[1][3:], legacy[0][1][3:])

  def test_display_uses_selected_panda_bus_offset(self):
    from opendbc.car import get_safety_config, structs
    self.receive("CCNC_0x161")
    self.cp.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.noOutput), self.cp.safetyConfigs[0]]
    messages = ClusterDisplay().messages(self.parser, self.cp, self.cc, self.cs, True, self.now)
    self.assertEqual(messages[0][2], 4)
