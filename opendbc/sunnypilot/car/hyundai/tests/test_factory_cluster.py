import unittest
from types import SimpleNamespace

from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.hyundaicanfd import hkg_can_fd_checksum
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.car.structs import CarParamsSP, FactoryClusterTarget
from opendbc.sunnypilot.car.hyundai.factory_cluster import (
  ADRV_1EA_CANDIDATE_LAYOUT,
  COUNTER,
  FactoryClusterDisplayManager,
  FactoryClusterStatus,
  FirmwareEvidence,
  SlotSourceRule,
  TargetSlot,
  VerifiedFactoryClusterProfile,
  VerifiedMessageProfile,
  configuration_status,
  enable_for_verified_profile,
  patch_hkg_frame,
  valid_hkg_frame,
)
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


def make_base(counter: int = 9) -> bytes:
  data = bytearray((index * 37 + 11) & 0xFF for index in range(32))
  data[2] = counter
  data[:2] = hkg_can_fd_checksum(0x1EA, None, data).to_bytes(2, "little")
  return bytes(data)


def make_cp():
  firmware = SimpleNamespace(ecu="combinationMeter", address=0x7C6, subAddress=0, bus=1,
                             fwVersion=b"TEST-EV6-CLU")
  return SimpleNamespace(
    brand="hyundai",
    carFingerprint=CAR.KIA_EV6,
    flags=HyundaiFlags.CANFD | HyundaiFlags.CANFD_LKA_STEER_MSG,
    carFw=[firmware],
  )


def make_profile() -> VerifiedFactoryClusterProfile:
  fields = ADRV_1EA_CANDIDATE_LAYOUT.fields
  message = VerifiedMessageProfile(
    layout=ADRV_1EA_CANDIDATE_LAYOUT,
    source_bus=1,
    transmit_bus=1,
    frequency_hz=20.0,
    vehicle_detect_values={slot: 3 for slot in fields},
    neutral_values={slot: (0, 0.0, 0.0) for slot in fields},
    source_rules=(
      SlotSourceRule(TargetSlot.LEFT_FRONT, 0.0, 100.0, -5.0, -1.0, lateral_scale=-1.0),
      SlotSourceRule(TargetSlot.RIGHT_FRONT, 0.0, 100.0, 1.0, 5.0),
    ),
    allow_cached_template=True,
    non_target_fields_static_verified=True,
    radar_source_verified=True,
  )
  return VerifiedFactoryClusterProfile(
    profile_id="synthetic-test-only",
    car_fingerprint=CAR.KIA_EV6,
    required_flags=int(HyundaiFlags.CANFD | HyundaiFlags.CANFD_LKA_STEER_MSG),
    required_firmware=(FirmwareEvidence("combinationMeter", 0x7C6, b"TEST-EV6-CLU", 1),),
    message=message,
    evidence=("synthetic unit test",),
    production_authorized=True,
  )


class TestFactoryClusterProfileGate(unittest.TestCase):
  def test_repository_has_no_implicitly_verified_ev6(self):
    self.assertEqual(configuration_status(make_cp(), True), FactoryClusterStatus.UNAVAILABLE_UNVERIFIED_PROFILE)

  def test_user_toggle_cannot_bypass_empty_verified_registry(self):
    cp_sp = CarParamsSP()
    status = enable_for_verified_profile(make_cp(), cp_sp, True)
    self.assertEqual(status, FactoryClusterStatus.UNAVAILABLE_UNVERIFIED_PROFILE)
    self.assertFalse(cp_sp.flags & HyundaiFlagsSP.FACTORY_CLUSTER_SIDE_DISPLAY)

  def test_exact_test_profile_requires_firmware(self):
    cp = make_cp()
    profile = make_profile()
    self.assertTrue(profile.matches(cp))
    cp.carFw[0].fwVersion = b"DIFFERENT"
    self.assertFalse(profile.matches(cp))

  def test_fuzzy_fingerprint_is_never_a_production_match(self):
    cp = make_cp()
    cp.fuzzyFingerprint = True
    self.assertFalse(make_profile().matches(cp))

  def test_cached_template_requires_static_non_target_proof(self):
    profile = make_profile()
    invalid_message = VerifiedMessageProfile(
      **{**profile.message.__dict__, "non_target_fields_static_verified": False}
    )
    with self.assertRaisesRegex(ValueError, "cached templates"):
      invalid_message.validate()

  def test_live_sender_rejects_a_rate_the_controller_cannot_schedule(self):
    profile = make_profile()
    invalid_message = VerifiedMessageProfile(
      **{**profile.message.__dict__, "frequency_hz": 10.0}
    )
    with self.assertRaisesRegex(ValueError, "20 Hz"):
      invalid_message.validate()


class TestFactoryClusterRawPatching(unittest.TestCase):
  def test_patch_preserves_every_unmodified_bit(self):
    base = make_base()
    layout = ADRV_1EA_CANDIDATE_LAYOUT.fields[TargetSlot.LEFT_FRONT]
    values = {layout.detect: 3, layout.distance: 24.5, layout.lateral: 3.2}
    patched = patch_hkg_frame(0x1EA, base, values, 10)
    self.assertTrue(valid_hkg_frame(0x1EA, patched, 32))
    self.assertEqual(COUNTER.decode(patched), 10)
    self.assertEqual(layout.detect.decode(patched), 3)
    self.assertAlmostEqual(layout.distance.decode(patched), 24.5)
    self.assertAlmostEqual(layout.lateral.decode(patched), 3.2)

    expected = bytearray(base)
    for signal, value in values.items():
      signal.encode(expected, value)
    COUNTER.encode(expected, 10)
    expected[:2] = patched[:2]
    self.assertEqual(patched, bytes(expected))

  def test_bad_template_is_rejected(self):
    with self.assertRaisesRegex(ValueError, "invalid HKG checksum"):
      patch_hkg_frame(0x1EA, bytes(32), {}, 1)


class TestAdrvReplacementTransport(unittest.TestCase):
  class Packer:
    @staticmethod
    def make_can_msg(name, bus, values):
      return name, values, bus

  CAN = SimpleNamespace(ACAN=0, ECAN=1)

  @staticmethod
  def _adrv_1ea(messages):
    return [message for message in messages if message[0] in (0x1EA, "ADRV_0x1ea")]

  def test_override_replaces_default_once_at_20_hz(self):
    base = make_base()
    messages = hyundaicanfd.create_adrv_messages(self.Packer(), self.CAN, 10, adrv_1ea_override=base)
    self.assertEqual(self._adrv_1ea(messages), [(0x1EA, base, 1)])
    self.assertEqual(self._adrv_1ea(hyundaicanfd.create_adrv_messages(
      self.Packer(), self.CAN, 11, adrv_1ea_override=base)), [])

  def test_suppression_never_leaves_a_second_sender(self):
    messages = hyundaicanfd.create_adrv_messages(self.Packer(), self.CAN, 10, suppress_default_adrv_1ea=True)
    self.assertEqual(self._adrv_1ea(messages), [])


class TestFactoryClusterDisplayManager(unittest.TestCase):
  def setUp(self):
    self.cp = make_cp()
    self.profile = make_profile()
    self.manager = FactoryClusterDisplayManager(self.cp, CarParamsSP(), profile=self.profile, requested=True)
    self.base_time = 1_000_000_000
    self.base = make_base()
    self.manager.observe_can([(self.base_time, [(0x1EA, self.base, 1)])])

  def _target(self, track_id: int, d_rel: float, y_rel: float, timestamp: int) -> FactoryClusterTarget:
    return FactoryClusterTarget(trackId=track_id, sourceMonoTime=timestamp, dRel=d_rel, yRel=y_rel,
                                vRel=-1.0, measured=True)

  def test_fresh_stock_sender_wins_without_duplicate_transmission(self):
    decision = self.manager.build_adrv_1ea([], self.base_time, True, False, False, self.base_time + 100_000_000)
    self.assertTrue(decision.suppress_default)
    self.assertIsNone(decision.data)
    self.assertEqual(self.manager.status, FactoryClusterStatus.ACTIVE_STOCK_PASSTHROUGH)

  def test_stale_stock_frame_can_supply_only_a_verified_template(self):
    now = self.base_time + 1_000_000_000
    targets = [self._target(2, 22.0, -3.1, now), self._target(1, 31.0, 3.4, now)]
    decision = self.manager.build_adrv_1ea(targets, now, True, False, False, now)
    self.assertIsNotNone(decision.data)
    self.assertEqual(self.manager.status, FactoryClusterStatus.ACTIVE_REAL_TARGETS)
    assert decision.data is not None
    left = ADRV_1EA_CANDIDATE_LAYOUT.fields[TargetSlot.LEFT_FRONT]
    right = ADRV_1EA_CANDIDATE_LAYOUT.fields[TargetSlot.RIGHT_FRONT]
    self.assertEqual(left.detect.decode(decision.data), 3)
    self.assertAlmostEqual(left.distance.decode(decision.data), 22.0)
    self.assertAlmostEqual(left.lateral.decode(decision.data), 3.1)
    self.assertEqual(right.detect.decode(decision.data), 3)
    self.assertAlmostEqual(right.distance.decode(decision.data), 31.0)
    self.assertAlmostEqual(right.lateral.decode(decision.data), 3.4)

  def test_stale_radar_clears_slots_and_never_turns_bsm_into_distance(self):
    now = self.base_time + 1_000_000_000
    decision = self.manager.build_adrv_1ea([], 0, False, True, True, now)
    assert decision.data is not None
    self.assertEqual(self.manager.status, FactoryClusterStatus.UNAVAILABLE_RADAR_DATA)
    for layout in ADRV_1EA_CANDIDATE_LAYOUT.fields.values():
      self.assertEqual(layout.detect.decode(decision.data), 0)
      self.assertEqual(layout.distance.decode(decision.data), 0)
      self.assertEqual(layout.lateral.decode(decision.data), 0)

  def test_unrepresentable_target_fails_closed_without_crashing(self):
    message = VerifiedMessageProfile(
      **{**self.profile.message.__dict__, "source_rules": (
        SlotSourceRule(TargetSlot.LEFT_FRONT, 0.0, 100.0, -5.0, -1.0, distance_scale=1000.0),
      )}
    )
    profile = VerifiedFactoryClusterProfile(**{**self.profile.__dict__, "message": message})
    manager = FactoryClusterDisplayManager(self.cp, CarParamsSP(), profile=profile, requested=True)
    manager.observe_can([(self.base_time, [(0x1EA, self.base, 1)])])
    now = self.base_time + 1_000_000_000
    decision = manager.build_adrv_1ea([self._target(1, 50.0, -3.0, now)], now, True, False, False, now)
    self.assertIsNone(decision.data)
    self.assertFalse(decision.suppress_default)
    self.assertEqual(manager.status, FactoryClusterStatus.UNAVAILABLE_TARGET_ENCODING)

  def test_new_original_sender_after_takeover_latches_conflict(self):
    now = self.base_time + 1_000_000_000
    first = self.manager.build_adrv_1ea([], now, True, False, False, now)
    self.assertIsNotNone(first.data)
    self.manager.observe_can([(now + 1, [(0x1EA, make_base(20), 1)])])
    blocked = self.manager.build_adrv_1ea([], now + 1, True, False, False, now + 1)
    self.assertTrue(blocked.suppress_default)
    self.assertEqual(self.manager.status, FactoryClusterStatus.BLOCKED_SENDER_CONFLICT)


if __name__ == "__main__":
  unittest.main()
