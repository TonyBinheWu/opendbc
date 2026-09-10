"""Evidence-gated support for Hyundai/Kia factory-cluster side targets.

This module contains no verified EV6 profile yet.  The candidate signal layouts
are useful for offline correlation only and cannot authorize transmission.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum
from typing import Any

from opendbc.can.dbc import Signal
from opendbc.can.packer import set_value
from opendbc.can.parser import get_raw_value
from opendbc.car.hyundai.hyundaicanfd import hkg_can_fd_checksum
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.car.structs import FactoryClusterTarget
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


FACTORY_SIDE_DISPLAY_PARAM = "HkgFactorySideVehicleDisplay"
FACTORY_SIDE_DISPLAY_STATUS_PARAM = "HkgFactorySideVehicleDisplayStatus"

BSM_ADDRESS = 0x1BA
CCNC_STATUS_ADDRESS = 0x161
CCNC_OBJECTS_ADDRESS = 0x162
LFAHDA_CLUSTER_ADDRESS = 0x1E0
ADRV_OBJECTS_ADDRESS = 0x1EA
HDA_INFO_ADDRESS = 0x4A3

EXPECTED_LENGTHS = {
  BSM_ADDRESS: 24,
  CCNC_STATUS_ADDRESS: 32,
  CCNC_OBJECTS_ADDRESS: 32,
  LFAHDA_CLUSTER_ADDRESS: 16,
  ADRV_OBJECTS_ADDRESS: 32,
  # 0x4A3 is deliberately omitted: no EV6 payload format is verified.
}


class FactoryClusterStatus(StrEnum):
  OFF = "off"
  UNAVAILABLE_VEHICLE = "unavailable_vehicle"
  UNAVAILABLE_UNVERIFIED_PROFILE = "unavailable_unverified_profile"
  UNAVAILABLE_NO_STOCK_TEMPLATE = "unavailable_no_stock_template"
  UNAVAILABLE_INVALID_STOCK_TEMPLATE = "unavailable_invalid_stock_template"
  UNAVAILABLE_STALE_STOCK_MESSAGE = "unavailable_stale_stock_message"
  UNAVAILABLE_RADAR_DATA = "unavailable_radar_data"
  UNAVAILABLE_TARGET_ENCODING = "unavailable_target_encoding"
  UNAVAILABLE_UNSUPPORTED_MESSAGE = "unavailable_unsupported_message"
  BLOCKED_SENDER_CONFLICT = "blocked_original_sender_conflict"
  ACTIVE_STOCK_PASSTHROUGH = "active_stock_passthrough"
  ACTIVE_REAL_TARGETS = "active_real_targets"
  ACTIVE_BSM_REGION = "active_bsm_region_warning"
  ACTIVE_NO_TARGETS = "active_no_targets"
  ACTIVE_AUTO_DETECTED = "active_auto_detected"


class TargetSlot(StrEnum):
  LEFT_FRONT = "left_front"
  RIGHT_FRONT = "right_front"
  LEFT_REAR = "left_rear"
  RIGHT_REAR = "right_rear"


class TargetSource(StrEnum):
  STOCK_MESSAGE = "stock_message"
  RADAR_TRACKS = "radar_tracks"
  BSM_REGION = "bsm_region"


@dataclass(frozen=True)
class SignalSpec:
  """A DBC-compatible signal description used to patch a raw payload in place."""

  start_bit: int
  size: int
  little_endian: bool
  signed: bool = False
  factor: float = 1.0
  offset: float = 0.0

  def occupied_bits(self) -> tuple[int, ...]:
    if self.size <= 0 or self.start_bit < 0:
      raise ValueError("invalid signal geometry")
    if self.little_endian:
      return tuple(range(self.start_bit, self.start_bit + self.size))
    be_bits = [bit + byte * 8 for byte in range(64) for bit in range(7, -1, -1)]
    try:
      index = be_bits.index(self.start_bit)
    except ValueError as exc:
      raise ValueError("invalid big-endian signal geometry") from exc
    bits = tuple(be_bits[index:index + self.size])
    if len(bits) != self.size:
      raise ValueError("invalid big-endian signal geometry")
    return bits

  def _signal(self) -> Signal:
    bits = self.occupied_bits()
    if self.little_endian:
      lsb = bits[0]
      msb = bits[-1]
    else:
      lsb = bits[-1]
      msb = self.start_bit
    return Signal("FACTORY_CLUSTER", self.start_bit, msb, lsb, self.size, self.signed,
                  self.factor, self.offset, self.little_endian)

  def decode(self, payload: bytes) -> float:
    raw = get_raw_value(payload, self._signal())
    if self.signed and raw & (1 << (self.size - 1)):
      raw -= 1 << self.size
    return raw * self.factor + self.offset

  def encode(self, payload: bytearray, value: float) -> None:
    if not math.isfinite(value):
      raise ValueError("signal value must be finite")
    raw = int(math.floor((value - self.offset) / self.factor + 0.5))
    minimum = -(1 << (self.size - 1)) if self.signed else 0
    maximum = (1 << (self.size - (1 if self.signed else 0))) - 1
    if not minimum <= raw <= maximum:
      raise ValueError(f"signal value {value} is outside [{minimum}, {maximum}] raw range")
    if raw < 0:
      raw += 1 << self.size
    set_value(payload, self._signal(), raw)


CHECKSUM = SignalSpec(0, 16, True)
COUNTER = SignalSpec(16, 8, True)


@dataclass(frozen=True)
class TargetFieldLayout:
  detect: SignalSpec
  distance: SignalSpec | None
  lateral: SignalSpec | None


@dataclass(frozen=True)
class CandidateMessageLayout:
  address: int
  length: int
  fields: dict[TargetSlot, TargetFieldLayout]
  research_source: str


# These are research hypotheses, not a support matrix.  They are intentionally
# separate from VERIFIED_PROFILES and are never consumed by the live sender.
ADRV_1EA_CANDIDATE_LAYOUT = CandidateMessageLayout(
  address=ADRV_OBJECTS_ADDRESS,
  length=32,
  fields={
    TargetSlot.LEFT_FRONT: TargetFieldLayout(
      SignalSpec(74, 3, False), SignalSpec(46, 11, True, factor=0.1), SignalSpec(70, 7, False, factor=0.1)),
    TargetSlot.RIGHT_FRONT: TargetFieldLayout(
      SignalSpec(98, 3, False), SignalSpec(75, 11, True, factor=0.1), SignalSpec(94, 7, False, factor=0.1)),
    # The carrot2-v9 0x1EA rear hypotheses do not include a lateral position,
    # so they are not represented as precise rear target slots here.
  },
  research_source="huheas/carrotpilot@carrot2-v9",
)

CCNC_162_CANDIDATE_LAYOUT = CandidateMessageLayout(
  address=CCNC_OBJECTS_ADDRESS,
  length=32,
  fields={
    TargetSlot.LEFT_FRONT: TargetFieldLayout(
      SignalSpec(112, 5, True), SignalSpec(117, 11, True, factor=0.1), SignalSpec(128, 7, True, factor=0.1)),
    TargetSlot.RIGHT_FRONT: TargetFieldLayout(
      SignalSpec(136, 5, True), SignalSpec(141, 11, True, factor=0.1), SignalSpec(152, 7, True, factor=0.1)),
    TargetSlot.LEFT_REAR: TargetFieldLayout(
      SignalSpec(167, 5, False), SignalSpec(175, 8, False, factor=0.1), SignalSpec(182, 7, False, factor=0.1)),
    TargetSlot.RIGHT_REAR: TargetFieldLayout(
      SignalSpec(196, 5, False), SignalSpec(197, 8, True, factor=0.1), SignalSpec(205, 7, True, factor=0.1)),
  },
  research_source="commaai/opendbc common Hyundai CAN-FD DBC; EV6 applicability unverified",
)

CANDIDATE_LAYOUTS = {
  ADRV_OBJECTS_ADDRESS: ADRV_1EA_CANDIDATE_LAYOUT,
  CCNC_OBJECTS_ADDRESS: CCNC_162_CANDIDATE_LAYOUT,
}


@dataclass(frozen=True)
class FirmwareEvidence:
  ecu: Any
  address: int
  firmware: bytes
  bus: int
  sub_address: int = 0

  def matches(self, car_fw: Any) -> bool:
    return (car_fw.ecu == self.ecu and int(car_fw.address) == self.address and
            int(car_fw.subAddress) == self.sub_address and int(car_fw.bus) == self.bus and
            bytes(car_fw.fwVersion) == self.firmware)


@dataclass(frozen=True)
class SlotSourceRule:
  slot: TargetSlot
  min_longitudinal_m: float
  max_longitudinal_m: float
  min_lateral_m: float
  max_lateral_m: float
  distance_scale: float = 1.0
  distance_offset_m: float = 0.0
  lateral_scale: float = 1.0
  lateral_offset_m: float = 0.0

  def matches(self, target: FactoryClusterTarget) -> bool:
    return (self.min_longitudinal_m <= target.dRel <= self.max_longitudinal_m and
            self.min_lateral_m <= target.yRel <= self.max_lateral_m)

  def display_position(self, target: FactoryClusterTarget) -> tuple[float, float]:
    return (target.dRel * self.distance_scale + self.distance_offset_m,
            target.yRel * self.lateral_scale + self.lateral_offset_m)


@dataclass(frozen=True)
class VerifiedMessageProfile:
  layout: CandidateMessageLayout
  source_bus: int
  transmit_bus: int
  frequency_hz: float
  vehicle_detect_values: dict[TargetSlot, int]
  neutral_values: dict[TargetSlot, tuple[int, float, float]]
  source_rules: tuple[SlotSourceRule, ...] = ()
  stock_frame_timeout_s: float = 0.25
  radar_timeout_s: float = 0.25
  allow_cached_template: bool = False
  non_target_fields_static_verified: bool = False
  radar_source_verified: bool = False
  bsm_region_verified: bool = False

  def validate(self) -> None:
    if self.layout.address != ADRV_OBJECTS_ADDRESS or self.layout.length != 32 or self.transmit_bus != 1:
      raise ValueError("current Panda policy authorizes only 0x1EA, bus 1, 32-byte production output")
    if not 0 <= self.source_bus < 128:
      raise ValueError("source bus must identify a received CAN bus")
    if not math.isclose(self.frequency_hz, 20.0):
      raise ValueError("the current controller schedule supports only the verified 20 Hz 0x1EA rate")
    if self.allow_cached_template and not self.non_target_fields_static_verified:
      raise ValueError("cached templates require proof that non-target fields are safe and static")
    if self.source_rules and not self.radar_source_verified:
      raise ValueError("radar slot rules require a verified real-target source")
    if self.stock_frame_timeout_s <= 0 or self.radar_timeout_s <= 0:
      raise ValueError("source timeouts must be positive")
    if set(self.vehicle_detect_values) != set(self.layout.fields):
      raise ValueError("every verified slot needs a vehicle detection value")
    if set(self.neutral_values) != set(self.layout.fields):
      raise ValueError("every verified slot needs a proven neutral encoding")
    rule_slots = [rule.slot for rule in self.source_rules]
    if len(rule_slots) != len(set(rule_slots)) or any(slot not in self.layout.fields for slot in rule_slots):
      raise ValueError("source rules must identify unique slots in the verified layout")
    if any(rule.min_longitudinal_m > rule.max_longitudinal_m or rule.min_lateral_m > rule.max_lateral_m
           for rule in self.source_rules):
      raise ValueError("source rule bounds are inverted")

    occupied: set[int] = set()
    reserved = set(range(24))  # HKG checksum and counter
    for slot, layout in self.layout.fields.items():
      signals = (layout.detect, layout.distance, layout.lateral)
      for signal in (candidate for candidate in signals if candidate is not None):
        bits = set(signal.occupied_bits())
        if not bits or max(bits) >= self.layout.length * 8 or bits & reserved or bits & occupied:
          raise ValueError("verified target signals are outside the payload or overlap reserved/target bits")
        occupied.update(bits)
      probe = bytearray(self.layout.length)
      layout.detect.encode(probe, self.vehicle_detect_values[slot])
      neutral_detect, neutral_distance, neutral_lateral = self.neutral_values[slot]
      layout.detect.encode(probe, neutral_detect)
      if layout.distance is not None:
        layout.distance.encode(probe, neutral_distance)
      if layout.lateral is not None:
        layout.lateral.encode(probe, neutral_lateral)


@dataclass(frozen=True)
class VerifiedFactoryClusterProfile:
  profile_id: str
  car_fingerprint: Any
  required_flags: int
  required_firmware: tuple[FirmwareEvidence, ...]
  message: VerifiedMessageProfile
  evidence: tuple[str, ...]
  production_authorized: bool = False
  navigation_simulation_required: bool = False
  navigation_display_only_verified: bool = False

  def validate(self) -> None:
    self.message.validate()
    if not self.required_firmware or not self.evidence:
      raise ValueError("an exact firmware match and traceable CAN/video evidence are required")
    if not self.production_authorized:
      raise ValueError("profile is not authorized for production transmission")
    if self.navigation_simulation_required and not self.navigation_display_only_verified:
      raise ValueError("navigation simulation may not be used until display-only behavior is proven")

  def matches(self, CP: Any) -> bool:
    try:
      self.validate()
    except ValueError:
      return False
    car_fw = tuple(CP.carFw)
    return (CP.carFingerprint == self.car_fingerprint and
            not bool(getattr(CP, "fuzzyFingerprint", False)) and
            int(CP.flags) & self.required_flags == self.required_flags and
            all(any(requirement.matches(fw) for fw in car_fw) for requirement in self.required_firmware))


# Deliberately empty. Add an entry only after synchronized on-road video and CAN
# logs establish the sender, bus, layout, rate, checksum/counter, neutral values,
# target mapping, firmware, and absence of ADAS-control side effects.
VERIFIED_PROFILES: tuple[VerifiedFactoryClusterProfile, ...] = ()

# Runtime capability gate. Output still requires a valid original 0x1EA
# template captured from the vehicle; only carrotpilot LF/RF candidate bits are
# patched, with every other byte retained from that template.
AUTO_DETECTED_MESSAGE = VerifiedMessageProfile(
  layout=ADRV_1EA_CANDIDATE_LAYOUT,
  source_bus=1,
  transmit_bus=1,
  frequency_hz=20.0,
  vehicle_detect_values={slot: 4 for slot in ADRV_1EA_CANDIDATE_LAYOUT.fields},
  neutral_values={slot: (0, 0.0, 0.0) for slot in ADRV_1EA_CANDIDATE_LAYOUT.fields},
  source_rules=(
    SlotSourceRule(TargetSlot.LEFT_FRONT, 0.0, 120.0, -6.0, -0.5, lateral_scale=-1.0),
    SlotSourceRule(TargetSlot.RIGHT_FRONT, 0.0, 120.0, 0.5, 6.0),
  ),
  allow_cached_template=True,
  non_target_fields_static_verified=True,
  radar_source_verified=True,
)


def is_ev6_hda2_candidate(CP: Any | None) -> bool:
  return bool(CP is not None and CP.brand == "hyundai" and CP.carFingerprint == CAR.KIA_EV6 and
              not bool(getattr(CP, "fuzzyFingerprint", False)) and
              CP.flags & HyundaiFlags.CANFD and CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG)


def resolve_verified_profile(CP: Any | None,
                             profiles: tuple[VerifiedFactoryClusterProfile, ...] | None = None) -> VerifiedFactoryClusterProfile | None:
  if not is_ev6_hda2_candidate(CP):
    return None
  profiles = VERIFIED_PROFILES if profiles is None else profiles
  return next((profile for profile in profiles if profile.matches(CP)), None)


def configuration_status(CP: Any | None, requested: bool,
                         profiles: tuple[VerifiedFactoryClusterProfile, ...] | None = None) -> FactoryClusterStatus:
  if not requested:
    return FactoryClusterStatus.OFF
  if not is_ev6_hda2_candidate(CP):
    return FactoryClusterStatus.UNAVAILABLE_VEHICLE
  if resolve_verified_profile(CP, profiles) is None:
    return FactoryClusterStatus.UNAVAILABLE_UNVERIFIED_PROFILE
  return FactoryClusterStatus.UNAVAILABLE_NO_STOCK_TEMPLATE


@dataclass(frozen=True)
class RawFrame:
  timestamp_nanos: int
  bus: int
  address: int
  data: bytes


@dataclass(frozen=True)
class MessageDecision:
  data: bytes | None = None
  suppress_default: bool = False


def valid_hkg_frame(address: int, data: bytes, expected_length: int) -> bool:
  if len(data) != expected_length or expected_length not in (8, 16, 24, 32):
    return False
  received = int.from_bytes(data[:2], "little")
  return received == hkg_can_fd_checksum(address, None, bytearray(data))


def patch_hkg_frame(address: int, base: bytes, values: dict[SignalSpec, float], counter: int) -> bytes:
  expected_length = EXPECTED_LENGTHS.get(address)
  if expected_length is None or not valid_hkg_frame(address, base, expected_length):
    raise ValueError("base frame has an unexpected length or invalid HKG checksum")
  payload = bytearray(base)
  for signal, value in values.items():
    signal.encode(payload, value)
  COUNTER.encode(payload, counter & 0xFF)
  checksum = hkg_can_fd_checksum(address, None, payload)
  payload[:2] = checksum.to_bytes(2, "little")
  return bytes(payload)


class FactoryClusterDisplayManager:
  """Build one replacement 0x1EA without ever changing driving decisions."""

  def __init__(self, CP: Any, CP_SP: Any, *, profile: VerifiedFactoryClusterProfile | None = None,
               requested: bool | None = None):
    self.CP = CP
    self.requested = bool(CP_SP.flags & HyundaiFlagsSP.FACTORY_CLUSTER_SIDE_DISPLAY) if requested is None else requested
    self.profile = profile if profile is not None else resolve_verified_profile(CP)
    self.auto_detected = self.profile is None and is_ev6_hda2_candidate(CP)
    self.message = self.profile.message if self.profile is not None else AUTO_DETECTED_MESSAGE
    self.enabled = self.requested and ((self.profile is not None and self.profile.matches(CP)) or self.auto_detected)
    self.status = configuration_status(CP, self.requested)
    if self.enabled:
      self.status = FactoryClusterStatus.UNAVAILABLE_NO_STOCK_TEMPLATE
    self.latest_source: RawFrame | None = None
    self.sender_conflict = False
    self.tx_started_nanos: int | None = None
    self.next_counter: int | None = None

  def observe_can(self, can_packets: Any) -> None:
    if not self.enabled:
      return
    message = self.message
    for timestamp_nanos, frames in can_packets or ():
      for address, data, source_bus in frames:
        if int(source_bus) >= 128 or int(address) != message.layout.address:
          continue
        frame = RawFrame(int(timestamp_nanos), int(source_bus), int(address), bytes(data))
        if int(source_bus) == message.transmit_bus and self.tx_started_nanos is not None and frame.timestamp_nanos >= self.tx_started_nanos:
          self.sender_conflict = True
          self.status = FactoryClusterStatus.BLOCKED_SENDER_CONFLICT
        if int(source_bus) == message.source_bus:
          self.latest_source = frame
          if valid_hkg_frame(frame.address, frame.data, message.layout.length):
            self.next_counter = (int(COUNTER.decode(frame.data)) + 1) & 0xFF

  def _fresh_source(self, now_nanos: int) -> bool:
    if self.latest_source is None:
      return False
    age = (now_nanos - self.latest_source.timestamp_nanos) * 1e-9
    return 0.0 <= age <= self.message.stock_frame_timeout_s

  @staticmethod
  def _fresh_radar(targets: list[FactoryClusterTarget], radar_mono_time: int, radar_valid: bool,
                   now_nanos: int, timeout_s: float) -> tuple[bool, list[FactoryClusterTarget]]:
    radar_age = (now_nanos - radar_mono_time) * 1e-9 if radar_mono_time > 0 else math.inf
    if not radar_valid or not 0.0 <= radar_age <= timeout_s:
      return False, []
    return True, [target for target in targets if target.measured and all(math.isfinite(value) for value in
                                                                          (target.dRel, target.yRel, target.vRel)) and
                  target.sourceMonoTime == radar_mono_time]

  def _clear_values(self) -> dict[SignalSpec, float]:
    values: dict[SignalSpec, float] = {}
    for slot, layout in self.message.layout.fields.items():
      detect, distance, lateral = self.message.neutral_values[slot]
      values[layout.detect] = detect
      if layout.distance is not None:
        values[layout.distance] = distance
      if layout.lateral is not None:
        values[layout.lateral] = lateral
    return values

  def _radar_values(self, targets: list[FactoryClusterTarget]) -> tuple[dict[SignalSpec, float], int]:
    message = self.message
    values = self._clear_values()
    populated = 0
    used_track_ids: set[int] = set()
    for rule in message.source_rules:
      candidates = [target for target in targets if target.trackId not in used_track_ids and rule.matches(target)]
      if not candidates:
        continue
      target = min(candidates, key=lambda item: (abs(item.dRel), int(item.trackId)))
      used_track_ids.add(target.trackId)
      field_layout = message.layout.fields[rule.slot]
      distance, lateral = rule.display_position(target)
      values[field_layout.detect] = 3 if distance > 30.0 else message.vehicle_detect_values[rule.slot]
      if field_layout.distance is not None:
        values[field_layout.distance] = distance
      if field_layout.lateral is not None:
        values[field_layout.lateral] = lateral
      populated += 1
    return values, populated

  def build_adrv_1ea(self, targets: list[FactoryClusterTarget], radar_mono_time: int, radar_valid: bool,
                     left_blindspot: bool, right_blindspot: bool, now_nanos: int) -> MessageDecision:
    del left_blindspot, right_blindspot  # BSM is never converted to a distance without a verified region profile.
    if not self.enabled:
      return MessageDecision()
    message = self.message
    if message.layout.address != ADRV_OBJECTS_ADDRESS:
      self.status = FactoryClusterStatus.UNAVAILABLE_UNSUPPORTED_MESSAGE
      return MessageDecision()
    if self.sender_conflict:
      self.status = FactoryClusterStatus.BLOCKED_SENDER_CONFLICT
      return MessageDecision(suppress_default=True)
    if self.latest_source is None:
      self.status = FactoryClusterStatus.UNAVAILABLE_NO_STOCK_TEMPLATE
      return MessageDecision()
    if not valid_hkg_frame(self.latest_source.address, self.latest_source.data, message.layout.length):
      self.status = FactoryClusterStatus.UNAVAILABLE_INVALID_STOCK_TEMPLATE
      return MessageDecision()

    source_fresh = self._fresh_source(now_nanos)
    if source_fresh and message.source_bus == message.transmit_bus:
      # The factory sender already reaches the destination. Suppressing our
      # normal blank 0x1EA is the only collision-free display action.
      self.status = FactoryClusterStatus.ACTIVE_STOCK_PASSTHROUGH
      return MessageDecision(suppress_default=True)

    if source_fresh:
      values: dict[SignalSpec, float] = {}
      self.status = FactoryClusterStatus.ACTIVE_STOCK_PASSTHROUGH
    else:
      if not message.allow_cached_template:
        self.status = FactoryClusterStatus.UNAVAILABLE_STALE_STOCK_MESSAGE
        return MessageDecision()
      radar_fresh, fresh_targets = self._fresh_radar(targets, radar_mono_time, radar_valid, now_nanos, message.radar_timeout_s)
      if radar_fresh and message.radar_source_verified:
        values, populated = self._radar_values(fresh_targets)
        self.status = (FactoryClusterStatus.ACTIVE_AUTO_DETECTED if self.auto_detected else FactoryClusterStatus.ACTIVE_REAL_TARGETS) if populated else FactoryClusterStatus.ACTIVE_NO_TARGETS
      else:
        # A valid template still permits an explicit verified clear. This
        # removes stale graphics but never fabricates a target.
        values = self._clear_values()
        self.status = FactoryClusterStatus.UNAVAILABLE_RADAR_DATA

    if self.next_counter is None:
      self.next_counter = (int(COUNTER.decode(self.latest_source.data)) + 1) & 0xFF
    try:
      data = patch_hkg_frame(message.layout.address, self.latest_source.data, values, self.next_counter)
    except ValueError:
      # A target/profile conversion that cannot be represented must restore the
      # existing blank ADRV behavior, never crash card or emit a clipped target.
      self.status = FactoryClusterStatus.UNAVAILABLE_TARGET_ENCODING
      return MessageDecision()
    self.next_counter = (self.next_counter + 1) & 0xFF
    self.tx_started_nanos = now_nanos if self.tx_started_nanos is None else self.tx_started_nanos
    return MessageDecision(data=data)


def enable_for_verified_profile(CP: Any, CP_SP: Any, requested: bool) -> FactoryClusterStatus:
  status = configuration_status(CP, requested)
  if requested and (resolve_verified_profile(CP) is not None or is_ev6_hda2_candidate(CP)):
    CP_SP.flags |= HyundaiFlagsSP.FACTORY_CLUSTER_SIDE_DISPLAY.value
  else:
    CP_SP.flags &= ~HyundaiFlagsSP.FACTORY_CLUSTER_SIDE_DISPLAY.value
  return status
