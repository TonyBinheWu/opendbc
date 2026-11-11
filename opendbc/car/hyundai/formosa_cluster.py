"""Opt-in OEM cluster overlay, inspired by carrot-wip bc8eb4c2.

Keep original bytes, faults, sounds, object geometry and counters. Never control
steering/braking or manufacture targets. Requires matching Panda RX-sync support.
"""

import math
import time

from opendbc.can.packer import set_value
from opendbc.can.parser import get_raw_value
from opendbc.car.hyundai.hyundaicanfd import CanBus

DISPLAY_FLAG = 1 << 30
SAFETY_FLAG = 1 << 14
MESSAGES = {0x161: "CCNC_0x161", 0x162: "CCNC_0x162", 0x1E0: "LFAHDA_CLUSTER", 0x1EA: "ADRV_0x1ea", 0x200: "ADRV_0x200"}
MAX_SOURCE_AGE_NS = 150_000_000


def supported(CP):
  from opendbc.car.hyundai.values import HyundaiFlags

  return bool(
    CP.flags & HyundaiFlags.FORMOSA_CLUSTER_AVAILABLE
    and CP.flags & HyundaiFlags.CANFD
    and CP.flags & HyundaiFlags.CANFD_CAMERA_SCC
    and not CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG
  )


def configure(CP, enabled):
  from opendbc.car import structs

  if CP.brand != "hyundai":
    return
  CP.flags &= ~DISPLAY_FLAG
  for cfg in CP.safetyConfigs:
    cfg.safetyParam &= ~SAFETY_FLAG
  if enabled and supported(CP) and not CP.dashcamOnly and CP.safetyConfigs and CP.safetyConfigs[-1].safetyModel == structs.CarParams.SafetyModel.hyundaiCanfd:
    CP.flags |= DISPLAY_FLAG
    CP.safetyConfigs[-1].safetyParam |= SAFETY_FLAG


def checksum(dbc, address, data):
  msg = dbc.addr_to_msg[address]
  sig = msg.sigs["CHECKSUM"]
  return sig.calc_checksum(address, sig, bytearray(data))


def overlay(dbc, address, raw, changes):
  """Patch only explicitly named bits, preserving every other raw OEM bit."""
  msg = dbc.addr_to_msg[address]
  data = bytearray(raw)
  for name, value in changes.items():
    sig = msg.sigs[name]
    integer = int(math.floor((value - sig.offset) / sig.factor + 0.5))
    set_value(data, sig, integer)
  set_value(data, msg.sigs["CHECKSUM"], checksum(dbc, address, data))
  return bytes(data)


class ClusterDisplay:
  def __init__(self):
    self.counters = {}

  def messages(self, parser, CP, CC, CS, lfa_icon, now=None):
    if not CP.flags & DISPLAY_FLAG or not supported(CP):
      return []
    now = time.monotonic_ns() if now is None else now
    output = []
    for address in MESSAGES:
      item = parser.raw_messages.get(address)
      if item is None:
        continue
      received, raw = item
      msg = parser.dbc.addr_to_msg[address]
      if not 0 <= now - received < MAX_SOURCE_AGE_NS or len(raw) != msg.size:
        continue
      if int.from_bytes(raw[:2], "little") != checksum(parser.dbc, address, raw):
        continue
      # Publish each original snapshot at most once, including counter wrap.
      if self.counters.get(address) == (received, raw[2]):
        continue
      self.counters[address] = received, raw[2]

      def value(name, msg=msg, raw=raw):
        sig = msg.sigs[name]
        integer = get_raw_value(raw, sig)
        if sig.is_signed and integer & (1 << (sig.size - 1)):
          integer -= 1 << sig.size
        return integer * sig.factor + sig.offset

      changes = {}
      if address == 0x161:
        hud = CC.hudControl
        active = bool(CC.latActive)
        changes.update(
          LFA_ICON=2 if active else 1 if lfa_icon else 0,
          CENTERLINE=1 if active else 0,
          LCA_LEFT_ARROW=2 if CC.leftBlinker else 0,
          LCA_RIGHT_ARROW=2 if CC.rightBlinker else 0,
        )
        for side, visible, depart, blind in (
          ("LEFT", hud.leftLaneVisible, hud.leftLaneDepart, CS.out.leftBlindspot),
          ("RIGHT", hud.rightLaneVisible, hud.rightLaneDepart, CS.out.rightBlindspot),
        ):
          changes["LANELINE_" + side] = 4 if visible and (depart or blind) else 2 if visible else 0
        if CP.openpilotLongitudinalControl:
          enabled = bool(CC.longActive)
          changes.update(SETSPEED=3 if enabled else 1, SETSPEED_HUD=2 if enabled else 1, DISTANCE=min(4, max(0, int(hud.leadDistanceBars))))
          speed = CS.out.vCruiseCluster * (1 if CS.is_metric else 1 / 1.609344)
          if math.isfinite(speed) and 0 < speed < 255:
            changes["SETSPEED_SPEED"] = round(speed)
      elif address == 0x162:
        # Translate known box types to cars only when real OEM geometry exists.
        # Hidden objects and all existing truck/person/bicycle classes stay intact.
        for kind, distance in (
          ("LEAD", "LEAD_DISTANCE"),
          ("LEAD_ALT", "LEAD_ALT_DISTANCE"),
          ("LEAD_LEFT", "LEAD_LEFT_DISTANCE"),
          ("LEAD_RIGHT", "LEAD_RIGHT_DISTANCE"),
          ("LEAD_LEFT_REAR_STATUS", "LEAD_LEFT_REAR_DISTANCE"),
          ("LEAD_RIGHT_REAR_STATUS", "LEAD_RIGHT_REAR_DISTANCE"),
        ):
          if value(kind) in (1, 2) and 0 < value(distance) < 204.6:
            changes[kind] = value(kind) + 2
      elif address == 0x1E0:
        changes.update(LFA_ICON=2 if CC.latActive else 0)
        if CP.openpilotLongitudinalControl:
          changes["HDA_ICON"] = 1 if CC.longActive else 0
      output.append((address, overlay(parser.dbc, address, raw, changes), CanBus(CP).ECAN))
    return output
