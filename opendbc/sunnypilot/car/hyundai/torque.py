from __future__ import annotations

import math

from opendbc.car.hyundai.values import CANFD_CAR, HyundaiFlags, HyundaiSafetyFlags
from opendbc.car.structs import CarParams


NATURAL_STEERING_DEMAND_RATES = (
  (0.15, 3),
  (0.30, 4),
  (0.50, 6),
  (0.75, 8),
  (1.00, 10),
)


def get_natural_steering_rate(target_torque: int, steer_max: int) -> int:
  """Return the comfort slew-rate cap for the requested torque magnitude."""
  if steer_max <= 0:
    return NATURAL_STEERING_DEMAND_RATES[0][1]

  ratio = min(abs(target_torque) / float(steer_max), 1.0)
  for breakpoint, rate in NATURAL_STEERING_DEMAND_RATES:
    if ratio <= breakpoint:
      return rate
  return NATURAL_STEERING_DEMAND_RATES[-1][1]


class NaturalSteeringTorqueShaper:
  """Second-order comfort shaper applied before the existing HKG torque limits.

  The requested torque remains inside the existing speed-dependent steer_max envelope.
  Panda safety and apply_driver_steer_torque_limits remain the authoritative limits.
  """

  RATE_ACCEL_PER_FRAME = 1.0
  UNWIND_MIN_RATE = 6
  UNWIND_RATE_BONUS = 2
  REVERSAL_RATE = 10

  def __init__(self) -> None:
    self.slew_rate = 0.0
    self.phase = "inactive"

  def reset(self) -> None:
    self.slew_rate = 0.0
    self.phase = "inactive"

  def _move_toward(self, current_torque: int, target_torque: int, max_rate: int) -> int:
    diff = target_torque - current_torque
    if diff == 0:
      self.slew_rate = max(0.0, self.slew_rate - self.RATE_ACCEL_PER_FRAME)
      return target_torque

    remaining = abs(diff)

    # Acceleration-limited slew-rate with a braking envelope. This produces a
    # progressive turn-in and a progressive ease-out instead of a step change
    # in torque rate at the start/end of a steering request.
    braking_rate = math.sqrt(2.0 * self.RATE_ACCEL_PER_FRAME * remaining)
    desired_rate = min(float(max_rate), braking_rate)

    if desired_rate > self.slew_rate:
      self.slew_rate = min(desired_rate, self.slew_rate + self.RATE_ACCEL_PER_FRAME)
    else:
      self.slew_rate = max(desired_rate, self.slew_rate - self.RATE_ACCEL_PER_FRAME)

    step = max(1, min(remaining, int(round(self.slew_rate))))
    shaped_torque = current_torque + (step if diff > 0 else -step)
    if shaped_torque == target_torque:
      self.slew_rate = 0.0
    return shaped_torque

  def update(self, target_torque: int, current_torque: int, steer_max: int, active: bool) -> int:
    if not active:
      self.reset()
      return 0

    # This layer is comfort-only. It can never request more than the existing
    # Python-side dynamic maximum, and Panda still independently enforces the
    # same safety envelope after CAN packing.
    target_torque = max(-steer_max, min(steer_max, target_torque))

    if target_torque == current_torque:
      self.phase = "hold"
      self.slew_rate = max(0.0, self.slew_rate - self.RATE_ACCEL_PER_FRAME)
      return target_torque

    # Direction reversals must unload through zero before torque is built in the
    # opposite direction. Once zero is reached, the next frame starts a fresh
    # S-curve turn-in in the new direction.
    if current_torque != 0 and target_torque != 0 and ((current_torque > 0) != (target_torque > 0)):
      self.phase = "reversal"
      shaped_torque = self._move_toward(current_torque, 0, self.REVERSAL_RATE)
      if shaped_torque == 0:
        self.slew_rate = 0.0
      return shaped_torque

    # Turn-in uses the requested torque ratio table. Unwind is deliberately a
    # little quicker than turn-in, while still using the same second-order
    # S-curve and remaining below the existing steer_delta/Panda limits.
    if abs(target_torque) > abs(current_torque):
      self.phase = "turn_in"
      max_rate = get_natural_steering_rate(target_torque, steer_max)
    else:
      self.phase = "unwind"
      current_rate = get_natural_steering_rate(current_torque, steer_max)
      max_rate = min(10, max(self.UNWIND_MIN_RATE, current_rate + self.UNWIND_RATE_BONUS))

    return self._move_toward(current_torque, target_torque, max_rate)


def supports_low_speed_torque(CP: CarParams | None) -> bool:
  """Eligibility shared by vehicle initialization and the settings UI."""
  return bool(CP is not None and CP.brand == "hyundai" and CP.carFingerprint in CANFD_CAR and
              CP.flags & HyundaiFlags.CANFD and not CP.flags & (HyundaiFlags.ALT_LIMITS | HyundaiFlags.ALT_LIMITS_2) and
              CP.steerControlType == CarParams.SteerControlType.torque and not CP.dashcamOnly and
              CP.safetyConfigs and CP.safetyConfigs[-1].safetyModel == CarParams.SafetyModel.hyundaiCanfd)


def configure_low_speed_torque(CP: CarParams, enabled: bool) -> None:
  """Configure the unified HKG low-speed steering feature before constructing the controller."""
  if CP.brand != "hyundai":
    return

  # Clear the flags first, so missing/disabled settings always restore stock limits.
  CP.flags &= ~(HyundaiFlags.CANFD_DYNAMIC_TORQUE | HyundaiFlags.CANFD_CREEP_LANE_CHANGE).value
  for config in CP.safetyConfigs:
    config.safetyParam &= ~(HyundaiSafetyFlags.CANFD_DYNAMIC_TORQUE | HyundaiSafetyFlags.CANFD_CREEP_LANE_CHANGE).value

  supported = supports_low_speed_torque(CP)
  if enabled and supported:
    # One setting enables both the 2022 low-speed steering profile and the
    # existing 0-5 km/h automatic lane-change behavior. Panda only needs the
    # torque-profile bit; the lane-change bit remains an openpilot-side flag.
    CP.flags |= (HyundaiFlags.CANFD_DYNAMIC_TORQUE | HyundaiFlags.CANFD_CREEP_LANE_CHANGE).value
    CP.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CANFD_DYNAMIC_TORQUE.value
