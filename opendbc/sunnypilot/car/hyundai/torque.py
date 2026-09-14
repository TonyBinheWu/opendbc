from __future__ import annotations

import math

from opendbc.car.hyundai.values import CANFD_CAR, HyundaiFlags, HyundaiSafetyFlags
from opendbc.car.structs import CarParams


NATURAL_STEERING_DEMAND_RATES = (
  (0.15, 2),
  (0.30, 3),
  (0.50, 4),
  (0.75, 6),
  (1.00, 8),
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
  UNWIND_MIN_RATE = 5
  UNWIND_RATE_BONUS = 1
  REVERSAL_RATE = 6
  REVERSAL_CONFIRM_FRAMES = 5  # 50 ms at the 100 Hz car-control loop

  def __init__(self) -> None:
    self.slew_rate = 0.0
    self.phase = "inactive"
    self.reversal_sign = 0
    self.reversal_frames = 0

  def reset(self) -> None:
    self.slew_rate = 0.0
    self.phase = "inactive"
    self._clear_reversal()

  def _clear_reversal(self) -> None:
    self.reversal_sign = 0
    self.reversal_frames = 0

  @staticmethod
  def _sign(value: int) -> int:
    return 1 if value > 0 else -1 if value < 0 else 0

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

  def _handle_reversal(self, target_torque: int, current_torque: int) -> int | None:
    """Debounce sign flips and unload through zero before building opposite torque.

    Short controller sign flips commonly occur while the desired path curvature is
    still continuous. During the confirmation window we only unload existing torque;
    we never build torque in the opposite direction. A persistent opposite request
    must remain for REVERSAL_CONFIRM_FRAMES before a new S-curve can begin.
    """
    target_sign = self._sign(target_torque)
    current_sign = self._sign(current_torque)

    if self.reversal_sign != 0:
      if target_sign != self.reversal_sign:
        self._clear_reversal()
      else:
        self.reversal_frames += 1
        confirmed = self.reversal_frames > self.REVERSAL_CONFIRM_FRAMES
        self.phase = "reversal" if confirmed else "reversal_confirm"

        if current_torque != 0:
          shaped_torque = self._move_toward(current_torque, 0, self.REVERSAL_RATE)
          if shaped_torque == 0:
            self.slew_rate = 0.0
          return shaped_torque

        # Hold zero until the opposite-direction request has persisted for the
        # full confirmation window. Once confirmed, allow normal turn-in below.
        self.slew_rate = 0.0
        if not confirmed:
          return 0
        self._clear_reversal()
        return None

    if current_sign != 0 and target_sign != 0 and current_sign != target_sign:
      self.reversal_sign = target_sign
      self.reversal_frames = 1
      self.phase = "reversal_confirm"
      shaped_torque = self._move_toward(current_torque, 0, self.REVERSAL_RATE)
      if shaped_torque == 0:
        self.slew_rate = 0.0
      return shaped_torque

    return None

  def update(self, target_torque: int, current_torque: int, steer_max: int, active: bool) -> int:
    if not active:
      self.reset()
      return 0

    # This layer is comfort-only. It can never request more than the existing
    # Python-side dynamic maximum, and Panda still independently enforces the
    # same safety envelope after CAN packing.
    target_torque = max(-steer_max, min(steer_max, target_torque))

    if target_torque == current_torque:
      self._clear_reversal()
      self.phase = "hold"
      self.slew_rate = max(0.0, self.slew_rate - self.RATE_ACCEL_PER_FRAME)
      return target_torque

    if target_torque == 0:
      self._clear_reversal()
    else:
      reversal_torque = self._handle_reversal(target_torque, current_torque)
      if reversal_torque is not None:
        return reversal_torque

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
