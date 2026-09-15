from __future__ import annotations

import math

from opendbc.car.hyundai.values import CANFD_CAR, HyundaiFlags, HyundaiSafetyFlags
from opendbc.car.structs import CarParams


# Base demand-to-rate curve. The curve is interpolated with smoothstep instead
# of hard bins, then warped by a speed-dependent nonlinearity. This keeps the
# steering response continuous while allowing parking/hairpin turns to be
# responsive and highway corrections to be deliberately calm.
NATURAL_STEERING_DEMAND_CURVE = (
  (0.00, 1.5),
  (0.15, 2.0),
  (0.30, 3.0),
  (0.50, 4.0),
  (0.75, 6.0),
  (1.00, 8.0),
)

# m/s. Values between breakpoints are linearly interpolated so steering
# character never changes abruptly at a particular road speed.
NATURAL_STEERING_SPEED_BP = (0.0, 5.0, 11.0, 17.0, 25.0, 35.0)

# <1 expands medium/large demand at low speed; >1 compresses small/medium
# demand at higher speed. This only changes the comfort slew response, never
# the requested torque magnitude or Panda safety limits.
NATURAL_STEERING_NONLINEAR_EXPONENT = (0.72, 0.82, 1.00, 1.15, 1.30, 1.45)

# Scales the demand curve's slew-rate cap. Low speed can still reach 10 units
# per 10 ms frame for tight turns; highway micro-corrections converge toward
# 1 unit per frame before the existing Hyundai/Panda rate limits are applied.
NATURAL_STEERING_RATE_SCALE = (1.30, 1.20, 1.00, 0.80, 0.60, 0.45)

# Error deadband as a fraction of the current speed-dependent steer_max. This
# is intentionally tiny at low speed and grows gradually at highway speed to
# reject controller hunting and side-wind noise. A sustained correction that
# grows beyond the deadband still passes through normally.
NATURAL_STEERING_ERROR_DEADBAND_RATIO = (0.0010, 0.0015, 0.0030, 0.0060, 0.0100, 0.0140)

# Opposite-direction requests must persist longer at higher speed before the
# shaper is allowed to build torque on the other side of zero.
NATURAL_STEERING_REVERSAL_CONFIRM_FRAMES = (3.0, 4.0, 5.0, 7.0, 9.0, 12.0)
NATURAL_STEERING_REVERSAL_RATE_SCALE = (1.15, 1.10, 1.00, 0.85, 0.70, 0.55)


def _interp_profile(v_ego: float, values: tuple[float, ...]) -> float:
  x = max(0.0, float(v_ego))
  if x <= NATURAL_STEERING_SPEED_BP[0]:
    return values[0]
  if x >= NATURAL_STEERING_SPEED_BP[-1]:
    return values[-1]

  for i in range(1, len(NATURAL_STEERING_SPEED_BP)):
    x1 = NATURAL_STEERING_SPEED_BP[i]
    if x <= x1:
      x0 = NATURAL_STEERING_SPEED_BP[i - 1]
      y0, y1 = values[i - 1], values[i]
      t = (x - x0) / (x1 - x0)
      return y0 + (y1 - y0) * t
  return values[-1]


def _smoothstep(t: float) -> float:
  t = max(0.0, min(1.0, t))
  return t * t * (3.0 - 2.0 * t)


def _demand_curve_rate(ratio: float) -> float:
  ratio = max(0.0, min(1.0, ratio))
  if ratio <= NATURAL_STEERING_DEMAND_CURVE[0][0]:
    return NATURAL_STEERING_DEMAND_CURVE[0][1]

  for i in range(1, len(NATURAL_STEERING_DEMAND_CURVE)):
    x1, y1 = NATURAL_STEERING_DEMAND_CURVE[i]
    if ratio <= x1:
      x0, y0 = NATURAL_STEERING_DEMAND_CURVE[i - 1]
      t = _smoothstep((ratio - x0) / (x1 - x0))
      return y0 + (y1 - y0) * t
  return NATURAL_STEERING_DEMAND_CURVE[-1][1]


def get_natural_steering_rate(target_torque: int, steer_max: int, v_ego: float = 11.0) -> float:
  """Return a continuous speed- and demand-adaptive comfort slew-rate cap."""
  if steer_max <= 0:
    return 1.0

  ratio = min(abs(target_torque) / float(steer_max), 1.0)
  exponent = _interp_profile(v_ego, NATURAL_STEERING_NONLINEAR_EXPONENT)
  nonlinear_ratio = ratio ** exponent if ratio > 0.0 else 0.0
  base_rate = _demand_curve_rate(nonlinear_ratio)
  speed_scale = _interp_profile(v_ego, NATURAL_STEERING_RATE_SCALE)
  return max(1.0, min(10.0, base_rate * speed_scale))


def get_natural_steering_deadband(steer_max: int, v_ego: float) -> float:
  ratio = _interp_profile(v_ego, NATURAL_STEERING_ERROR_DEADBAND_RATIO)
  return max(0.5, max(0, steer_max) * ratio)


def get_natural_steering_reversal_confirm_frames(v_ego: float) -> int:
  return max(1, int(round(_interp_profile(v_ego, NATURAL_STEERING_REVERSAL_CONFIRM_FRAMES))))


def get_natural_steering_reversal_rate(v_ego: float) -> float:
  return max(1.0, min(10.0, 6.0 * _interp_profile(v_ego, NATURAL_STEERING_REVERSAL_RATE_SCALE)))


class NaturalSteeringTorqueShaper:
  """Speed-adaptive nonlinear comfort shaper before the existing HKG limits.

  This layer never increases the requested torque magnitude. It only controls
  how quickly torque is allowed to approach the controller request. The
  existing speed-dependent steer_max, driver-torque limiter and Panda safety
  remain authoritative.
  """

  RATE_ACCEL_PER_FRAME = 1.0
  UNWIND_MIN_RATE = 5.0
  UNWIND_RATE_BONUS = 1.0

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

  def _move_toward(self, current_torque: int, target_torque: int, max_rate: float) -> int:
    diff = target_torque - current_torque
    if diff == 0:
      self.slew_rate = max(0.0, self.slew_rate - self.RATE_ACCEL_PER_FRAME)
      return target_torque

    remaining = abs(diff)

    # Acceleration-limited slew-rate with a braking envelope. This produces a
    # progressive turn-in and progressive ease-out instead of a step change in
    # torque rate at the start/end of a steering request.
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

  def _handle_reversal(self, target_torque: int, current_torque: int, v_ego: float) -> int | None:
    """Debounce sign flips and unload through zero before opposite torque."""
    target_sign = self._sign(target_torque)
    current_sign = self._sign(current_torque)
    confirm_frames = get_natural_steering_reversal_confirm_frames(v_ego)
    reversal_rate = get_natural_steering_reversal_rate(v_ego)

    if self.reversal_sign != 0:
      if target_sign != self.reversal_sign:
        self._clear_reversal()
      else:
        self.reversal_frames += 1
        confirmed = self.reversal_frames > confirm_frames
        self.phase = "reversal" if confirmed else "reversal_confirm"

        if current_torque != 0:
          shaped_torque = self._move_toward(current_torque, 0, reversal_rate)
          if shaped_torque == 0:
            self.slew_rate = 0.0
          return shaped_torque

        # Hold zero until the opposite request has persisted for the complete
        # speed-dependent confirmation window.
        self.slew_rate = 0.0
        if not confirmed:
          return 0
        self._clear_reversal()
        return None

    if current_sign != 0 and target_sign != 0 and current_sign != target_sign:
      self.reversal_sign = target_sign
      self.reversal_frames = 1
      self.phase = "reversal_confirm"
      shaped_torque = self._move_toward(current_torque, 0, reversal_rate)
      if shaped_torque == 0:
        self.slew_rate = 0.0
      return shaped_torque

    return None

  def update(self, target_torque: int, current_torque: int, steer_max: int, active: bool, v_ego: float = 11.0) -> int:
    if not active:
      self.reset()
      return 0

    # Comfort-only clamp. This can never request more than the controller asked
    # for or more than the existing Python-side speed-dependent maximum.
    target_torque = max(-steer_max, min(steer_max, target_torque))

    if target_torque == current_torque:
      self._clear_reversal()
      self.phase = "hold"
      self.slew_rate = max(0.0, self.slew_rate - self.RATE_ACCEL_PER_FRAME)
      return target_torque

    if target_torque == 0:
      self._clear_reversal()
    else:
      reversal_torque = self._handle_reversal(target_torque, current_torque, v_ego)
      if reversal_torque is not None:
        return reversal_torque

      # Speed-adaptive error deadband. At highway speeds tiny controller
      # corrections are held until their accumulated difference becomes
      # meaningful. Low-speed deadband remains nearly transparent.
      deadband = get_natural_steering_deadband(steer_max, v_ego)
      if abs(target_torque - current_torque) <= deadband:
        self.phase = "micro_hold"
        self.slew_rate = max(0.0, self.slew_rate - self.RATE_ACCEL_PER_FRAME)
        return current_torque

    speed_scale = _interp_profile(v_ego, NATURAL_STEERING_RATE_SCALE)

    # Turn-in is demand-shaped. Unwind is slightly quicker than turn-in at the
    # same speed, but its floor and bonus are also scaled down progressively at
    # highway speed so release remains calm rather than snapping toward zero.
    if abs(target_torque) > abs(current_torque):
      self.phase = "turn_in"
      max_rate = get_natural_steering_rate(target_torque, steer_max, v_ego)
    else:
      self.phase = "unwind"
      current_rate = get_natural_steering_rate(current_torque, steer_max, v_ego)
      unwind_floor = max(1.0, self.UNWIND_MIN_RATE * speed_scale)
      unwind_bonus = self.UNWIND_RATE_BONUS * speed_scale
      max_rate = min(10.0, max(unwind_floor, current_rate + unwind_bonus))

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
