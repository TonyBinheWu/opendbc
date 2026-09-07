from __future__ import annotations

from opendbc.car.hyundai.values import CANFD_CAR, HyundaiFlags, HyundaiSafetyFlags
from opendbc.car.structs import CarParams


def supports_low_speed_torque(CP: CarParams | None) -> bool:
  """Eligibility shared by vehicle initialization and the settings UI."""
  return bool(CP is not None and CP.brand == "hyundai" and CP.carFingerprint in CANFD_CAR and
              CP.flags & HyundaiFlags.CANFD and not CP.flags & (HyundaiFlags.ALT_LIMITS | HyundaiFlags.ALT_LIMITS_2) and
              CP.steerControlType == CarParams.SteerControlType.torque and not CP.dashcamOnly and
              CP.safetyConfigs and CP.safetyConfigs[-1].safetyModel == CarParams.SafetyModel.hyundaiCanfd)


def configure_low_speed_torque(CP: CarParams, enabled: bool) -> None:
  """Configure both limits before CarInterface constructs the controller; never call onroad."""
  if CP.brand != "hyundai":
    return

  # Clear the old EV6-only flag too, so missing/disabled settings always restore stock limits.
  CP.flags &= ~HyundaiFlags.CANFD_DYNAMIC_TORQUE.value
  for config in CP.safetyConfigs:
    config.safetyParam &= ~HyundaiSafetyFlags.CANFD_DYNAMIC_TORQUE.value

  if enabled and supports_low_speed_torque(CP):
    CP.flags |= HyundaiFlags.CANFD_DYNAMIC_TORQUE.value
    CP.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CANFD_DYNAMIC_TORQUE.value
