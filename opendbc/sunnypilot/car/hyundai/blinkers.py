from __future__ import annotations

from opendbc.car.hyundai.values import CANFD_CAR, HyundaiFlags, HyundaiSafetyFlags
from opendbc.car.structs import CarParams


def supports_model_blinkers(CP: CarParams | None) -> bool:
  """taco2 SPAS transport requires the CAN-FD LKA/HDA2 architecture."""
  return bool(CP is not None and CP.brand == "hyundai" and CP.carFingerprint in CANFD_CAR and
              CP.flags & HyundaiFlags.CANFD and CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG and
              not CP.dashcamOnly and CP.safetyConfigs and
              CP.safetyConfigs[-1].safetyModel == CarParams.SafetyModel.hyundaiCanfd)


def configure_model_blinkers(CP: CarParams, enabled: bool) -> None:
  if CP.brand != "hyundai":
    return
  CP.flags &= ~HyundaiFlags.CANFD_ENABLE_BLINKERS.value
  for config in CP.safetyConfigs:
    config.safetyParam &= ~HyundaiSafetyFlags.CANFD_ENABLE_BLINKERS.value
  if enabled and supports_model_blinkers(CP):
    CP.flags |= HyundaiFlags.CANFD_ENABLE_BLINKERS.value
    CP.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CANFD_ENABLE_BLINKERS.value
