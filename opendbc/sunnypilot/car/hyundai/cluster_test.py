from __future__ import annotations

from opendbc.car.hyundai.values import CAR, HyundaiFlags, HyundaiSafetyFlags
from opendbc.car.structs import CarParams


def supports_hkg_cluster_test(CP: CarParams | None) -> bool:
  """Return whether the narrow, park-only EV6 cluster test may be armed."""
  return bool(CP is not None and CP.brand == "hyundai" and CP.carFingerprint == CAR.KIA_EV6 and
              CP.flags & HyundaiFlags.CANFD and CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG and
              CP.flags & HyundaiFlags.EV and CP.safetyConfigs and
              CP.safetyConfigs[-1].safetyModel == CarParams.SafetyModel.hyundaiCanfd)


def configure_hkg_cluster_test(CP: CarParams, enabled: bool) -> None:
  """Arm the matching Panda safety gate before the car controller is built.

  The test flag changes only the two display-message allowlist entries. Panda
  still independently requires Park, standstill, disengagement, valid CRC, and
  a restricted display-only payload before accepting either frame.
  """
  if CP.brand != "hyundai":
    return

  CP.flags &= ~HyundaiFlags.CANFD_HKG_CLUSTER_TEST.value
  for config in CP.safetyConfigs:
    config.safetyParam &= ~HyundaiSafetyFlags.CANFD_HKG_CLUSTER_TEST.value

  if enabled and supports_hkg_cluster_test(CP):
    CP.flags |= HyundaiFlags.CANFD_HKG_CLUSTER_TEST.value
    CP.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CANFD_HKG_CLUSTER_TEST.value
