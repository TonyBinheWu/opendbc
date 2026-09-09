import unittest

from opendbc.car import gen_empty_fingerprint, get_safety_config
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, HyundaiFlags, HyundaiSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.hyundai.cluster_test import configure_hkg_cluster_test, supports_hkg_cluster_test
from opendbc.sunnypilot.car.interfaces import setup_interfaces


def ev6_hda2_params():
  CP = CarInterface.get_non_essential_params(CAR.KIA_EV6)
  CP.flags |= HyundaiFlags.CANFD_LKA_STEER_MSG.value
  CP.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CANFD_LKA_STEER_MSG.value
  return CP


class TestHkgClusterTestConfiguration(unittest.TestCase):
  def test_exact_ev6_hda2_canfd_profile_can_be_armed(self):
    CP = ev6_hda2_params()
    self.assertTrue(supports_hkg_cluster_test(CP))

    configure_hkg_cluster_test(CP, True)
    self.assertTrue(CP.flags & HyundaiFlags.CANFD_HKG_CLUSTER_TEST)
    self.assertTrue(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_HKG_CLUSTER_TEST)

    configure_hkg_cluster_test(CP, False)
    self.assertFalse(CP.flags & HyundaiFlags.CANFD_HKG_CLUSTER_TEST)
    self.assertTrue(all(not config.safetyParam & HyundaiSafetyFlags.CANFD_HKG_CLUSTER_TEST for config in CP.safetyConfigs))

  def test_every_required_identity_and_safety_gate_is_enforced(self):
    for change in ("brand", "platform", "canfd", "hda2", "ev", "safety", "no_safety"):
      with self.subTest(change=change):
        CP = ev6_hda2_params()
        if change == "brand":
          CP.brand = "toyota"
        elif change == "platform":
          CP.carFingerprint = CAR.KIA_NIRO_EV
        elif change == "canfd":
          CP.flags &= ~HyundaiFlags.CANFD.value
        elif change == "hda2":
          CP.flags &= ~HyundaiFlags.CANFD_LKA_STEER_MSG.value
        elif change == "ev":
          CP.flags &= ~HyundaiFlags.EV.value
        elif change == "safety":
          CP.safetyConfigs = [get_safety_config(CarParams.SafetyModel.hyundai)]
        else:
          CP.safetyConfigs = []

        self.assertFalse(supports_hkg_cluster_test(CP))
        configure_hkg_cluster_test(CP, True)
        self.assertFalse(CP.flags & HyundaiFlags.CANFD_HKG_CLUSTER_TEST)
        self.assertTrue(all(not config.safetyParam & HyundaiSafetyFlags.CANFD_HKG_CLUSTER_TEST for config in CP.safetyConfigs))

  def test_only_the_vehicle_panda_is_armed(self):
    CP = ev6_hda2_params()
    vehicle_param = CP.safetyConfigs[-1].safetyParam
    CP.safetyConfigs = [
      get_safety_config(CarParams.SafetyModel.noOutput),
      get_safety_config(CarParams.SafetyModel.hyundaiCanfd, vehicle_param),
    ]
    configure_hkg_cluster_test(CP, True)

    self.assertEqual(CP.safetyConfigs[0].safetyParam, 0)
    self.assertTrue(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_HKG_CLUSTER_TEST)

  def test_other_brands_are_not_mutated(self):
    CP = ev6_hda2_params()
    CP.brand = "toyota"
    CP.flags |= HyundaiFlags.CANFD_HKG_CLUSTER_TEST.value
    CP.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CANFD_HKG_CLUSTER_TEST.value
    before = CP.to_dict()
    configure_hkg_cluster_test(CP, False)
    self.assertEqual(CP.to_dict(), before)

  def test_setup_requires_master_and_local_one_shot_settings_together(self):
    for master, one_shot, expected in ((False, False, False), (True, False, False), (False, True, False), (True, True, True)):
      with self.subTest(master=master, one_shot=one_shot):
        CP = ev6_hda2_params()
        CP_SP = CarInterface.get_params_sp(CP, CAR.KIA_EV6, gen_empty_fingerprint(), [], False, False, False)
        setup_interfaces(CarInterface, CP, CP_SP, [
          {"HkgStockClusterDisplay": master},
          {"HkgStockClusterDisplayTest": one_shot},
        ])
        self.assertEqual(bool(CP.flags & HyundaiFlags.CANFD_HKG_CLUSTER_TEST), expected)
        self.assertEqual(bool(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_HKG_CLUSTER_TEST), expected)


if __name__ == "__main__":
  unittest.main()
