import unittest
from types import SimpleNamespace

from opendbc.can import CANParser
from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.hyundaicanfd import create_lfahda_cluster
from opendbc.car.hyundai.values import CAR
from opendbc.car.structs import CarControl, CarControlSP
from opendbc.sunnypilot.car.interfaces import setup_interfaces


class RecordingPacker:
  def __init__(self):
    self.values = None

  def make_can_msg(self, name, bus, values):
    self.values = values
    return name, bus, values


class TestLfaHdaCluster(unittest.TestCase):
  @staticmethod
  def cluster_icons(model, *, lateral_active):
    CP = CarInterface.get_params(model, gen_empty_fingerprint(), [], False, False, False)
    CP_SP = CarInterface.get_params_sp(CP, model, gen_empty_fingerprint(), [], False, False, False)
    setup_interfaces(CarInterface, CP, CP_SP, [])
    CI = CarInterface(CP, CP_SP)
    CC = CarControl(enabled=True, latActive=lateral_active)
    CC_SP = CarControlSP()
    CC_SP.mads.available = True
    CC_SP.mads.enabled = True
    _, messages = CI.apply(CC.as_reader(), CC_SP, 0)

    parser = CANParser("hyundai_canfd_generated", [("LFAHDA_CLUSTER", 20)], CI.CC.CAN.ECAN)
    parser.update([0, messages])
    values = parser.vl["LFAHDA_CLUSTER"]
    return values["HDA_ICON"], values["LFA_ICON"]

  def test_default_hda_icon_follows_controls_enabled(self):
    packer = RecordingPacker()
    create_lfahda_cluster(packer, SimpleNamespace(ECAN=1), True, 1)
    self.assertEqual(packer.values, {"HDA_ICON": 1, "LFA_ICON": 1})

  def test_ev6_hda_icon_follows_active_lfa_icon(self):
    packer = RecordingPacker()
    can = SimpleNamespace(ECAN=1)

    create_lfahda_cluster(packer, can, True, 1, hda_follows_lfa=True)
    self.assertEqual(packer.values, {"HDA_ICON": 0, "LFA_ICON": 1})

    create_lfahda_cluster(packer, can, False, 2, hda_follows_lfa=True)
    self.assertEqual(packer.values, {"HDA_ICON": 1, "LFA_ICON": 2})

    create_lfahda_cluster(packer, can, True, 3, hda_follows_lfa=True)
    self.assertEqual(packer.values, {"HDA_ICON": 0, "LFA_ICON": 3})

  def test_ev6_controller_scopes_follow_lfa_behavior(self):
    self.assertEqual(self.cluster_icons(CAR.KIA_EV6, lateral_active=False), (0, 1))
    self.assertEqual(self.cluster_icons(CAR.KIA_EV6, lateral_active=True), (1, 2))
    self.assertEqual(self.cluster_icons(CAR.HYUNDAI_IONIQ_5, lateral_active=False), (1, 1))
