import unittest

from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.common import CANPackerSafety, make_msg
from opendbc.safety.tests.libsafety import libsafety_py


STATUS_PAGES = (
  {},
  {"LCA_LEFT_ARROW": 1, "LCA_LEFT_ICON": 2, "LANELINE_LEFT": 6, "LANELINE_RIGHT": 2},
  {"LCA_RIGHT_ARROW": 1, "LCA_RIGHT_ICON": 2, "LANELINE_LEFT": 2, "LANELINE_RIGHT": 6},
  {"CENTERLINE": 1, "LANELINE_LEFT": 2, "LANELINE_RIGHT": 2, "LANE_HIGHLIGHT": 2,
   "LANE_HIGHLIGHT_DISTANCE": 50, "LANE_LEFT": 1, "LANE_RIGHT": 1, "LANE_ZOOM": 0},
  {"CENTERLINE": 1, "LANELINE_LEFT": 6, "LANELINE_RIGHT": 6, "LANE_HIGHLIGHT": 1,
   "LANE_HIGHLIGHT_DISTANCE": 50, "LANE_LEFT": 1, "LANE_RIGHT": 1, "LANE_ZOOM": 0,
   "HDA_ICON": 2, "LFA_ICON": 2},
  {"CENTERLINE": 1, "LANELINE_LEFT": 4, "LANELINE_RIGHT": 4, "LANE_HIGHLIGHT": 4,
   "LANE_HIGHLIGHT_DISTANCE": 50, "LANE_LEFT": 1, "LANE_RIGHT": 1, "LANE_ZOOM": 0},
  {"NAV_ICON": 1, "HDA_ICON": 1, "LFA_ICON": 1},
  {"NAV_ICON": 2, "HDA_ICON": 2, "LFA_ICON": 2},
  {"NAV_ICON": 4, "HDA_ICON": 3, "LFA_ICON": 3},
)

OBJECT_PAGES = (
  {},
  {"LEAD": 4, "LEAD_DISTANCE": 30, "LEAD_LATERAL": 0,
   "LEAD_ALT": 4, "LEAD_ALT_DISTANCE": 40, "LEAD_ALT_LATERAL": 0,
   "LEAD_LEFT": 8, "LEAD_LEFT_DISTANCE": 20, "LEAD_LEFT_LATERAL": 2,
   "LEAD_RIGHT": 10, "LEAD_RIGHT_DISTANCE": 25, "LEAD_RIGHT_LATERAL": 2},
  {"LEAD": 12, "LEAD_DISTANCE": 30, "LEAD_LATERAL": 0,
   "LEAD_ALT": 2, "LEAD_ALT_DISTANCE": 40, "LEAD_ALT_LATERAL": 0,
   "LEAD_LEFT": 6, "LEAD_LEFT_DISTANCE": 20, "LEAD_LEFT_LATERAL": 2,
   "LEAD_RIGHT": 2, "LEAD_RIGHT_DISTANCE": 25, "LEAD_RIGHT_LATERAL": 2},
)

NEUTRAL_STATUS = {
  "LANELINE_LEFT": 1,
  "LANELINE_LEFT_POSITION": 15,
  "LANELINE_RIGHT": 1,
  "LANELINE_RIGHT_POSITION": 15,
  "LANELINE_CURVATURE": 15,
  "LANE_ZOOM": 1,
}


class TestHkgClusterDisplaySafety(unittest.TestCase):
  BASE_PARAM = (HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS |
                HyundaiSafetyFlags.CANFD_HKG_CLUSTER_TEST)

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self._reset(self.BASE_PARAM)

  def _reset(self, param):
    self.assertEqual(self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, param), 0)
    self.safety.init_tests()

  def _vehicle_state(self, *, gear=0, gas=0, speed=0.0):
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("ACCELERATOR", 1, {
      "GEAR": gear,
      "ACCELERATOR_PEDAL": gas,
    }))
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("WHEEL_SPEEDS", 1, {
      "WHL_SpdFLVal": speed,
      "WHL_SpdFRVal": speed,
      "WHL_SpdRLVal": speed,
      "WHL_SpdRRVal": speed,
    }))

  def _status(self, values=None, bus=1):
    page = dict(NEUTRAL_STATUS)
    page.update(values or {})
    return self.packer.make_can_msg_safety("CCNC_0x161", bus, page)

  def _objects(self, values=None, bus=1):
    return self.packer.make_can_msg_safety("CCNC_0x162", bus, values or {})

  def test_allowlisted_pages(self):
    self._vehicle_state()
    for page in STATUS_PAGES:
      self.assertTrue(self.safety.safety_tx_hook(self._status(page)))
    for page in OBJECT_PAGES:
      self.assertFalse(self.safety.safety_tx_hook(self._objects(page)))

  def test_init_variants(self):
    variants = (
      self.BASE_PARAM,
      self.BASE_PARAM | HyundaiSafetyFlags.CANFD_LKA_STEER_MSG_ALT,
      self.BASE_PARAM | HyundaiSafetyFlags.LONG,
    )
    for param in variants:
      with self.subTest(param=int(param)):
        self._reset(param)
        self._vehicle_state()
        self.assertTrue(self.safety.safety_tx_hook(self._status()))

  def test_flag_bus_length_checksum_and_budget(self):
    self._reset(self.BASE_PARAM & ~HyundaiSafetyFlags.CANFD_HKG_CLUSTER_TEST)
    self._vehicle_state()
    self.assertFalse(self.safety.safety_tx_hook(self._status()))

    self._reset(self.BASE_PARAM)
    self._vehicle_state()
    self.assertFalse(self.safety.safety_tx_hook(self._status(bus=0)))
    self.assertFalse(self.safety.safety_tx_hook(make_msg(1, 0x161, 8)))

    valid = self._status()
    corrupt = bytearray(valid.data)
    corrupt[0] ^= 1
    self.assertFalse(self.safety.safety_tx_hook(make_msg(1, 0x161, 32, bytes(corrupt))))

    for count in range(700):
      self.assertTrue(self.safety.safety_tx_hook(valid), count)
    self.assertFalse(self.safety.safety_tx_hook(valid))

  def test_vehicle_interlocks(self):
    for vehicle_state, controls_allowed, expected in (
      (None, False, False),
      ({}, False, True),
      ({"gear": 5}, False, False),
      ({"gas": 1}, False, False),
      ({"speed": 1.0}, False, False),
      ({}, True, False),
    ):
      with self.subTest(vehicle_state=vehicle_state, controls_allowed=controls_allowed):
        self._reset(self.BASE_PARAM)
        if vehicle_state is not None:
          self._vehicle_state(**vehicle_state)
        self.safety.set_controls_allowed(controls_allowed)
        self.assertEqual(self.safety.safety_tx_hook(self._status()), expected)

    self._reset(self.BASE_PARAM)
    self._vehicle_state()
    self.safety.set_controls_allowed_lateral(True)
    self.assertFalse(self.safety.safety_tx_hook(self._status()))

  def test_forbidden_status_fields_and_values(self):
    self._vehicle_state()
    forbidden_pages = (
      {"FCA_ICON": 1}, {"BCA_LEFT": 1}, {"TARGET": 1}, {"LANELINE_CURVATURE": 16},
      {"LANELINE_LEFT": 3}, {"LANELINE_LEFT_POSITION": 31}, {"LANE_HIGHLIGHT": 3},
      {"LANE_HIGHLIGHT_DISTANCE": 101}, {"LANE_LEFT": 2}, {"LANE_ZOOM": 2},
      {"HDA_ICON": 4}, {"NAV_ICON": 3}, {"LFA_ICON": 4},
      {"LCA_LEFT_ICON": 3}, {"LCA_RIGHT_ICON": 3}, {"ALERTS_2": 1},
      {"SOUNDS_1": 1}, {"SETSPEED": 1}, {"BACKGROUND": 1}, {"DAW_ICON": 1},
    )
    for values in forbidden_pages:
      with self.subTest(values=values):
        self.assertFalse(self.safety.safety_tx_hook(self._status(values)))

  def test_forbidden_object_fields_and_values(self):
    self._vehicle_state()
    forbidden_pages = (
      {"COUNTRY": 1}, {"SPEEDLIMIT": 50}, {"VIBRATE": 1},
      {"LEAD": 15}, {"LEAD_ALT": 5}, {"LEAD_DISTANCE": 100.1}, {"LEAD_LATERAL": 10.1},
      {"LEAD_LEFT_REAR_STATUS": 1}, {"FAULT_HDA": 1},
    )
    for values in forbidden_pages:
      with self.subTest(values=values):
        self.assertFalse(self.safety.safety_tx_hook(self._objects(values)))


if __name__ == "__main__":
  unittest.main()
