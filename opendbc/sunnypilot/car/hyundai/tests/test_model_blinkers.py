import unittest
from unittest.mock import patch

from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.hyundaicanfd import CanBus, create_spas_messages
from opendbc.car.hyundai.values import CAR, CANFD_CAR, HyundaiFlags, HyundaiSafetyFlags
from opendbc.car.structs import CarControl, CarControlSP
from opendbc.can import CANPacker
from opendbc.sunnypilot.car.hyundai.blinkers import configure_model_blinkers, supports_model_blinkers
from opendbc.sunnypilot.car.interfaces import setup_interfaces


def params(model=CAR.KIA_EV6, lka=True, longitudinal=False, offset=False):
  fp = gen_empty_fingerprint()
  if offset:
    fp[4][0x130] = 16
  if lka:
    fp[CanBus(None, fp).CAM][0x50] = 16
  cp = CarInterface.get_params(model, fp, [], longitudinal, False, False)
  sp = CarInterface.get_params_sp(cp, model, fp, [], longitudinal, False, False)
  return cp, sp


class TestModelBlinkers(unittest.TestCase):
  def test_scope_and_reset(self):
    for model in CAR:
      for lka in (False, True):
        cp, sp = params(model, lka)
        supported = model in CANFD_CAR and lka and not cp.dashcamOnly
        self.assertEqual(supports_model_blinkers(cp), supported, (model, lka))
        original = cp.to_dict()
        for value in (None, True, False, "1", "0"):
          setup_interfaces(CarInterface, cp, sp, [{"HkgModelBlinkers": value}])
          enabled = supported and value in (True, "1")
          self.assertEqual(bool(cp.flags & HyundaiFlags.CANFD_ENABLE_BLINKERS), enabled)
          self.assertEqual(bool(cp.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_ENABLE_BLINKERS), enabled)
          if not enabled:
            self.assertEqual(original, cp.to_dict())

  def test_other_brands_and_no_safety(self):
    cp, _ = params()
    cp.brand = 'toyota'
    original = cp.to_dict()
    configure_model_blinkers(cp, True)
    self.assertEqual(original, cp.to_dict())
    cp.brand = 'hyundai'
    cp.safetyConfigs = []
    configure_model_blinkers(cp, True)
    self.assertFalse(cp.flags & HyundaiFlags.CANFD_ENABLE_BLINKERS)
    self.assertFalse(supports_model_blinkers(None))

  def test_uds_init_restore_and_bus_offset(self):
    for offset in (False, True):
      for enabled in (False, True):
        cp, sp = params(offset=offset)
        configure_model_blinkers(cp, enabled)
        with patch('opendbc.car.hyundai.interface.disable_ecu') as disable:
          CarInterface.init(cp, sp, None, None)
          self.assertEqual(disable.call_count, int(enabled))
          if enabled:
            self.assertEqual(disable.call_args.kwargs['bus'], CanBus(cp).ECAN)
            self.assertEqual(disable.call_args.kwargs['addr'], 0x7B1)
            CarInterface.deinit(cp, sp, None, None)
            self.assertEqual(disable.call_args.kwargs['com_cont_req'], b'\x28\x80\x01')
        if offset:
          self.assertEqual(cp.safetyConfigs[0].safetyParam, 0)

  def test_actual_controller_keepalive_without_longitudinal(self):
    for enabled in (False, True):
      cp, sp = params()
      configure_model_blinkers(cp, enabled)
      ci = CarInterface(cp, sp)
      ci.CS.lfa_block_msg = {f"BYTE{i}": 0 for i in range(3, 24)}
      ci.CS.lfa_block_msg['COUNTER'] = 0
      cc = CarControl.new_message()
      cc_sp = CarControlSP()
      for frame in range(101):
        _, messages = ci.apply(cc.as_reader(), cc_sp, frame * 10_000_000)
        addresses = [address for address, _, _ in messages]
        self.assertEqual(0x165 in addresses, enabled)
        self.assertEqual(0x16A in addresses, enabled)
        self.assertEqual(0x7B1 in addresses, enabled and frame % 100 == 0)

  def test_spas_payload_is_neutral_except_signal_crc_and_counter(self):
    cp, _ = params()
    packer = CANPacker('hyundai_canfd_generated')
    for left, right, signal in [(False, False, 0), (True, False, 3), (False, True, 4)]:
      msgs = create_spas_messages(packer, CanBus(cp), left, right)
      for address, data, bus in msgs:
        self.assertEqual(bus, CanBus(cp).ECAN)
        if address == 0x16A:
          self.assertEqual(data[16] >> 5, signal)
        for i in range(3, len(data)):
          self.assertEqual(data[i] & (0x1F if address == 0x16A and i == 16 else 0xFF), 0)


if __name__ == '__main__':
  unittest.main()
