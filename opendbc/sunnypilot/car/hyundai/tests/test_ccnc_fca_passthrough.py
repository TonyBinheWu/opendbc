"""ccNC FCA dashboard statuses must reflect OEM camera messages, not fabricated faults."""

from types import SimpleNamespace

from opendbc.car.hyundai.hyundaicanfd import create_ccnc


class RecordingPacker:
  def make_can_msg(self, name, bus, values):
    return name, bus, dict(values)


def _render_ccnc(openpilot_longitudinal: bool, fca_icon: int, fca_fault: int):
  msg_161 = {"ALERTS_2": 0, "ALERTS_3": 0, "ALERTS_5": 0, "SOUNDS_4": 0,
             "LFA_ICON": 0, "FCA_ICON": fca_icon, "FCA_ALT_ICON": fca_icon}
  msg_162 = {"FAULT_FCA": fca_fault}
  msg_1b5 = {"Longitudinal_Distance": 0}
  hud = SimpleNamespace(leftLaneVisible=False, rightLaneVisible=False, leftLaneDepart=False, rightLaneDepart=False,
                        leadDistanceBars=2, leadVisible=False)
  out = SimpleNamespace(steeringAngleDeg=0., vEgo=0., leftBlindspot=False, rightBlindspot=False, vCruiseCluster=0.)
  result = create_ccnc(RecordingPacker(), SimpleNamespace(ECAN=0), openpilot_longitudinal, False,
                       hud, False, False, msg_161, msg_162, msg_1b5, True, out, False, 0)
  return {name: data for name, bus, data in result}


def test_ccnc_preserves_real_fca_fault_in_stock_and_openpilot_long():
  for op_long in (False, True):
    messages = _render_ccnc(op_long, fca_icon=1, fca_fault=1)
    assert messages["CCNC_0x161"]["FCA_ICON"] == 1
    assert messages["CCNC_0x161"]["FCA_ALT_ICON"] == 1
    assert messages["CCNC_0x162"]["FAULT_FCA"] == 1


def test_ccnc_does_not_fabricate_fca_failure_when_source_has_no_fault():
  for op_long in (False, True):
    messages = _render_ccnc(op_long, fca_icon=0, fca_fault=0)
    assert messages["CCNC_0x161"]["FCA_ICON"] == 0
    assert messages["CCNC_0x162"]["FAULT_FCA"] == 0
