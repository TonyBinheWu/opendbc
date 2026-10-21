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

def test_ccnc_default_preserves_oem_payload_and_source_dicts():
  source_161 = {
    "ALERTS_2": 5, "ALERTS_3": 17, "SOUNDS_3": 5, "SOUNDS_4": 2,
    "FCA_ICON": 1, "NAV_ICON": 2, "HDA_ICON": 1, "LFA_ICON": 3,
    "LCA_LEFT_ARROW": 2, "SIGNS": 1,
  }
  source_162 = {
    "FAULT_FCA": 1, "FAULT_HDA": 1, "FAULT_LFA": 1, "FAULT_DAS": 1,
    "LEAD": 4, "LEAD_LEFT": 3, "LEAD_RIGHT": 5,
    "LEAD_LEFT_REAR_STATUS": 1, "LEAD_RIGHT_REAR_STATUS": 2,
  }
  hud = SimpleNamespace(leadDistanceBars=2, leadVisible=True)
  out = SimpleNamespace(vCruiseCluster=80)
  original_161, original_162 = dict(source_161), dict(source_162)
  frames = create_ccnc(RecordingPacker(), SimpleNamespace(ECAN=0), True, True,
                       hud, False, False, source_161, source_162, {}, True, out, True, 1,
                       display_options={"master": True, "assist": True, "lanes": True, "objects": True, "signs": True},
                       protocol_verified=False)
  sent = {name: data for name, _, data in frames}
  assert sent["CCNC_0x161"] == original_161
  assert sent["CCNC_0x162"] == original_162
  assert source_161 == original_161
  assert source_162 == original_162


def test_ccnc_verified_opt_in_only_changes_real_lfa_and_cruise_values():
  source_161 = {"LFA_ICON": 0, "HDA_ICON": 3, "FCA_ICON": 1, "NAV_ICON": 2, "ALERTS_2": 5}
  source_162 = {"FAULT_FCA": 1, "LEAD": 4, "LEAD_LEFT": 3, "SIGNS": 2}
  hud = SimpleNamespace(leadDistanceBars=3, leadVisible=False)
  out = SimpleNamespace(vCruiseCluster=80)
  frames = create_ccnc(RecordingPacker(), SimpleNamespace(ECAN=0), True, True, hud, False, False,
                       source_161, source_162, {}, True, out, True, 1,
                       display_options={"master": True, "assist": True}, protocol_verified=True)
  sent = {name: data for name, _, data in frames}
  assert sent["CCNC_0x161"]["LFA_ICON"] == 2
  assert sent["CCNC_0x161"]["SETSPEED_SPEED"] == 80
  assert sent["CCNC_0x161"]["DISTANCE"] == 3
  for key in ("HDA_ICON", "FCA_ICON", "NAV_ICON", "ALERTS_2"):
    assert sent["CCNC_0x161"][key] == source_161[key]
  assert sent["CCNC_0x162"] == source_162

def test_ccnc_source_freshness_requires_both_live_messages():
  from opendbc.car.hyundai.hyundaicanfd import ccnc_camera_frames_fresh

  now = 2_000_000_000
  assert ccnc_camera_frames_fresh(now, now - 100_000_000, now - 250_000_000)
  assert not ccnc_camera_frames_fresh(now, 0, now - 100_000_000)
  assert not ccnc_camera_frames_fresh(now, now - 100_000_000, 0)
  assert not ccnc_camera_frames_fresh(now, now - 301_000_000, now - 100_000_000)
  assert not ccnc_camera_frames_fresh(now, now - 100_000_000, now - 301_000_000)
  assert not ccnc_camera_frames_fresh(now, now + 1, now - 100_000_000)
