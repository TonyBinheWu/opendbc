#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/hyundai_common.h"

#define HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(bus) \
  {0x1CF, bus, 8, .check_relay = false},  /* CRUISE_BUTTON */   \

#define HYUNDAI_CANFD_LKA_STEER_MSG_COMMON_TX_MSGS(a_can, e_can) \
  HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(e_can)                        \
  {0x50,  a_can, 16, .check_relay = (a_can) == 0},  /* LKAS */      \
  {0x2A4, a_can, 24, .check_relay = (a_can) == 0},  /* CAM_0x2A4 */ \

#define HYUNDAI_CANFD_LKA_STEER_MSG_ALT_COMMON_TX_MSGS(a_can, e_can) \
  HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(e_can)                        \
  {0x110, a_can, 32, .check_relay = (a_can) == 0},  /* LKAS_ALT */  \
  {0x362, a_can, 32, .check_relay = (a_can) == 0},  /* CAM_0x362 */ \

#define HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(e_can)  \
  {0x12A, e_can, 16, .check_relay = (e_can) == 0},  /* LFA */            \
  {0x1E0, e_can, 16, .check_relay = (e_can) == 0},  /* LFAHDA_CLUSTER */ \

#define HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(e_can, longitudinal) \
  {0x1A0, e_can, 32, .check_relay = (longitudinal)},  /* SCC_CONTROL */ \

#define HYUNDAI_CANFD_HKG_CLUSTER_TEST_TX_MSGS(e_can) \
  {0x161, e_can, 32, .check_relay = false},  /* CCNC_0x161 display test */ \

// *** Addresses checked in rx hook ***
// EV, ICE, HYBRID: ACCELERATOR (0x35), ACCELERATOR_BRAKE_ALT (0x100), ACCELERATOR_ALT (0x105)
#define HYUNDAI_CANFD_COMMON_RX_CHECKS(pt_bus)                                                                          \
  {.msg = {{0x35, (pt_bus), 32, 100U, .max_counter = 0xffU, .ignore_quality_flag = true},                  \
           {0x100, (pt_bus), 32, 100U, .max_counter = 0xffU, .ignore_quality_flag = true},                 \
           {0x105, (pt_bus), 32, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}}},               \
  {.msg = {{0x175, (pt_bus), 24, 50U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{0xa0, (pt_bus), 24, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{0xea, (pt_bus), 24, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

#define HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(pt_bus)                                                                                            \
  HYUNDAI_CANFD_COMMON_RX_CHECKS(pt_bus)                                                                                                       \
  {.msg = {{0x1cf, (pt_bus), 8, 50U, .ignore_checksum = true, .max_counter = 0xfU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

#define HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(pt_bus)                                                                                              \
  HYUNDAI_CANFD_COMMON_RX_CHECKS(pt_bus)                                                                                                         \
  {.msg = {{0x1aa, (pt_bus), 16, 50U, .ignore_checksum = true, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

// SCC_CONTROL (from ADAS unit or camera)
#define HYUNDAI_CANFD_SCC_ADDR_CHECK(scc_bus)                                                                            \
  {.msg = {{0x1a0, (scc_bus), 32, 50U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

static bool hyundai_canfd_alt_buttons = false;
static bool hyundai_canfd_lka_steer_msg_alt = false;
static bool hyundai_canfd_dynamic_torque = false;
static bool hyundai_canfd_low_speed_rt_active = false;
static bool hyundai_canfd_hkg_cluster_test = false;
static bool hyundai_canfd_hkg_cluster_test_parked = false;
static uint16_t hyundai_canfd_hkg_cluster_test_tx_count = 0U;

static unsigned int hyundai_canfd_get_lka_addr(void) {
  return hyundai_canfd_lka_steer_msg_alt ? 0x110U : 0x50U;
}

static uint8_t hyundai_canfd_get_counter(const CANPacket_t *msg) {
  uint8_t ret = 0;
  if (GET_LEN(msg) == 8U) {
    ret = msg->data[1] >> 4;
  } else {
    ret = msg->data[2];
  }
  return ret;
}

static uint32_t hyundai_canfd_get_checksum(const CANPacket_t *msg) {
  uint32_t chksum = msg->data[0] | (msg->data[1] << 8);
  return chksum;
}

static bool hyundai_canfd_hkg_lane_color_valid(uint8_t color) {
  return (color == 0U) || (color == 1U) || (color == 2U) || (color == 4U) || (color == 6U);
}

static bool hyundai_canfd_hkg_assist_icon_valid(uint8_t icon) {
  return (icon == 0U) || (icon == 1U) || (icon == 2U) || (icon == 3U) || (icon == 4U);
}

static bool hyundai_canfd_hkg_status_test_valid(const CANPacket_t *msg) {
  const uint8_t left_arrow = (msg->data[7] >> 1U) & 0x7U;
  const uint8_t right_arrow = (msg->data[7] >> 4U) & 0x7U;
  const uint8_t centerline = msg->data[8] & 0x3U;
  const uint8_t left_lane = msg->data[10] & 0xFU;
  const uint8_t left_lane_position = ((msg->data[10] >> 4U) & 0xFU) | ((msg->data[11] & 0x3U) << 4U);
  const uint8_t right_lane = (msg->data[11] >> 2U) & 0xFU;
  const uint8_t right_lane_position = ((msg->data[11] >> 6U) & 0x3U) | ((msg->data[12] & 0xFU) << 2U);
  const uint8_t lane_highlight = (msg->data[13] >> 1U) & 0xFU;
  const uint16_t lane_highlight_distance = ((msg->data[13] >> 5U) & 0x7U) | ((uint16_t)msg->data[14] << 3U);
  const uint8_t lane_left = msg->data[15] & 0x7U;
  const uint8_t lane_right = (msg->data[15] >> 3U) & 0x7U;
  const uint8_t lane_zoom = (msg->data[15] >> 6U) & 0x3U;
  const uint8_t hda_icon = (msg->data[26] >> 4U) & 0xFU;
  const uint8_t nav_icon = (msg->data[27] >> 4U) & 0xFU;
  const uint8_t lfa_icon = msg->data[28] & 0xFU;
  const uint8_t lca_left_icon = (msg->data[28] >> 4U) & 0xFU;
  const uint8_t lca_right_icon = msg->data[29] & 0xFU;

  // Block collision/AEB, BCA, alert, sound, set-speed, background, DAW, and
  // unknown/reserved fields. This test may exercise only benign display bits.
  bool valid = (msg->data[3] == 0U) && (msg->data[4] == 0U) && (msg->data[5] == 0U) && (msg->data[6] == 0U);
  valid &= (msg->data[7] & 0x81U) == 0U;
  valid &= (msg->data[8] & 0xFCU) == 0U;
  valid &= msg->data[9] == 0U;
  for (int i = 16; i <= 25; i++) {
    valid &= msg->data[i] == 0U;
  }
  valid &= (msg->data[26] & 0xFU) == 0U;
  valid &= (msg->data[27] & 0xFU) == 0U;
  valid &= (msg->data[29] & 0xF0U) == 0U;
  valid &= (msg->data[30] == 0U) && (msg->data[31] == 0U);

  valid &= (left_arrow <= 1U) && (right_arrow <= 1U) && (centerline <= 1U);
  valid &= hyundai_canfd_hkg_lane_color_valid(left_lane) && hyundai_canfd_hkg_lane_color_valid(right_lane);
  valid &= (left_lane_position <= 30U) && (right_lane_position <= 30U);
  // Keep the signed curvature field at its neutral raw value (physical 15).
  valid &= ((msg->data[12] & 0xF0U) == 0U) && ((msg->data[13] & 0x1U) == 0U);
  valid &= (lane_highlight == 0U) || (lane_highlight == 1U) || (lane_highlight == 2U) || (lane_highlight == 4U);
  valid &= lane_highlight_distance <= 1000U;
  valid &= (lane_left <= 1U) && (lane_right <= 1U) && (lane_zoom <= 1U);
  valid &= (hda_icon <= 3U) && hyundai_canfd_hkg_assist_icon_valid(nav_icon) && (nav_icon != 3U);
  valid &= (lfa_icon <= 3U) && hyundai_canfd_hkg_assist_icon_valid(lca_left_icon) && (lca_left_icon != 3U);
  valid &= hyundai_canfd_hkg_assist_icon_valid(lca_right_icon) && (lca_right_icon != 3U);
  return valid;
}

static void hyundai_canfd_rx_hook(const CANPacket_t *msg) {

  const unsigned pt_bus = hyundai_canfd_lka_steer_msg ? 1U : 0U;
  const unsigned int scc_bus = hyundai_camera_scc ? 2U : pt_bus;

  if (msg->bus == pt_bus) {
    // driver torque
    if (msg->addr == 0xeaU) {
      int torque_driver_new = ((msg->data[11] & 0x1fU) << 8U) | msg->data[10];
      torque_driver_new -= 4095;
      update_sample(&torque_driver, torque_driver_new);
    }

    // cruise buttons
    const unsigned int button_addr = hyundai_canfd_alt_buttons ? 0x1aaU : 0x1cfU;
    if (msg->addr == button_addr) {
      bool main_button = false;
      int cruise_button = 0;
      if (msg->addr == 0x1cfU) {
        cruise_button = msg->data[2] & 0x7U;
        main_button = GET_BIT(msg, 19U);
        mads_button_press = GET_BIT(msg, 23U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
      } else {
        cruise_button = (msg->data[4] >> 4) & 0x7U;
        main_button = GET_BIT(msg, 34U);
        mads_button_press = GET_BIT(msg, 39U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
      }
      hyundai_common_cruise_buttons_check(cruise_button, main_button);
    }

    // gas press, different for EV, hybrid, and ICE models
    if ((msg->addr == 0x35U) && hyundai_ev_gas_signal) {
      gas_pressed = msg->data[5] != 0U;
      hyundai_canfd_hkg_cluster_test_parked = (msg->data[24] & 0x7U) == 0U;
    } else if ((msg->addr == 0x105U) && hyundai_hybrid_gas_signal) {
      gas_pressed = GET_BIT(msg, 103U) || (msg->data[13] != 0U) || GET_BIT(msg, 112U);
    } else if ((msg->addr == 0x100U) && !hyundai_ev_gas_signal && !hyundai_hybrid_gas_signal) {
      gas_pressed = GET_BIT(msg, 176U);
    } else {
    }

    // brake press
    if (msg->addr == 0x175U) {
      brake_pressed = GET_BIT(msg, 81U);
    }

    // vehicle moving
    if (msg->addr == 0xa0U) {
      uint32_t fl = (GET_BYTES(msg, 8, 2)) & 0x3FFFU;
      uint32_t fr = (GET_BYTES(msg, 10, 2)) & 0x3FFFU;
      uint32_t rl = (GET_BYTES(msg, 12, 2)) & 0x3FFFU;
      uint32_t rr = (GET_BYTES(msg, 14, 2)) & 0x3FFFU;
      vehicle_moving = (fl > HYUNDAI_STANDSTILL_THRSLD) || (fr > HYUNDAI_STANDSTILL_THRSLD) ||
                       (rl > HYUNDAI_STANDSTILL_THRSLD) || (rr > HYUNDAI_STANDSTILL_THRSLD);

      // average of all 4 wheel speeds. Conversion: raw * 0.03125 / 3.6 = m/s
      UPDATE_VEHICLE_SPEED((fr + rr + rl + fl) / 4.0 * 0.03125 * KPH_TO_MS);
    }
  }

  if (msg->bus == scc_bus) {
    // cruise state
    if ((msg->addr == 0x1a0U) && !hyundai_longitudinal) {
      // 1=enabled, 2=driver override
      int cruise_status = ((msg->data[8] >> 4) & 0x7U);
      bool cruise_engaged = (cruise_status == 1) || (cruise_status == 2);
      hyundai_common_cruise_state_check(cruise_engaged);
      acc_main_on = GET_BIT(msg, 66U);
    }
  }

  hyundai_common_reset_acc_main_on_mismatches();
}

static bool hyundai_canfd_tx_hook(const CANPacket_t *msg) {
  const struct lookup_t HYUNDAI_CANFD_MAX_TORQUE_LOOKUP = {
    {11., 13., 17.},
    {384., 350., 270.},
  };
  const struct lookup_t HYUNDAI_CANFD_RATE_UP_LOOKUP = {
    {11., 13., 13.},
    {10., 2., 2.},
  };
  const struct lookup_t HYUNDAI_CANFD_RATE_DOWN_LOOKUP = {
    {11., 13., 13.},
    {10., 3., 3.},
  };
  // Cap the generic dynamic-limit tolerance at the nominal curve, including 270 at high speed.
  const int requested_max_torque = hyundai_canfd_dynamic_torque ?
    ROUND(safety_interpolate(HYUNDAI_CANFD_MAX_TORQUE_LOOKUP, vehicle_speed.min / VEHICLE_SPEED_FACTOR)) : 270;

  // If speed increases while torque is above the new curve, permit only a monotonic wind-down.
  const int desired_torque_last_abs = (desired_torque_last >= 0) ? desired_torque_last : -desired_torque_last;
  const bool torque_wind_down = hyundai_canfd_dynamic_torque && (desired_torque_last_abs > requested_max_torque);
  const float rate_speed = vehicle_speed.min / VEHICLE_SPEED_FACTOR;
  const int max_rate_up = hyundai_canfd_dynamic_torque ? ROUND(safety_interpolate(HYUNDAI_CANFD_RATE_UP_LOOKUP, rate_speed)) : 2;
  const int max_rate_down = hyundai_canfd_dynamic_torque ? ROUND(safety_interpolate(HYUNDAI_CANFD_RATE_DOWN_LOOKUP, rate_speed)) : 3;
  const int rt_torque_delta = desired_torque_last - rt_torque_last;
  const int rt_torque_delta_abs = (rt_torque_delta >= 0) ? rt_torque_delta : -rt_torque_delta;
  if (max_rate_up > 2) {
    hyundai_canfd_low_speed_rt_active = true;
  } else if (hyundai_canfd_low_speed_rt_active && ((rt_torque_delta_abs + max_rate_up) <= 112)) {
    hyundai_canfd_low_speed_rt_active = false;
  } else {
    // Retain the wider RT window until the stored reference catches up.
  }
  const int max_torque = torque_wind_down ? desired_torque_last_abs : requested_max_torque;
  const TorqueSteeringLimits HYUNDAI_CANFD_STEERING_LIMITS = {
    .max_torque = max_torque,
    .dynamic_max_torque = hyundai_canfd_dynamic_torque && !torque_wind_down,
    .max_torque_lookup = HYUNDAI_CANFD_MAX_TORQUE_LOOKUP,
    // 10 units/frame needs 270 units/250 ms because the RT reference updates after checking the 27th 100 Hz frame.
    // Keep it until the RT reference catches up, so crossing 13 m/s cannot reject an otherwise valid command.
    .max_rt_delta = hyundai_canfd_low_speed_rt_active ? 270 : 112,
    .max_rate_up = max_rate_up,
    .max_rate_down = max_rate_down,
    .driver_torque_allowance = 250,
    .driver_torque_multiplier = 2,
    .type = TorqueDriverLimited,

    // the EPS faults when the steering angle is above a certain threshold for too long. to prevent this,
    // we allow setting torque actuation bit to 0 while maintaining the requested torque value for two consecutive frames
    .min_valid_request_frames = 89,
    .max_invalid_request_frames = 2,
    .min_valid_request_rt_interval = 810000,  // 810ms; a ~10% buffer on cutting every 90 frames
    .has_steer_req_tolerance = true,
  };

  bool tx = true;

  if ((msg->addr == 0x161U) || (msg->addr == 0x162U)) {
    const uint16_t hkg_cluster_test_max_tx = 700U;
    const bool checksum_valid = hyundai_canfd_get_checksum(msg) == hyundai_common_canfd_compute_checksum(msg);
    const bool payload_valid = (msg->addr == 0x161U) && hyundai_canfd_hkg_status_test_valid(msg);
    const bool interlocks_valid = hyundai_canfd_hkg_cluster_test && hyundai_canfd_hkg_cluster_test_parked &&
                                  !vehicle_moving && !controls_allowed && !controls_allowed_lateral && !gas_pressed;
    if (!checksum_valid || !payload_valid || !interlocks_valid ||
        (hyundai_canfd_hkg_cluster_test_tx_count >= hkg_cluster_test_max_tx)) {
      tx = false;
    } else {
      hyundai_canfd_hkg_cluster_test_tx_count++;
    }
  }

  // steering
  const unsigned int steer_addr = (hyundai_canfd_lka_steer_msg && !hyundai_longitudinal) ? hyundai_canfd_get_lka_addr() : 0x12aU;
  if (msg->addr == steer_addr) {
    int desired_torque = (((msg->data[6] & 0xFU) << 7U) | (msg->data[5] >> 1U)) - 1024U;
    bool steer_req = GET_BIT(msg, 52U);
    const int desired_torque_abs = (desired_torque >= 0) ? desired_torque : -desired_torque;

    // Normal steer-request cuts may hold torque briefly; every requested frame above the new limit must decrease.
    if (torque_wind_down && (desired_torque_abs > requested_max_torque) &&
        ((desired_torque_abs > desired_torque_last_abs) ||
         ((desired_torque_abs == desired_torque_last_abs) && steer_req))) {
      tx = false;
    }

    if (steer_torque_cmd_checks(desired_torque, steer_req, HYUNDAI_CANFD_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // cruise buttons check
  if (msg->addr == 0x1cfU) {
    int button = msg->data[2] & 0x7U;
    bool is_cancel = (button == HYUNDAI_BTN_CANCEL);
    bool is_resume = (button == HYUNDAI_BTN_RESUME);
    bool is_set = (button == HYUNDAI_BTN_SET);

    bool allowed = (is_cancel && cruise_engaged_prev) || ((is_resume || is_set) && controls_allowed);
    if (!allowed) {
      tx = false;
    }
  }

  // UDS: only tester present ("\x02\x3E\x80\x00\x00\x00\x00\x00") allowed on diagnostics address
  if (((msg->addr == 0x730U) && hyundai_canfd_lka_steer_msg) || ((msg->addr == 0x7D0U) && !hyundai_camera_scc)) {
    if ((GET_BYTES(msg, 0, 4) != 0x00803E02U) || (GET_BYTES(msg, 4, 4) != 0x0U)) {
      tx = false;
    }
  }

  // ACCEL: safety check
  if (msg->addr == 0x1a0U) {
    int desired_accel_raw = (((msg->data[17] & 0x7U) << 8) | msg->data[16]) - 1023U;
    int desired_accel_val = ((msg->data[18] << 4) | (msg->data[17] >> 4)) - 1023U;

    bool violation = false;

    if (hyundai_longitudinal) {
      violation |= longitudinal_accel_checks(desired_accel_raw, HYUNDAI_LONG_LIMITS);
      violation |= longitudinal_accel_checks(desired_accel_val, HYUNDAI_LONG_LIMITS);
    } else {
      // only used to cancel on here
      const int acc_mode = (msg->data[8] >> 4) & 0x7U;
      if (acc_mode != 4) {
        violation = true;
      }

      if ((desired_accel_raw != 0) || (desired_accel_val != 0)) {
        violation = true;
      }
    }

    if (violation) {
      tx = false;
    }

    acc_main_on_tx = GET_BIT(msg, 66U);
    hyundai_common_acc_main_on_sync();
  }

  return tx;
}

static safety_config hyundai_canfd_init(uint16_t param) {
  const uint16_t HYUNDAI_PARAM_CANFD_LKA_STEER_MSG_ALT = 128;
  const uint16_t HYUNDAI_PARAM_CANFD_ALT_BUTTONS = 32;
  const uint16_t HYUNDAI_PARAM_CANFD_DYNAMIC_TORQUE = 1024;
  const uint16_t HYUNDAI_PARAM_CANFD_HKG_CLUSTER_TEST = 4096;

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_COMMON_TX_MSGS(0, 1)
  };

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_HKG_CLUSTER_TEST_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_COMMON_TX_MSGS(0, 1)
    HYUNDAI_CANFD_HKG_CLUSTER_TEST_TX_MSGS(1)
  };

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_ALT_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_ALT_COMMON_TX_MSGS(0, 1)
  };

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_ALT_HKG_CLUSTER_TEST_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_ALT_COMMON_TX_MSGS(0, 1)
    HYUNDAI_CANFD_HKG_CLUSTER_TEST_TX_MSGS(1)
  };

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_LONG_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_COMMON_TX_MSGS(0, 1)
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(1)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(1, true)
    {0x51,  0, 32, .check_relay = false},  // ADRV_0x51
    {0x730, 1,  8, .check_relay = false},  // tester present for ADAS ECU disable
    {0x160, 1, 16, .check_relay = false},  // ADRV_0x160
    {0x1EA, 1, 32, .check_relay = false},  // ADRV_0x1ea
    {0x200, 1,  8, .check_relay = false},  // ADRV_0x200
    {0x345, 1,  8, .check_relay = false},  // ADRV_0x345
    {0x1DA, 1, 32, .check_relay = false},  // ADRV_0x1da
  };

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_LONG_HKG_CLUSTER_TEST_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_COMMON_TX_MSGS(0, 1)
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(1)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(1, true)
    HYUNDAI_CANFD_HKG_CLUSTER_TEST_TX_MSGS(1)
    {0x51,  0, 32, .check_relay = false},  // ADRV_0x51
    {0x730, 1,  8, .check_relay = false},  // tester present for ADAS ECU disable
    {0x160, 1, 16, .check_relay = false},  // ADRV_0x160
    {0x1EA, 1, 32, .check_relay = false},  // ADRV_0x1ea
    {0x200, 1,  8, .check_relay = false},  // ADRV_0x200
    {0x345, 1,  8, .check_relay = false},  // ADRV_0x345
    {0x1DA, 1, 32, .check_relay = false},  // ADRV_0x1da
  };

  static const CanMsg HYUNDAI_CANFD_LFA_STEERING_TX_MSGS[] = {
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2)
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, false)
  };

  // ADRV_0x160 is checked for radar liveness
  static const CanMsg HYUNDAI_CANFD_LFA_STEERING_LONG_TX_MSGS[] = {
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2)
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, true)
    {0x160, 0, 16, .check_relay = true}, // ADRV_0x160
    {0x7D0, 0, 8, .check_relay = false},  // tester present for radar ECU disable
  };

  // ADRV_0x160 is checked for relay malfunction
#define HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(longitudinal) \
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2) \
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0) \
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, (longitudinal)) \
    {0x160, 0, 16, .check_relay = (longitudinal)}, /* ADRV_0x160 */ \

  hyundai_common_init(param);

  gen_crc_lookup_table_16(0x1021, hyundai_canfd_crc_lut);
  hyundai_canfd_alt_buttons = GET_FLAG(param, HYUNDAI_PARAM_CANFD_ALT_BUTTONS);
  hyundai_canfd_lka_steer_msg_alt = GET_FLAG(param, HYUNDAI_PARAM_CANFD_LKA_STEER_MSG_ALT);
  hyundai_canfd_dynamic_torque = GET_FLAG(param, HYUNDAI_PARAM_CANFD_DYNAMIC_TORQUE);
  hyundai_canfd_low_speed_rt_active = false;
  hyundai_canfd_hkg_cluster_test = GET_FLAG(param, HYUNDAI_PARAM_CANFD_HKG_CLUSTER_TEST);
  hyundai_canfd_hkg_cluster_test_parked = false;
  hyundai_canfd_hkg_cluster_test_tx_count = 0U;

  safety_config ret;
  if (hyundai_longitudinal) {
    if (hyundai_canfd_lka_steer_msg) {
      static RxCheck hyundai_canfd_lka_steer_msg_long_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(1)
      };

      if (hyundai_canfd_hkg_cluster_test) {
        ret = BUILD_SAFETY_CFG(hyundai_canfd_lka_steer_msg_long_rx_checks, HYUNDAI_CANFD_LKA_STEER_MSG_LONG_HKG_CLUSTER_TEST_TX_MSGS);
      } else {
        ret = BUILD_SAFETY_CFG(hyundai_canfd_lka_steer_msg_long_rx_checks, HYUNDAI_CANFD_LKA_STEER_MSG_LONG_TX_MSGS);
      }

    } else {
      // Longitudinal checks for LFA steering
      static RxCheck hyundai_canfd_long_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
      };

      static RxCheck hyundai_canfd_alt_buttons_long_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(0)
      };

      static CanMsg hyundai_canfd_lfa_steering_camera_scc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(true)
      };

      if (hyundai_canfd_alt_buttons) {
        SET_RX_CHECKS(hyundai_canfd_alt_buttons_long_rx_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_canfd_long_rx_checks, ret);
      }

      if (hyundai_camera_scc) {
        SET_TX_MSGS(hyundai_canfd_lfa_steering_camera_scc_tx_msgs, ret);
      } else {
        SET_TX_MSGS(HYUNDAI_CANFD_LFA_STEERING_LONG_TX_MSGS, ret);
      }
    }

  } else {
    if (hyundai_canfd_lka_steer_msg) {
      // *** LKA steering checks ***
      // E-CAN is on bus 1, SCC messages are sent on cars with ADRV ECU.
      // Does not use the alt buttons message
      static RxCheck hyundai_canfd_lka_steer_msg_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(1)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(1)
      };

      SET_RX_CHECKS(hyundai_canfd_lka_steer_msg_rx_checks, ret);
      if (hyundai_canfd_lka_steer_msg_alt) {
        if (hyundai_canfd_hkg_cluster_test) {
          SET_TX_MSGS(HYUNDAI_CANFD_LKA_STEER_MSG_ALT_HKG_CLUSTER_TEST_TX_MSGS, ret);
        } else {
          SET_TX_MSGS(HYUNDAI_CANFD_LKA_STEER_MSG_ALT_TX_MSGS, ret);
        }
      } else {
        if (hyundai_canfd_hkg_cluster_test) {
          SET_TX_MSGS(HYUNDAI_CANFD_LKA_STEER_MSG_HKG_CLUSTER_TEST_TX_MSGS, ret);
        } else {
          SET_TX_MSGS(HYUNDAI_CANFD_LKA_STEER_MSG_TX_MSGS, ret);
        }
      }

    } else if (!hyundai_camera_scc) {
      // Radar sends SCC messages on these cars instead of camera
      static RxCheck hyundai_canfd_radar_scc_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(0)
      };

      static RxCheck hyundai_canfd_alt_buttons_radar_scc_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(0)
      };

      SET_TX_MSGS(HYUNDAI_CANFD_LFA_STEERING_TX_MSGS, ret);

      if (hyundai_canfd_alt_buttons) {
        SET_RX_CHECKS(hyundai_canfd_alt_buttons_radar_scc_rx_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_canfd_radar_scc_rx_checks, ret);
      }

    } else {
      // *** LFA steering checks ***
      // Camera sends SCC messages on LFA steering cars.
      // Both button messages exist on some platforms, so we ensure we track the correct one using flag
      static RxCheck hyundai_canfd_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
      };

      static RxCheck hyundai_canfd_alt_buttons_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
      };

      static CanMsg hyundai_canfd_lfa_steering_camera_scc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(false)
      };

      SET_TX_MSGS(hyundai_canfd_lfa_steering_camera_scc_tx_msgs, ret);

      if (hyundai_canfd_alt_buttons) {
        SET_RX_CHECKS(hyundai_canfd_alt_buttons_rx_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_canfd_rx_checks, ret);
      }
    }
  }

  return ret;
}

const safety_hooks hyundai_canfd_hooks = {
  .init = hyundai_canfd_init,
  .rx = hyundai_canfd_rx_hook,
  .tx = hyundai_canfd_tx_hook,
  .get_counter = hyundai_canfd_get_counter,
  .get_checksum = hyundai_canfd_get_checksum,
  .compute_checksum = hyundai_common_canfd_compute_checksum,
};
