#pragma once

// FormosaPilot RX-paced display-only overlay, based on the carrot-wip design.
// The original OEM packet remains the source of counters, geometry, faults,
// sounds and unknown bits. No control-message FIFO or actuation policy changes.
#define HYUNDAI_FORMOSA_CLUSTER_PARAM 16384U
#define HYUNDAI_CLUSTER_MAX_AGE_US 150000U

typedef struct {
  uint32_t addr;
  uint8_t len;
  uint8_t data[32];
  uint32_t updated;
  bool valid;
} HyundaiClusterFrame;

static bool hyundai_formosa_cluster = false;
static HyundaiClusterFrame hyundai_cluster_frames[] = {
  {.addr=0x161U, .len=32U}, {.addr=0x162U, .len=32U}, {.addr=0x1E0U, .len=16U},
  {.addr=0x1EAU, .len=32U}, {.addr=0x200U, .len=8U},
};
static CanMsg hyundai_cluster_tx_msgs[64];

static HyundaiClusterFrame *hyundai_cluster_find(uint32_t addr) {
  HyundaiClusterFrame *result = NULL;
  for (unsigned int i=0U; i<5U; i++) {
    if (hyundai_cluster_frames[i].addr == addr) {
      result = &hyundai_cluster_frames[i];
    }
  }
  return result;
}

static uint16_t hyundai_cluster_checksum(const CANPacket_t *msg) {
  uint16_t crc = (uint16_t)hyundai_common_canfd_compute_checksum(msg);
  if (GET_LEN(msg) == 8U) {
    crc ^= 0x5F29U;
  } else if (GET_LEN(msg) == 16U) {
    crc ^= 0x041DU;
  } else {
  }
  return crc;
}

static bool hyundai_cluster_valid(const CANPacket_t *msg, const HyundaiClusterFrame *frame) {
  return (msg->extended == 0U) && (GET_LEN(msg) == frame->len) &&
         (hyundai_canfd_get_checksum(msg) == hyundai_cluster_checksum(msg));
}

static uint8_t hyundai_cluster_mask(uint32_t addr, unsigned int byte) {
  uint8_t mask = 0U;
  if (addr == 0x161U) {
    switch (byte) {
      case 7U: mask=0x7EU; break;  // lane-change arrows
      case 8U: mask=0x03U; break;  // centerline
      case 10U: mask=0x0FU; break; // left lane color
      case 11U: mask=0x3CU; break; // right lane color
      case 22U: mask=0xE0U; break; // set-speed HUD state
      case 24U: mask=0xF0U; break; // distance bars
      case 25U: mask=0xFFU; break; // set speed
      case 26U: mask=0x0FU; break; // set-speed state
      case 28U: mask=0x0FU; break; // LFA icon
      default: break;
    }
  } else if (addr == 0x162U) {
    switch (byte) {
      case 8U: case 11U: case 14U: case 17U: case 24U: mask=0x1FU; break;
      case 20U: mask=0xF8U; break;
      default: break;
    }
  } else if (addr == 0x1E0U) {
    switch (byte) {
      case 3U: case 5U: mask=0x80U; break;
      case 6U: mask=0x01U; break;
      default: break;
    }
  } else {
    // 0x1EA/0x200 fields have not been verified in this branch's DBC.
    // Follow their original reception unchanged instead of inventing geometry.
  }
  return mask;
}

static bool hyundai_cluster_consume(const CANPacket_t *msg) {
  HyundaiClusterFrame *frame = hyundai_cluster_find(msg->addr);
  bool consumed = false;
  if (hyundai_formosa_cluster && (msg->bus == 0U) && (frame != NULL) && hyundai_cluster_valid(msg, frame)) {
    for (unsigned int i=0U; i<frame->len; i++) {
      frame->data[i] = msg->data[i];
    }
    frame->updated = microsecond_timer_get();
    frame->valid = true;
    consumed = true;
  }
  return consumed;
}

static void hyundai_cluster_forward(CANPacket_t *msg) {
  HyundaiClusterFrame *frame = hyundai_cluster_find(msg->addr);
  if (hyundai_formosa_cluster && (msg->bus == 2U) && (frame != NULL) && frame->valid) {
    if (((microsecond_timer_get()-frame->updated) >= HYUNDAI_CLUSTER_MAX_AGE_US) || !hyundai_cluster_valid(msg, frame)) {
      frame->valid = false;
    } else {
      for (unsigned int i=3U; i<frame->len; i++) {
        const uint8_t mask = hyundai_cluster_mask(msg->addr, i);
        msg->data[i] = (msg->data[i] & (uint8_t)(~mask)) | (frame->data[i] & mask);
      }
      const uint16_t checksum = hyundai_cluster_checksum(msg);
      msg->data[0] = checksum & 0xFFU;
      msg->data[1] = checksum >> 8U;
    }
  }
}

static safety_config hyundai_cluster_config(safety_config cfg, uint16_t param) {
  hyundai_formosa_cluster = GET_FLAG(param, HYUNDAI_FORMOSA_CLUSTER_PARAM) && hyundai_camera_scc && !hyundai_canfd_lka_steer_msg;
  for (unsigned int i=0U; i<5U; i++) {
    hyundai_cluster_frames[i].valid = false;
  }
  if (hyundai_formosa_cluster) {
    if (cfg.tx_msgs_len > 59) {
      hyundai_formosa_cluster = false;
    } else {
      int count = cfg.tx_msgs_len;
      for (int i=0; i<count; i++) {
        hyundai_cluster_tx_msgs[i] = cfg.tx_msgs[i];
        if ((cfg.tx_msgs[i].bus == 0U) && (hyundai_cluster_find(cfg.tx_msgs[i].addr) != NULL)) {
          hyundai_cluster_tx_msgs[i].disable_static_blocking = true;
        }
      }
      for (unsigned int j=0U; j<5U; j++) {
        bool found = false;
        for (int i=0; i<count; i++) {
          found |= (hyundai_cluster_tx_msgs[i].addr == (int)hyundai_cluster_frames[j].addr) && (hyundai_cluster_tx_msgs[i].bus == 0U);
        }
        if (!found) {
          hyundai_cluster_tx_msgs[count] = (CanMsg){.addr=hyundai_cluster_frames[j].addr, .bus=0U,
            .len=hyundai_cluster_frames[j].len, .check_relay=true, .disable_static_blocking=true};
          count += 1;
        }
      }
      cfg.tx_msgs = hyundai_cluster_tx_msgs;
      cfg.tx_msgs_len = count;
    }
  }
  return cfg;
}
