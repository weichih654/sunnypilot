#pragma once

#include "opendbc/safety/declarations.h"

// CAN msgs we care about
#define MAZDA_LKAS          0x243U
#define MAZDA_LKAS_HUD      0x440U
#define MAZDA_CRZ_CTRL      0x21cU
#define MAZDA_CRZ_BTNS      0x09dU
#define MAZDA_STEER_TORQUE  0x240U
#define MAZDA_ENGINE_DATA   0x202U
#define MAZDA_PEDALS        0x165U

// torque interceptor (sunnypilot)
#define MAZDA_TI_LKAS       0x249U  // tx: torque cmd to the interceptor (CAM_LKAS2)
#define MAZDA_TI_FEEDBACK   0x24AU  // rx: driver torque from the interceptor (TI_FEEDBACK)

// CAN bus numbers
#define MAZDA_MAIN 0
#define MAZDA_AUX  1
#define MAZDA_CAM  2

// sunnypilot safety param flags
#define MAZDA_PARAM_SP_TORQUE_INTERCEPTOR 1U

static bool mazda_torque_interceptor = false;

// dedicated rate-limit state for the torque interceptor channel:
// steer_torque_cmd_checks() keeps single-channel global state which is
// already used by the stock MAZDA_LKAS channel
static int mazda_ti_desired_torque_last = 0;
static int mazda_ti_rt_torque_last = 0;
static uint32_t mazda_ti_ts_torque_check_last = 0;

// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static void mazda_rx_hook(const CANPacket_t *msg) {
  if ((int)msg->bus == MAZDA_MAIN) {
    if (msg->addr == MAZDA_ENGINE_DATA) {
      // sample speed: scale by 0.01 to get kph
      int speed = (msg->data[2] << 8) | msg->data[3];
      vehicle_moving = speed > 10; // moving when speed > 0.1 kph
    }

    if ((msg->addr == MAZDA_STEER_TORQUE) && !mazda_torque_interceptor) {
      int torque_driver_new = msg->data[0] - 127U;
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if (msg->addr == MAZDA_CRZ_CTRL) {
      bool cruise_engaged = msg->data[0] & 0x8U;
      pcm_cruise_check(cruise_engaged);
      acc_main_on = GET_BIT(msg, 17U);
    }

    if (msg->addr == MAZDA_ENGINE_DATA) {
      gas_pressed = (msg->data[4] || (msg->data[5] & 0xF0U));
    }

    if (msg->addr == MAZDA_PEDALS) {
      brake_pressed = (msg->data[0] & 0x10U);
    }
  }

  if (((int)msg->bus == MAZDA_AUX) && mazda_torque_interceptor) {
    if (msg->addr == MAZDA_TI_FEEDBACK) {
      int torque_driver_new = msg->data[0] - 127U;
      update_sample(&torque_driver, torque_driver_new);
    }
  }
}

static bool mazda_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits MAZDA_STEERING_LIMITS = {
    .max_torque = 800,
    .max_rate_up = 10,
    .max_rate_down = 25,
    .max_rt_delta = 300,
    .driver_torque_multiplier = 1,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  bool tx = true;
  // Check if msg is sent on the main BUS
  if (msg->bus == (unsigned char)MAZDA_MAIN) {
    // steer cmd checks
    if (msg->addr == MAZDA_LKAS) {
      int desired_torque = (((msg->data[0] & 0x0FU) << 8) | msg->data[1]) - 2048U;

      if (steer_torque_cmd_checks(desired_torque, -1, MAZDA_STEERING_LIMITS)) {
        tx = false;
      }
    }

    // cruise buttons check
    if (msg->addr == MAZDA_CRZ_BTNS) {
      // allow resume spamming while controls allowed, but
      // only allow cancel while controls not allowed
      bool cancel_cmd = (msg->data[0] == 0x1U);
      if (!controls_allowed && !cancel_cmd) {
        tx = false;
      }
    }
  }

  // torque interceptor steer cmd checks (bus 1), mirroring steer_torque_cmd_checks()
  // with dedicated per-channel state. Zero-torque messages while controls are not
  // allowed must always pass: they are the interceptor keepalive.
  if (mazda_torque_interceptor && (msg->bus == (unsigned char)MAZDA_AUX) && (msg->addr == MAZDA_TI_LKAS)) {
    const int MAZDA_TI_MAX_TORQUE = 600;
    const int MAZDA_TI_MAX_RATE_UP = 6;
    const int MAZDA_TI_MAX_RATE_DOWN = 15;
    const int MAZDA_TI_DRIVER_TORQUE_ALLOWANCE = 15;
    const int MAZDA_TI_DRIVER_TORQUE_FACTOR = 40;
    const int MAZDA_TI_MAX_RT_DELTA = 225;  // 6 * 100Hz * 250000/1000000 * 1.5

    int desired_torque = (((msg->data[0] & 0x0FU) << 8) | msg->data[1]) - 2048U;
    bool violation = false;
    uint32_t ts = microsecond_timer_get();

    if (controls_allowed || controls_allowed_lateral) {
      // global torque limit check
      violation |= safety_max_limit_check(desired_torque, MAZDA_TI_MAX_TORQUE, -MAZDA_TI_MAX_TORQUE);

      // torque rate limit check against the interceptor driver torque sensor
      violation |= driver_limit_check(desired_torque, mazda_ti_desired_torque_last, &torque_driver,
                                      MAZDA_TI_MAX_TORQUE, MAZDA_TI_MAX_RATE_UP, MAZDA_TI_MAX_RATE_DOWN,
                                      MAZDA_TI_DRIVER_TORQUE_ALLOWANCE, MAZDA_TI_DRIVER_TORQUE_FACTOR);
      mazda_ti_desired_torque_last = desired_torque;

      // torque real time rate limit check
      violation |= rt_torque_rate_limit_check(desired_torque, mazda_ti_rt_torque_last, MAZDA_TI_MAX_RT_DELTA);

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = safety_get_ts_elapsed(ts, mazda_ti_ts_torque_check_last);
      if (ts_elapsed > MAX_RT_INTERVAL) {
        mazda_ti_rt_torque_last = desired_torque;
        mazda_ti_ts_torque_check_last = ts;
      }
    }

    // no torque if controls is not allowed
    if (!(controls_allowed || controls_allowed_lateral) && (desired_torque != 0)) {
      violation = true;
    }

    // reset to 0 if either controls is not allowed or there's a violation, so a
    // commanded instant drop to zero only blocks a single keepalive frame
    if (violation || !(controls_allowed || controls_allowed_lateral)) {
      mazda_ti_desired_torque_last = 0;
      mazda_ti_rt_torque_last = 0;
      mazda_ti_ts_torque_check_last = ts;
    }

    if (violation) {
      tx = false;
    }
  }

  return tx;
}

static safety_config mazda_init(uint16_t param) {
  static const CanMsg MAZDA_TX_MSGS[] = {{MAZDA_LKAS, 0, 8, .check_relay = true}, {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false}, {MAZDA_LKAS_HUD, 0, 8, .check_relay = true}};

  static const CanMsg MAZDA_TI_TX_MSGS[] = {{MAZDA_LKAS, 0, 8, .check_relay = true}, {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false}, {MAZDA_LKAS_HUD, 0, 8, .check_relay = true},
                                            {MAZDA_TI_LKAS, 1, 8, .check_relay = false}};

  mazda_torque_interceptor = GET_FLAG(current_safety_param_sp, MAZDA_PARAM_SP_TORQUE_INTERCEPTOR);
  mazda_ti_desired_torque_last = 0;
  mazda_ti_rt_torque_last = 0;
  mazda_ti_ts_torque_check_last = 0;
  SAFETY_UNUSED(param);

  safety_config ret;
  if (mazda_torque_interceptor) {
    static RxCheck mazda_ti_rx_checks[] = {
      {.msg = {{MAZDA_CRZ_CTRL,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_TI_FEEDBACK,  1, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    };
    ret = BUILD_SAFETY_CFG(mazda_ti_rx_checks, MAZDA_TI_TX_MSGS);
  } else {
    static RxCheck mazda_rx_checks[] = {
      {.msg = {{MAZDA_CRZ_CTRL,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    };
    ret = BUILD_SAFETY_CFG(mazda_rx_checks, MAZDA_TX_MSGS);
  }
  return ret;
}

const safety_hooks mazda_hooks = {
  .init = mazda_init,
  .rx = mazda_rx_hook,
  .tx = mazda_tx_hook,
};
