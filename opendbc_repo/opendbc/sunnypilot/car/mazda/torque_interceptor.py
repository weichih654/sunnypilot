"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Support for the Mazda GEN1 torque interceptor: hardware spliced into the EPS
torque sensor line, commanded over CAN bus 1, that bypasses the stock EPS
low-speed LKAS lockout and enables full-speed-range steering.
"""

from opendbc.car import structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP, TICarControllerParams, TI_LKAS_LIMITS, TI_STATE

TI_KEY = 3294744160


def create_ti_steering_control(packer, apply_torque):
  # The torque interceptor only transmits TI_FEEDBACK after it sees this signature
  # key, so this message must be sent every frame (even with zero torque) to keep
  # the interceptor alive and detectable.
  values = {
    "LKAS_REQUEST": apply_torque,
    "CHKSUM": apply_torque,
    "KEY": TI_KEY,
  }
  return packer.make_can_msg("CAM_LKAS2", 1, values)


_FAILURE_DEBOUNCE_FRAMES = 100  # 1s at 100Hz


class TorqueInterceptorCarState:
  def __init__(self, CP_SP: structs.CarParamsSP):
    self.enabled = bool(CP_SP.flags & MazdaFlagsSP.TORQUE_INTERCEPTOR)
    self.version = 1
    self.state = TI_STATE.RUN
    self.violation = 0
    self.error = 0
    self.ramp_down = False
    self.lkas_allowed = False
    self.seen_run = False
    self.failure_frames = 0
    self.fault = False

  def update(self, ret: structs.CarState, cp_body) -> None:
    """Reads TI_FEEDBACK and replaces the driver torque source with the interceptor sensor."""
    vl = cp_body.vl["TI_FEEDBACK"]

    ret.steeringTorque = vl["TI_TORQUE_SENSOR"]
    ret.steeringPressed = abs(ret.steeringTorque) > TI_LKAS_LIMITS.STEER_THRESHOLD

    self.version = vl["VERSION_NUMBER"]
    self.state = vl["STATE"]  # DISCOVER = 0, OFF = 1, DRIVER_OVER = 2, RUN = 3
    self.violation = vl["VIOL"]  # 0 = no violation
    self.error = vl["ERROR"]  # 0 = no error
    if self.version > 1:
      self.ramp_down = vl["RAMP_DOWN"] == 1

    self.lkas_allowed = not self.ramp_down and self.state == TI_STATE.RUN and self.error == 0

    # fault when the interceptor has not come up yet (no-entry at startup), or has
    # been failed for over a second while driving. DRIVER_OVER and ramp_down are
    # expected transients and only pause the torque output.
    self.seen_run = self.seen_run or self.state == TI_STATE.RUN
    failure = self.error != 0 or self.state in (TI_STATE.DISCOVER, TI_STATE.OFF)
    self.failure_frames = self.failure_frames + 1 if failure else 0
    self.fault = not self.seen_run or self.failure_frames > _FAILURE_DEBOUNCE_FRAMES


class TorqueInterceptorCarController:
  def __init__(self, CP_SP: structs.CarParamsSP):
    self.enabled = bool(CP_SP.flags & MazdaFlagsSP.TORQUE_INTERCEPTOR)
    self.apply_torque_last = 0

  def update(self, CC: structs.CarControl, CS, packer) -> list:
    if not self.enabled:
      return []

    apply_torque = 0
    if CC.latActive and CS.ti.lkas_allowed:
      new_torque = int(round(CC.actuators.torque * TICarControllerParams.STEER_MAX))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, TICarControllerParams)
    self.apply_torque_last = apply_torque

    # must be sent every frame as a keepalive, even with zero torque
    return [create_ti_steering_control(packer, apply_torque)]
