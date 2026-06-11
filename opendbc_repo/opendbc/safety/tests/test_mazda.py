#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.sunnypilot.car.mazda.torque_interceptor import TI_KEY
from opendbc.sunnypilot.car.mazda.values import MazdaSafetyFlagsSP, TICarControllerParams


class TestMazdaSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):

  TX_MSGS = [[0x243, 0], [0x09d, 0], [0x440, 0]]
  STANDSTILL_THRESHOLD = .1
  RELAY_MALFUNCTION_ADDRS = {0: (0x243, 0x440)}
  FWD_BLACKLISTED_ADDRS = {2: [0x243, 0x440]}

  MAX_RATE_UP = 10
  MAX_RATE_DOWN = 25
  MAX_TORQUE_LOOKUP = [0], [800]

  MAX_RT_DELTA = 300

  DRIVER_TORQUE_ALLOWANCE = 15
  DRIVER_TORQUE_FACTOR = 1

  # Mazda actually does not set any bit when requesting torque
  NO_STEER_REQ_BIT = True

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, 0)
    self.safety.init_tests()

  def _torque_meas_msg(self, torque):
    values = {"STEER_TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"STEER_TORQUE_SENSOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_REQUEST": torque}
    return self.packer.make_can_msg_safety("CAM_LKAS", 0, values)

  def _speed_msg(self, speed):
    values = {"SPEED": speed}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_ON": brake}
    return self.packer.make_can_msg_safety("PEDALS", 0, values)

  def _user_gas_msg(self, gas):
    values = {"PEDAL_GAS": gas}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRZ_ACTIVE": enable}
    return self.packer.make_can_msg_safety("CRZ_CTRL", 0, values)

  def _button_msg(self, resume=False, cancel=False):
    values = {
      "CAN_OFF": cancel,
      "CAN_OFF_INV": (cancel + 1) % 2,
      "RES": resume,
      "RES_INV": (resume + 1) % 2,
    }
    return self.packer.make_can_msg_safety("CRZ_BTNS", 0, values)

  def test_buttons(self):
    # only cancel allows while controls not allowed
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertFalse(self._tx(self._button_msg(resume=True)))

    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertTrue(self._tx(self._button_msg(resume=True)))


class TestMazdaTorqueInterceptorSafety(TestMazdaSafety):
  """Mazda GEN1 with a torque interceptor on bus 1 (sunnypilot)"""

  TX_MSGS = [[0x243, 0], [0x09d, 0], [0x440, 0], [0x249, 1]]

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(MazdaSafetyFlagsSP.TORQUE_INTERCEPTOR)
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, 0)
    self.safety.init_tests()
    # init_tests() clears the SP param; set it again so re-inits keep TI mode
    self.safety.set_current_safety_param_sp(MazdaSafetyFlagsSP.TORQUE_INTERCEPTOR)

  def tearDown(self):
    # do not leak the SP param into other tests
    self.safety.set_current_safety_param_sp(0)

  def _torque_driver_msg(self, torque):
    # with the interceptor installed, driver torque comes from TI_FEEDBACK on bus 1
    values = {"TI_TORQUE_SENSOR": torque}
    return self.packer.make_can_msg_safety("TI_FEEDBACK", 1, values)

  def _ti_torque_cmd_msg(self, torque):
    values = {"LKAS_REQUEST": torque, "CHKSUM": torque, "KEY": TI_KEY}
    return self.packer.make_can_msg_safety("CAM_LKAS2", 1, values)

  def test_ti_keepalive_always_allowed(self):
    # zero torque keepalive must always be allowed, engaged or not
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._ti_torque_cmd_msg(0)))
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._ti_torque_cmd_msg(0)))

  def test_ti_torque_blocked_when_controls_not_allowed(self):
    self.safety.set_controls_allowed(0)
    for torque in (-100, -1, 1, 100):
      self.assertFalse(self._tx(self._ti_torque_cmd_msg(torque)))

  def test_ti_violation_resets_baseline(self):
    self.safety.set_controls_allowed(1)
    self.safety.set_mazda_ti_torque_last(0)
    # oversized command is blocked and resets the rate-limit baseline to zero
    self.assertFalse(self._tx(self._ti_torque_cmd_msg(601)))
    # the blocked value must not become the new baseline: a jump within the
    # rt window but far above the rate limit from zero is still blocked
    self.assertFalse(self._tx(self._ti_torque_cmd_msg(220)))
    # ramping from zero passes
    self.assertTrue(self._tx(self._ti_torque_cmd_msg(6)))


class TestMazdaTISteeringSafety(TestMazdaTorqueInterceptorSafety):
  """Runs the full driver-torque steering test suite against the interceptor
  channel (CAM_LKAS2 on bus 1) instead of the stock LKAS channel."""

  MAX_RATE_UP = TICarControllerParams.STEER_DELTA_UP
  MAX_RATE_DOWN = TICarControllerParams.STEER_DELTA_DOWN
  MAX_TORQUE_LOOKUP = [0], [TICarControllerParams.STEER_MAX]

  MAX_RT_DELTA = 225

  DRIVER_TORQUE_ALLOWANCE = TICarControllerParams.STEER_DRIVER_ALLOWANCE
  DRIVER_TORQUE_FACTOR = TICarControllerParams.STEER_DRIVER_MULTIPLIER

  def _torque_cmd_msg(self, torque, steer_req=1):
    return self._ti_torque_cmd_msg(torque)

  def _set_prev_torque(self, t):
    super()._set_prev_torque(t)
    self.safety.set_mazda_ti_torque_last(t)


if __name__ == "__main__":
  unittest.main()
