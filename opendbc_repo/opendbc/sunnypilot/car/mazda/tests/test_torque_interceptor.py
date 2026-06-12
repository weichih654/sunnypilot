"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR
from opendbc.sunnypilot.car.interfaces import _initialize_mazda
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP, MazdaSafetyFlagsSP, TICarControllerParams

TI_LKAS = 0x249


def make_car_interface(ti=True):
  CP = CarInterface.get_non_essential_params(CAR.MAZDA_CX5)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.MAZDA_CX5)
  if ti:
    CP_SP.flags |= MazdaFlagsSP.TORQUE_INTERCEPTOR.value
  return CarInterface(CP, CP_SP)


def feed_ti_feedback(ci, packer, torque=0, state=3, version=2, viol=0, error=0, ramp_down=0, t=0):
  msg = packer.make_can_msg("TI_FEEDBACK", 1, {
    "TI_TORQUE_SENSOR": torque, "CHKSUM": torque, "VERSION_NUMBER": version,
    "STATE": state, "VIOL": viol, "ERROR": error, "RAMP_DOWN": ramp_down,
  })
  ci.can_parsers[Bus.body].update([t, [msg]])


def make_car_control(lat_active):
  builder = structs.CarControl()
  builder.latActive = lat_active
  builder.actuators.torque = 1.0
  return builder.as_reader()


def ti_torque_from_msg(msg):
  dat = msg[1]
  return (((dat[0] & 0x0F) << 8) | dat[1]) - 2048


class TestInitializeMazda:
  def test_toggle_enabled(self):
    CP = CarInterface.get_non_essential_params(CAR.MAZDA_CX5)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.MAZDA_CX5)
    _initialize_mazda(CP, CP_SP, {"MazdaTorqueInterceptor": "1"})
    assert CP_SP.flags & MazdaFlagsSP.TORQUE_INTERCEPTOR
    assert CP_SP.safetyParam & MazdaSafetyFlagsSP.TORQUE_INTERCEPTOR
    assert CP.minSteerSpeed == 0.
    assert not CP.dashcamOnly

  def test_toggle_disabled(self):
    CP = CarInterface.get_non_essential_params(CAR.MAZDA_CX5)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.MAZDA_CX5)
    min_steer_speed = CP.minSteerSpeed
    _initialize_mazda(CP, CP_SP, {})
    assert not CP_SP.flags & MazdaFlagsSP.TORQUE_INTERCEPTOR
    assert CP_SP.safetyParam == 0
    assert CP.minSteerSpeed == min_steer_speed


class TestTorqueInterceptorCarState:
  def setup_method(self):
    self.packer = CANPacker("mazda_2017")
    self.ci = make_car_interface(ti=True)

  def test_body_parser_created(self):
    assert Bus.body in self.ci.can_parsers
    assert Bus.body not in make_car_interface(ti=False).can_parsers

  def test_torque_source_and_threshold(self):
    feed_ti_feedback(self.ci, self.packer, torque=10)
    ret, _ = self.ci.CS.update(self.ci.can_parsers)
    assert ret.steeringTorque == 10
    assert ret.steeringPressed

    feed_ti_feedback(self.ci, self.packer, torque=3, t=1)
    ret, _ = self.ci.CS.update(self.ci.can_parsers)
    assert not ret.steeringPressed

  def test_lkas_allowed_state_machine(self):
    # boot: no TI_FEEDBACK yet, parser default STATE=0 (DISCOVER)
    self.ci.CS.update(self.ci.can_parsers)
    assert not self.ci.CS.ti.lkas_allowed

    feed_ti_feedback(self.ci, self.packer, state=3, t=1)
    self.ci.CS.update(self.ci.can_parsers)
    assert self.ci.CS.ti.lkas_allowed

    for kwargs in ({"state": 1}, {"state": 2}, {"ramp_down": 1}, {"error": 5}):
      feed_ti_feedback(self.ci, self.packer, t=2, **kwargs)
      self.ci.CS.update(self.ci.can_parsers)
      assert not self.ci.CS.ti.lkas_allowed, kwargs

      feed_ti_feedback(self.ci, self.packer, state=3, t=3)
      self.ci.CS.update(self.ci.can_parsers)
      assert self.ci.CS.ti.lkas_allowed

  def test_not_ready_sets_steer_fault(self):
    # no-entry via steerFaultTemporary until the interceptor reaches RUN
    ret, _ = self.ci.CS.update(self.ci.can_parsers)
    assert ret.steerFaultTemporary

    feed_ti_feedback(self.ci, self.packer, state=3, t=1)
    ret, _ = self.ci.CS.update(self.ci.can_parsers)
    assert not ret.steerFaultTemporary

  def test_persistent_failure_sets_steer_fault(self):
    feed_ti_feedback(self.ci, self.packer, state=3)
    self.ci.CS.update(self.ci.can_parsers)

    # a transient error does not fault
    feed_ti_feedback(self.ci, self.packer, error=5, t=1)
    for _ in range(50):
      ret, _ = self.ci.CS.update(self.ci.can_parsers)
    assert not ret.steerFaultTemporary

    # a persistent (>1s) error faults even while driving
    for _ in range(60):
      ret, _ = self.ci.CS.update(self.ci.can_parsers)
    assert ret.steerFaultTemporary

    # recovery clears the fault
    feed_ti_feedback(self.ci, self.packer, state=3, t=2)
    ret, _ = self.ci.CS.update(self.ci.can_parsers)
    assert not ret.steerFaultTemporary

  def test_driver_over_does_not_fault(self):
    feed_ti_feedback(self.ci, self.packer, state=3)
    self.ci.CS.update(self.ci.can_parsers)

    # long DRIVER_OVER (driver holding the wheel) pauses torque but never faults
    feed_ti_feedback(self.ci, self.packer, state=2, t=1)
    for _ in range(200):
      ret, _ = self.ci.CS.update(self.ci.can_parsers)
    assert not self.ci.CS.ti.lkas_allowed
    assert not ret.steerFaultTemporary


class TestTorqueInterceptorCarController:
  def setup_method(self):
    self.packer = CANPacker("mazda_2017")
    self.ci = make_car_interface(ti=True)
    self.ci.CS.update(self.ci.can_parsers)
    self.CC_SP = structs.CarControlSP()

  def _ti_msgs(self, lat_active=True):
    _, can_sends = self.ci.CC.update(make_car_control(lat_active), self.CC_SP, self.ci.CS, 0)
    return [m for m in can_sends if m[0] == TI_LKAS]

  def test_keepalive_every_frame(self):
    # keepalive with zero torque must be sent even when inactive or not allowed
    for lat_active in (False, True):
      msgs = self._ti_msgs(lat_active)
      assert len(msgs) == 1
      assert msgs[0][2] == 1  # bus
      assert ti_torque_from_msg(msgs[0]) == 0

  def test_rate_limit_and_max(self):
    feed_ti_feedback(self.ci, self.packer, state=3)
    self.ci.CS.update(self.ci.can_parsers)

    prev = 0
    peak = 0
    for _ in range(150):
      msgs = self._ti_msgs()
      assert len(msgs) == 1
      torque = ti_torque_from_msg(msgs[0])
      assert torque - prev <= TICarControllerParams.STEER_DELTA_UP
      prev = torque
      peak = max(peak, torque)
    assert peak == TICarControllerParams.STEER_MAX

  def test_no_ti_message_when_disabled(self):
    ci = make_car_interface(ti=False)
    ci.CS.update(ci.can_parsers)
    _, can_sends = ci.CC.update(make_car_control(True), self.CC_SP, ci.CS, 0)
    assert not any(m[0] == TI_LKAS for m in can_sends)
