import numpy as np
from common.conversions import Conversions as CV
from common.numpy_fast import clip, interp
from common.realtime import sec_since_boot
from common.params import Params


# The minimum amount of lateral acceleration that the controller considers to be an active turn
TURN_ACTIVE_LIMIT = 1.2 # m/s^2, lateral acceleration

TURN_INACTIVE_LIMIT = 1.0 # m/s^2

class ExperimentalTurnController():
  def __init__(self, VM):
    self.VM = VM
    self.op_enabled = False
    self.gas_pressed = False
    self.last_params_update = 0
    self.v_ego = 0
    self.max_pred_lat_acc = 0
    self.current_lat_accel = 0
    self.enabled_experimental = False
    self._active = False
    self.params = Params()
    self.enabled = not self.params.get_bool("TurnSpeedControl")

    self.reset()

  @property
  def active(self):
    turn_limit_exceeded = self.current_lat_accel > TURN_ACTIVE_LIMIT
    return (turn_limit_exceeded or self._active ) and self.op_enabled and not self.gas_pressed and self.enabled


  def reset(self):
    self.current_lat_accel = 0
    self.max_pred_lat_acc = 0
    self.pred_lat_accels = []

  def update_params(self):
    time = sec_since_boot()
    if time > self.last_params_update + 5.0:
      self.enabled = not self.params.get_bool("TurnSpeedControl")
      self.last_params_update = time

  def update_current_state(self, sm):
    """
    Uses the current state of the car to calculate the curvature based off the
    angle of the wheels and store the max acceptable velocity for the curve as
    well as the current lateral acceleration.
    """
    lp = sm['liveParameters']
    sa = sm['carState'].steeringAngleDeg * CV.DEG_TO_RAD
    current_curvature = self.VM.calc_curvature(sa, self.v_ego, lp.roll)

    self.current_lat_accel = current_curvature * self.v_ego**2
    self.current_curvature = current_curvature

  def update_calculations(self, sm):
    self.update_current_state(sm)

    curves = self.pred_lat_accels / (self.pred_velocities ** 2)
    pred_lat_accels = curves * (self.v_ego ** 2)

    # Find max lat accel
    self.max_pred_lat_acc = np.max(pred_lat_accels)
    if self.max_pred_lat_acc > TURN_ACTIVE_LIMIT:
      self._active = True
    elif self.max_pred_lat_acc < TURN_INACTIVE_LIMIT:
       self._active = False

  def update_experimental_mode(self):
    if not self.enabled:
      return
    experimental_mode = self.params.get_bool("ExperimentalMode")
    if self.active and not experimental_mode and not self.enabled_experimental:
      self.enabled_experimental = True
      self.params.put_bool("ExperimentalMode", True)
    elif not self.active and experimental_mode and self.enabled_experimental:
      self.enabled_experimental = False
      self.params.put_bool("ExperimentalMode", False)


  def update(self, op_enabled, v_ego, sm):
    self.op_enabled = op_enabled
    self.gas_pressed = sm['carState'].gasPressed
    self.v_ego = v_ego
    self.pred_lat_accels = np.abs(np.array(sm['modelV2'].acceleration.y))
    self.pred_velocities = np.array(sm['modelV2'].velocity.x)
    if len(self.pred_lat_accels) == 0 or len(self.pred_lat_accels) != len(self.pred_velocities):
      return

    self.update_params()
    self.update_calculations(sm)
    self.update_experimental_mode()