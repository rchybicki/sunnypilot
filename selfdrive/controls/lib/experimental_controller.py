import numpy as np
from common.conversions import Conversions as CV
from selfdrive.controls.lib.lateral_planner import TRAJECTORY_SIZE
from common.numpy_fast import clip, interp
from common.realtime import sec_since_boot
from common.params import Params, put_bool_nonblocking

# Time threshold for Conditional Experimental Mode (Code runs at 20hz, so: THRESHOLD / 20 = seconds)
THRESHOLD = 5 # 0.25s

# Lookup table for stop sign / stop light detection. Credit goes to the DragonPilot team!
STOP_SIGN_BREAKING_POINT = [0., 10., 20., 30., 40., 50., 55.]
STOP_SIGN_DISTANCE = [10, 30., 50., 70., 80., 90., 120.]

class ExperimentalController():
  def __init__(self, VM):
    self.VM = VM
    self.op_enabled = False
    self.gas_pressed = False
    self.last_params_update = 0
    self.v_ego = 0
    self.v_ego_kph = 0
    self.curve = False
    self.curvature_count = 0
    self.enabled_experimental = False
    self.lead_status_count = 0
    self.stop_light_count = 0
    self.previous_lead_status = False
    self.previous_lead_speed = 0
    self.params = Params()
    self.enabled = not self.params.get_bool("TurnSpeedControl")

  def update_params(self):
    time = sec_since_boot()
    if time > self.last_params_update + 5.0:
      self.enabled = not self.params.get_bool("TurnSpeedControl")
      self.last_params_update = time

  def road_curvature(self, lead, standstill):
    # Check if there's no lead vehicle if the toggle is on
    if not lead and not standstill:
      predicted_lateral_accelerations = np.abs(np.array(self.modelData.acceleration.y))
      predicted_velocities = np.array(self.modelData.velocity.x)
      if len(predicted_lateral_accelerations) == len(predicted_velocities) != 0:
        curvature_ratios = predicted_lateral_accelerations / (predicted_velocities ** 2)
        predicted_lateral_accelerations = curvature_ratios * (self.v_ego ** 2)
        curvature = np.amax(predicted_lateral_accelerations)
        if curvature >= 1.3 or (self.curve and curvature > 1.1):
          # Setting the maximum to 10 lets it hold the status for 0.25s after it goes "false" to help prevent false negatives
          self.curvature_count = min(10, self.curvature_count + 1)
        else:
          self.curvature_count = max(0, self.curvature_count - 1)
        # Check if curve is detected for > 0.25s
        return self.curvature_count >= THRESHOLD
    return False

  # Stop sign and stop light detection - Credit goes to the DragonPilot team!
  def stop_sign_and_light(self, lead, standstill):
    lead_speed = self.radarState.leadOne.vLead
    if abs(self.carState.steeringAngleDeg) <= 60 and not standstill:
      # Check to make sure we don't have a lead that's stopping for the red light / stop sign
      if not (lead and self.previous_lead_speed > lead_speed) or self.radarState.leadOne.dRel >= 10:
        if len(self.modelData.orientation.x) == len(self.modelData.position.x) == TRAJECTORY_SIZE:
          if self.modelData.position.x[TRAJECTORY_SIZE - 1] < interp(self.v_ego_kph, STOP_SIGN_BREAKING_POINT, STOP_SIGN_DISTANCE):
            self.stop_light_count = min(10, self.stop_light_count + 1)
          else:
            self.stop_light_count = max(0, self.stop_light_count - 1)
        else:
          self.stop_light_count = max(0, self.stop_light_count - 1)
      else:
        self.stop_light_count = max(0, self.stop_light_count - 1)
    else:
      self.stop_light_count = max(0, self.stop_light_count - 1)
    self.previous_lead_speed = lead_speed
    # Check if stop sign / stop light is detected for > 0.25s
    return self.stop_light_count >= THRESHOLD
  
  def detect_lead(self):
    lead_status = self.radarState.leadOne.status
    self.lead_status_count = (self.lead_status_count + 1) if lead_status == self.previous_lead_status else 0
    self.previous_lead_status = lead_status
    # Check if lead is detected for > 0.25s
    return self.lead_status_count >= THRESHOLD and lead_status

  def update_calculations(self):
    lead = self.detect_lead()
    standstill = self.carState.standstill
    signal = self.v_ego < 25 and (self.carState.leftBlinker or self.carState.rightBlinker)
    self.curve = self.road_curvature(lead, standstill)
    stop_light_detected = self.stop_sign_and_light(lead, standstill)
    speed = self.v_ego_kph <= 30.
    self.active = (self.curve or stop_light_detected or standstill or signal or speed) and not self.gas_pressed and self.op_enabled


  def update_experimental_mode(self):
    if not self.enabled:
      return
    experimental_mode = self.params.get_bool("ExperimentalMode")
    if self.active and not experimental_mode and not self.enabled_experimental:
      self.enabled_experimental = True
      put_bool_nonblocking("ExperimentalMode", True)
    elif not self.active and experimental_mode and self.enabled_experimental:
        put_bool_nonblocking("ExperimentalMode", False)
    elif not self.active and not experimental_mode and self.enabled_experimental:
      self.enabled_experimental = False


  def update(self, op_enabled, v_ego, sm):
    self.op_enabled = op_enabled
    self.carState, self.modelData, self.radarState = (sm[key] for key in ['carState', 'modelV2', 'radarState'])
    self.gas_pressed = self.carState.gasPressed
    self.v_ego = v_ego
    self.v_ego_kph = v_ego * 3.6

    self.update_params()
    self.update_calculations()
    self.update_experimental_mode()