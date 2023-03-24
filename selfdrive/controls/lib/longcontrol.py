import random
from cereal import car
from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL
from selfdrive.controls.lib.drive_helpers import CONTROL_N, apply_deadzone
from selfdrive.controls.lib.pid import PIDController
from selfdrive.modeld.constants import T_IDXS

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP, active, long_control_state, v_ego, a_ego, v_target,
                             v_target_1sec, brake_pressed, cruise_standstill, force_stop): 
  # Ignore cruise standstill if car has a gas interceptor
  cruise_standstill = cruise_standstill and not CP.enableGasInterceptor
  accelerating = v_target_1sec > v_target
  planned_stop = force_stop or (v_target < CP.vEgoStopping and
                  v_target_1sec < CP.vEgoStopping and
                  not accelerating)
  stay_stopped = (v_ego < CP.vEgoStopping and
                  (brake_pressed or cruise_standstill))
  stopping_condition = planned_stop or stay_stopped

  starting_condition = (v_target_1sec > CP.vEgoStarting and
                        accelerating and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > 0.03 or a_ego > 0.1

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state in (LongCtrlState.off, LongCtrlState.pid):
      long_control_state = LongCtrlState.pid
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping and not force_stop:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.starting:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid

  return long_control_state


class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off  # initialized to off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             k_f=CP.longitudinalTuning.kf, rate=1 / DT_CTRL)

    self.stopping_pid = PIDController(([0.], [0.]),
                                      ([0.], [0.]),
                             k_f=CP.longitudinalTuning.kf, rate=1 / DT_CTRL)
    self.v_pid = 0.0
    self.last_output_accel = 0.0
    self.stopping_accel = []
    self.stopping_v_bp = []
    self.stopping_breakpoint = 0.
    self.initial_stopping_accel = 0.

  def reset(self, v_pid):
    """Reset PID controller and change setpoint"""
    self.pid.reset()
    self.v_pid = v_pid

  def update(self, active, CS, long_plan, accel_limits, t_since_plan):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    # Interp control trajectory
    speeds = long_plan.speeds
    if len(speeds) == CONTROL_N:
      v_target_now = interp(t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target_now = interp(t_since_plan, T_IDXS[:CONTROL_N], long_plan.accels)

      v_target_lower = interp(self.CP.longitudinalActuatorDelayLowerBound + t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target_lower = 2 * (v_target_lower - v_target_now) / self.CP.longitudinalActuatorDelayLowerBound - a_target_now

      v_target_upper = interp(self.CP.longitudinalActuatorDelayUpperBound + t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target_upper = 2 * (v_target_upper - v_target_now) / self.CP.longitudinalActuatorDelayUpperBound - a_target_now

      v_target = min(v_target_lower, v_target_upper)
      a_target = min(a_target_lower, a_target_upper)

      v_target_1sec = interp(self.CP.longitudinalActuatorDelayUpperBound + t_since_plan + 1.0, T_IDXS[:CONTROL_N], speeds)
    else:
      v_target = 0.0
      v_target_now = 0.0
      v_target_1sec = 0.0
      a_target = 0.0

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    output_accel = self.last_output_accel
    force_stop = self.CP.carName == "hyundai" and CS.gapAdjustCruiseTr == 1
    new_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo, CS.aEgo,
                                                       v_target, v_target_1sec, CS.brakePressed, CS.cruiseState.standstill, force_stop)

    if self.long_control_state != LongCtrlState.stopping and new_control_state == LongCtrlState.stopping:    

      # self.initial_stopping_accel = random.random() * -1. - 0.2 if force_stop else CS.aEgo
      self.initial_stopping_accel = -1.5 if force_stop else CS.aEgo
      # print(f"Starting to stop, initial accel {self.initial_stopping_accel}")                            

    
    self.long_control_state = new_control_state

    if self.long_control_state == LongCtrlState.off:
      self.reset(CS.vEgo)
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      
      if CS.aEgo < 0.:
        stopping_breakpoint_bp = [ -1.0,  -0.5,  -0.1]    
        stopping_breakpoint_v  = [  0.25,  0.25, 0.18 ]   

        if CS.vEgo > 0.25 and CS.vEgo < 0.26:
          self.initial_stopping_accel = CS.aEgo
          # print(f"stopping accel {self.initial_stopping_accel}")                            

                                    
        self.stopping_breakpoint = interp(self.initial_stopping_accel, stopping_breakpoint_bp, stopping_breakpoint_v)                                 
        self.stopping_v_bp =  [ self.stopping_breakpoint-0.05, self.stopping_breakpoint,  self.stopping_breakpoint+0.01,          0.5,                                    5. ]  #max(CS.vEgo, 0.7) ]
        self.stopping_accel = [ -0.10,                         -0.15,                     max(self.initial_stopping_accel, -0.5), max(self.initial_stopping_accel, -0.5), min(self.initial_stopping_accel, -0.3) ] 
  

        # stopping_a_bp = [ -1.0,    -0.4 ]
        # stoping_a_k =   [ 0.020,  0.012 ]
        # kpV = [ interp(CS.aEgo, stopping_a_bp, stoping_a_k), 0.012 ]
        kpV = [ 0.02, 0.4, 0.008, 0.008, 0.015]

        kiBP = [ 0. ]
        kiV = [ 0.0004 ]
  
        self.stopping_pid._k_p = (self.stopping_v_bp, kpV)
        self.stopping_pid._k_i = (kiBP, kiV)

        # smooth expected stopping accel
        expected_accel = interp(CS.vEgo, self.stopping_v_bp, self.stopping_accel)
        error = expected_accel - CS.aEgo
        next = 0. # interp(CS.vEgo + expected_accel * 0.01, self.stopping_v_bp, self.stopping_accel) - expected_accel
        update = self.stopping_pid.update(error, speed=CS.vEgo, feedforward=next)
        output_accel += update if CS.vEgo < self.stopping_breakpoint+0.01 or update < 0. or CS.vEgo > 1.5 or CS.aEgo < -0.7 else 0.
        
        # print(f"in {self.initial_stopping_accel} bkpt {self.stopping_breakpoint} aEgo {CS.aEgo} vEgo {CS.vEgo} exp {expected_accel} error {error} update {update} output_accel {output_accel}")    
      else:
        #cancel out the car wanting to start when stopping
        output_accel -= 0.5 * DT_CTRL
        # self.stopping_pid.set_i(output_accel)

      breaking_pause = 0.01
      output_min_bp =  [ self.stopping_breakpoint - breaking_pause - 0.001, self.stopping_breakpoint - breaking_pause ]
      output_min_v =   [ -0.35,                                             -0.35                                   ]
      output_accel = clip(output_accel, self.CP.stopAccel, interp(CS.vEgo, output_min_bp, output_min_v))
      # print(f"clipped output_accel {output_accel}")    

        
      self.reset(CS.vEgo)

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset(CS.vEgo)
      self.stopping_pid.reset()

    elif self.long_control_state == LongCtrlState.pid:
      self.v_pid = v_target_now

      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      # TODO too complex, needs to be simplified and tested on toyotas
      prevent_overshoot = not self.CP.stoppingControl and CS.vEgo < 1.5 and v_target_1sec < 0.7 and v_target_1sec < self.v_pid
      deadzone = interp(CS.vEgo, self.CP.longitudinalTuning.deadzoneBP, self.CP.longitudinalTuning.deadzoneV)
      freeze_integrator = prevent_overshoot

      error = self.v_pid - CS.vEgo
      error_deadzone = apply_deadzone(error, deadzone)
      output_accel = self.pid.update(error_deadzone, speed=CS.vEgo,
                                     feedforward=a_target,
                                     freeze_integrator=freeze_integrator)
      
      # might not be the best way to do this, but it limits acceleration jerk 
      # while not limiting braking, smooth as butter!
      if output_accel > 0. and output_accel > self.last_output_accel:
        step_limit_a_bp = [0.,  0.4]
        step_limit_a_k = [0.03, 0.005]
        max_pos_step = interp(CS.aEgo, step_limit_a_bp, step_limit_a_k)
        if output_accel - self.last_output_accel > max_pos_step:
          output_accel = self.last_output_accel + max_pos_step

        step_limit_v_bp = [0.,  7]
        step_limit_v_k = [0.03, 0.004]
        max_pos_step = interp(CS.vEgo, step_limit_v_bp, step_limit_v_k)
        if output_accel - self.last_output_accel > max_pos_step:
          output_accel = self.last_output_accel + max_pos_step
      self.stopping_pid.reset()

    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])
   
    return self.last_output_accel