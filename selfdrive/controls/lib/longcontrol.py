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
  started_condition = v_ego > CP.vEgoStarting

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
    self.stopping_breakpoint_recorded = False

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
    force_stop = False #self.CP.carName == "hyundai" and CS.gapAdjustCruiseTr == 1 and CS.vEgo < 15.
    new_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo, CS.aEgo,
                                                       v_target, v_target_1sec, CS.brakePressed, CS.cruiseState.standstill, force_stop)

    if self.long_control_state != LongCtrlState.stopping and new_control_state == LongCtrlState.stopping:    
      self.stopping_pid.reset()
      self.stopping_breakpoint_recorded = False
      initial_stopping_accel = random.random() * -1.8 -0.1 if force_stop else CS.aEgo
      initial_stopping_speed = random.random() * 5. + 1. if force_stop else CS.vEgo

      self.stopping_v_bp =  [ 0.,    0.1,   0.25,  0.39,  0.4,  max(initial_stopping_speed,  0.6)  ]
      self.stopping_accel = [-0.12, -0.1,  -0.15, -0.45, -0.45, min(initial_stopping_accel, -0.45) ] 
      
      kiBP = [ 0. ]
      kiV = [ 0. ]

      self.stopping_pid._k_i = (kiBP, kiV)
      # print(f"Starting to stop, initial accel {self.initial_stopping_accel}")                            
    
    self.long_control_state = new_control_state

    if self.long_control_state == LongCtrlState.off:
      self.reset(CS.vEgo)
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      
      # smooth expected stopping accel
      expected_accel = interp(CS.vEgo, self.stopping_v_bp, self.stopping_accel)
      error = expected_accel - CS.aEgo

      breakpoint_kpV = 0.015

      if not self.stopping_breakpoint_recorded and CS.vEgo < 0.4:
        self.stopping_breakpoint_recorded = True
        breakpoint_kpV_bp = [ -1.,  -0.1 ]
        breakpoint_kpV_v =  [ 0.02,  0.008 ]

        breakpoint_kpV = interp(CS.aEgo, breakpoint_kpV_bp, breakpoint_kpV_v)


      kpV = [ 0.1 if error < 0 else 0.006, 0.006, breakpoint_kpV, breakpoint_kpV, 0.005, 0.035 if CS.aEgo < -0.7 and error > 0.0 else 0.005 ]
      self.stopping_pid._k_p = (self.stopping_v_bp, kpV)

      error = error if error < 0 or error > abs(0.15 * CS.aEgo) else 0.
      next = 0. # interp(CS.vEgo + expected_accel * 0.01, self.stopping_v_bp, self.stopping_accel) - expected_accel
      update = self.stopping_pid.update(error, speed=CS.vEgo, feedforward=next)
      output_accel += update

      output_accel = clip(output_accel, self.CP.stopAccel, 0.0)
      # print(f"clipped output_accel {output_accel}")    

      self.reset(CS.vEgo)

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset(CS.vEgo)

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

    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])

    return self.last_output_accel
