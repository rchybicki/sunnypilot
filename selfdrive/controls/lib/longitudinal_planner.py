#!/usr/bin/env python3
import math
import numpy as np
from common.numpy_fast import clip, interp

import cereal.messaging as messaging
from cereal import car
from common.conversions import Conversions as CV
from common.filter_simple import FirstOrderFilter
from common.realtime import DT_MDL
from selfdrive.modeld.constants import T_IDXS
from selfdrive.controls.lib.longcontrol import LongCtrlState
from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, MIN_ACCEL, MAX_ACCEL, N
from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N, get_speed_error
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.controls.lib.vision_turn_controller import VisionTurnController
from selfdrive.controls.lib.speed_limit_controller import SpeedLimitController, SpeedLimitResolver
from selfdrive.controls.lib.turn_speed_controller import TurnSpeedController
from selfdrive.controls.lib.experimental_controller import ExperimentalController
from selfdrive.controls.lib.events import Events
from system.swaglog import cloudlog


LON_MPC_STEP = 0.2  # first step is 0.2s

A_CRUISE_MAX_VAL_GAP4 = [ 0.75, 0.7, 0.65, 0.6, 0.55, 0.5,  0.5,  0.4,  0.2 ]
A_CRUISE_MAX_VAL_GAP3 = [ 1.6,  1.5, 1.2,  1.1, 0.9,  0.8,  0.7,  0.6,  0.4 ]
A_CRUISE_MAX_VAL_GAP2 = A_CRUISE_MAX_VAL_GAP3 #[ 1.2, 1.4, 1.3, 1.2, 1.0,  0.8,  0.6,  0.5,  0.3]
A_CRUISE_MAX_VAL_GAP1 = A_CRUISE_MAX_VAL_GAP2 #[ 1.4, 1.6, 1.3, 1.2, 1.0,  0.8,  0.6,  0.5,  0.3]
             # in kph      0   7.2   28   39    54    72    90    108   195
A_CRUISE_MAX_BP =       [ 0.,   2.,  8.,  11.,  15.,  20.,  25.,  30.,  55.  ]

# _DP_CRUISE_MAX_V =       [3.5, 3.4, 2.1, 1.6, 1.1, 0.91, 0.68, 0.44, 0.34, 0.13]
# _DP_CRUISE_MAX_V_ECO =   [3.0, 1.7, 1.3, 0.7, 0.6, 0.44, 0.32, 0.22, 0.16, 0.0078]
# _DP_CRUISE_MAX_V_SPORT = [3.5, 3.5, 3.4, 3.0, 2.1, 1.61, 1.1,  0.63, 0.50, 0.33]
# _DP_CRUISE_MAX_BP =      [0.,  3,   6.,  8.,  11., 15.,  20.,  25.,  30.,  55.]

#A_CRUISE_MAX_VALS = [1.4, 1.0, 0.7, 0.5] # was [1.6, 1.2, 0.8, 0.6]
#A_CRUISE_MAX_BP = [0., 10.0, 25., 40.] # 0km/h, 36km/h, 90km/h, 144km/h

CRUISE_MIN_VAL_GAP3 =       [-0.65,  -0.60,  -0.7,  -0.8,   -1.0,  -1.5 ]
CRUISE_MIN_VAL_GAP4 = CRUISE_MIN_VAL_GAP3 #      [-0.65,  -0.60,  -0.73, -0.75,  -0.75, -0.75 ]
CRUISE_MIN_VAL_GAP2 = CRUISE_MIN_VAL_GAP3 #      [-0.65,  -0.60,  -1.3,  -1.2,   -1.6,  -2.0  ]
CRUISE_MIN_VAL_GAP1 = CRUISE_MIN_VAL_GAP3 #      [-0.65,  -0.60,  -1.5,  -1.7,   -2.0,  -2.5  ]
#                              0       0.5       36     72      108   195
CRUISE_MIN_BP =             [ 0.,     0.07,   10.,   20.,    30.,   55.  ]

# _DP_CRUISE_MIN_V =       [-0.65,  -0.60,  -0.73, -0.75,  -0.75, -0.75]
# _DP_CRUISE_MIN_V_ECO =   [-0.65,  -0.60,  -0.70, -0.70,  -0.65, -0.65]
# _DP_CRUISE_MIN_V_SPORT = [-0.70,  -0.80,  -0.90, -0.90,  -0.80, -0.70]
# _DP_CRUISE_MIN_BP =      [0.,     0.07,   10.,   20.,    30.,   55.]

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

EventName = car.CarEvent.EventName


def get_min_max_accel(v_ego, carstate):
  if carstate.gapAdjustCruiseTr == 1:
    return [interp(v_ego, CRUISE_MIN_BP, CRUISE_MIN_VAL_GAP1), interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VAL_GAP1)]
  elif carstate.gapAdjustCruiseTr == 2:
    return [interp(v_ego, CRUISE_MIN_BP, CRUISE_MIN_VAL_GAP2), interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VAL_GAP2)]
  elif carstate.gapAdjustCruiseTr == 4:
    return [interp(v_ego, CRUISE_MIN_BP, CRUISE_MIN_VAL_GAP4), interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VAL_GAP4)]
  else:
    return [interp(v_ego, CRUISE_MIN_BP, CRUISE_MIN_VAL_GAP3), interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VAL_GAP3)]


def limit_accel_in_turns(v_ego, angle_steers, a_target, live_params, VM):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """

  # FIXME: The lookup table for turns should be updated for VehicleModel lat accel calculations
  a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  omega = VM.yaw_rate(angle_steers * CV.DEG_TO_RAD, v_ego, live_params.roll)
  a_y = v_ego * omega
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0):
    self.CP = CP
    self.mpc = LongitudinalMpc()
    self.fcw = False

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, DT_MDL)
    self.v_model_error = 0.0

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0
    self.VM = VehicleModel(CP)

    self.cruise_source = 'cruise'
    self.vision_turn_controller = VisionTurnController(CP)
    self.speed_limit_controller = SpeedLimitController()
    self.events = Events()
    self.turn_speed_controller = TurnSpeedController()
    self.experimental_controller = ExperimentalController(self.VM)

  @staticmethod
  def parse_model(model_msg, model_error):
    if (len(model_msg.position.x) == 33 and
       len(model_msg.velocity.x) == 33 and
       len(model_msg.acceleration.x) == 33):
      x = np.interp(T_IDXS_MPC, T_IDXS, model_msg.position.x) - model_error * T_IDXS_MPC
      v = np.interp(T_IDXS_MPC, T_IDXS, model_msg.velocity.x) - model_error
      a = np.interp(T_IDXS_MPC, T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    return x, v, a, j

  def update(self, sm):
    self.mpc.mode = 'blended' if sm['controlsState'].experimentalMode else 'acc'

    v_ego = sm['carState'].vEgo
    a_ego = sm['carState'].aEgo
    v_cruise_kph = sm['controlsState'].vCruise
    v_cruise_kph = min(v_cruise_kph, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['controlsState'].enabled

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not sm['carState'].standstill

    # Update VehicleModel
    lp = sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    if self.mpc.mode == 'acc':
      accel_limits = get_min_max_accel(v_ego, sm['carState'])
      accel_limits_turns = limit_accel_in_turns(v_ego, sm['carState'].steeringAngleDeg, accel_limits, lp, self.VM)
    else:
      accel_limits = [MIN_ACCEL, MAX_ACCEL]
      accel_limits_turns = [MIN_ACCEL, MAX_ACCEL]

    leadOne = sm['radarState'].leadOne
    if leadOne.status and leadOne.dRel != 0. and leadOne.dRel < 50.: 
      dRel_bp =    [  6.,  15.,  20. ]
      accel_diff = [ -0.2, 0.5,  2   ]
      max_accel_limited = max(0.6, leadOne.aLeadK + interp(leadOne.dRel, dRel_bp, accel_diff)) if self.mpc.mode == 'acc' else  accel_limits[1]
      accel_limits = [accel_limits[0], min(max_accel_limited, accel_limits[1])]
      accel_limits_turns = [accel_limits_turns[0], min(max_accel_limited, accel_limits_turns[1])]


    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = clip(a_ego, accel_limits[0], accel_limits[1])
      self.mpc.prev_a = np.full(N+1, self.a_desired)

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    # Compute model v_ego error
    self.v_model_error = get_speed_error(sm['modelV2'], v_ego)

    if force_slow_decel:
      v_cruise = 0.0

    # Get acceleration and active solutions for custom long mpc.
    self.cruise_source, a_min_sol, v_cruise_sol = self.cruise_solutions(
      not reset_state and self.CP.openpilotLongitudinalControl, self.v_desired_filter.x,
      self.a_desired, v_cruise, sm)

    # clip limits, cannot init MPC outside of bounds
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired + 0.05, a_min_sol)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired - 0.05)

    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    x, v, a, j = self.parse_model(sm['modelV2'], self.v_model_error)
    self.mpc.update(sm['carState'], sm['radarState'], v_cruise_sol, x, v, a, j, prev_accel_constraint)

    self.v_desired_trajectory_full = np.interp(T_IDXS, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory_full = np.interp(T_IDXS, T_IDXS_MPC, self.mpc.a_solution)
    self.v_desired_trajectory = self.v_desired_trajectory_full[:CONTROL_N]
    self.a_desired_trajectory = self.a_desired_trajectory_full[:CONTROL_N]
    self.j_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(interp(DT_MDL, T_IDXS[:CONTROL_N], self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + DT_MDL * (self.a_desired + a_prev) / 2.0

    self.e2e_events(sm)

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source if self.mpc.source != 'cruise' else self.cruise_source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.e2eX = self.mpc.e2e_x.tolist()
    longitudinalPlan.desiredTF = float(self.mpc.desired_TF)

    longitudinalPlan.visionTurnControllerState = self.vision_turn_controller.state
    longitudinalPlan.visionTurnSpeed = float(self.vision_turn_controller.v_turn)
    longitudinalPlan.visionCurrentLatAcc = float(self.vision_turn_controller.current_lat_acc)
    longitudinalPlan.visionMaxPredLatAcc = float(self.vision_turn_controller.max_pred_lat_acc)

    longitudinalPlan.speedLimitControlState = self.speed_limit_controller.state
    longitudinalPlan.speedLimit = float(self.speed_limit_controller.speed_limit)
    longitudinalPlan.speedLimitOffset = float(self.speed_limit_controller.speed_limit_offset)
    longitudinalPlan.distToSpeedLimit = float(self.speed_limit_controller.distance)
    longitudinalPlan.isMapSpeedLimit = bool(self.speed_limit_controller.source == SpeedLimitResolver.Source.map_data)
    longitudinalPlan.eventsDEPRECATED = self.events.to_msg()

    longitudinalPlan.turnSpeedControlState = self.turn_speed_controller.state
    longitudinalPlan.turnSpeed = float(self.turn_speed_controller.speed_limit)
    longitudinalPlan.distToTurn = float(self.turn_speed_controller.distance)
    longitudinalPlan.turnSign = int(self.turn_speed_controller.turn_sign)

    pm.send('longitudinalPlan', plan_send)

  def cruise_solutions(self, enabled, v_ego, a_ego, v_cruise, sm):
    # Update controllers
    self.vision_turn_controller.update(enabled, v_ego, a_ego, v_cruise, sm)
    self.events = Events()
    self.speed_limit_controller.update(enabled, v_ego, a_ego, sm, v_cruise, self.events)
    self.turn_speed_controller.update(enabled, v_ego, a_ego, sm)
    self.experimental_controller.update(enabled, v_ego, sm)

    # Pick solution with the lowest velocity target.
    a_solutions = {'cruise': float("inf")}
    v_solutions = {'cruise': v_cruise}

    if self.speed_limit_controller.is_active:
      a_solutions['limit'] = self.speed_limit_controller.a_target
      v_solutions['limit'] = self.speed_limit_controller.speed_limit_offseted

    if not sm['controlsState'].experimentalMode:
      if self.turn_speed_controller.is_active:
        a_solutions['turnlimit'] = self.turn_speed_controller.a_target
        v_solutions['turnlimit'] = self.turn_speed_controller.speed_limit
      elif self.vision_turn_controller.is_active:
        a_solutions['turn'] = self.vision_turn_controller.a_target
        v_solutions['turn'] = self.vision_turn_controller.v_turn

    source = min(v_solutions, key=v_solutions.get)

    return source, a_solutions[source], v_solutions[source]

  def e2e_events(self, sm):
    e2e_long_status = sm['e2eLongState'].status

    if e2e_long_status in (1, 2):
      self.events.add(EventName.e2eLongStart)
