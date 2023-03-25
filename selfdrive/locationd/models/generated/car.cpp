#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_968838101684317911) {
   out_968838101684317911[0] = delta_x[0] + nom_x[0];
   out_968838101684317911[1] = delta_x[1] + nom_x[1];
   out_968838101684317911[2] = delta_x[2] + nom_x[2];
   out_968838101684317911[3] = delta_x[3] + nom_x[3];
   out_968838101684317911[4] = delta_x[4] + nom_x[4];
   out_968838101684317911[5] = delta_x[5] + nom_x[5];
   out_968838101684317911[6] = delta_x[6] + nom_x[6];
   out_968838101684317911[7] = delta_x[7] + nom_x[7];
   out_968838101684317911[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4310908025649758055) {
   out_4310908025649758055[0] = -nom_x[0] + true_x[0];
   out_4310908025649758055[1] = -nom_x[1] + true_x[1];
   out_4310908025649758055[2] = -nom_x[2] + true_x[2];
   out_4310908025649758055[3] = -nom_x[3] + true_x[3];
   out_4310908025649758055[4] = -nom_x[4] + true_x[4];
   out_4310908025649758055[5] = -nom_x[5] + true_x[5];
   out_4310908025649758055[6] = -nom_x[6] + true_x[6];
   out_4310908025649758055[7] = -nom_x[7] + true_x[7];
   out_4310908025649758055[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1396426837997155367) {
   out_1396426837997155367[0] = 1.0;
   out_1396426837997155367[1] = 0;
   out_1396426837997155367[2] = 0;
   out_1396426837997155367[3] = 0;
   out_1396426837997155367[4] = 0;
   out_1396426837997155367[5] = 0;
   out_1396426837997155367[6] = 0;
   out_1396426837997155367[7] = 0;
   out_1396426837997155367[8] = 0;
   out_1396426837997155367[9] = 0;
   out_1396426837997155367[10] = 1.0;
   out_1396426837997155367[11] = 0;
   out_1396426837997155367[12] = 0;
   out_1396426837997155367[13] = 0;
   out_1396426837997155367[14] = 0;
   out_1396426837997155367[15] = 0;
   out_1396426837997155367[16] = 0;
   out_1396426837997155367[17] = 0;
   out_1396426837997155367[18] = 0;
   out_1396426837997155367[19] = 0;
   out_1396426837997155367[20] = 1.0;
   out_1396426837997155367[21] = 0;
   out_1396426837997155367[22] = 0;
   out_1396426837997155367[23] = 0;
   out_1396426837997155367[24] = 0;
   out_1396426837997155367[25] = 0;
   out_1396426837997155367[26] = 0;
   out_1396426837997155367[27] = 0;
   out_1396426837997155367[28] = 0;
   out_1396426837997155367[29] = 0;
   out_1396426837997155367[30] = 1.0;
   out_1396426837997155367[31] = 0;
   out_1396426837997155367[32] = 0;
   out_1396426837997155367[33] = 0;
   out_1396426837997155367[34] = 0;
   out_1396426837997155367[35] = 0;
   out_1396426837997155367[36] = 0;
   out_1396426837997155367[37] = 0;
   out_1396426837997155367[38] = 0;
   out_1396426837997155367[39] = 0;
   out_1396426837997155367[40] = 1.0;
   out_1396426837997155367[41] = 0;
   out_1396426837997155367[42] = 0;
   out_1396426837997155367[43] = 0;
   out_1396426837997155367[44] = 0;
   out_1396426837997155367[45] = 0;
   out_1396426837997155367[46] = 0;
   out_1396426837997155367[47] = 0;
   out_1396426837997155367[48] = 0;
   out_1396426837997155367[49] = 0;
   out_1396426837997155367[50] = 1.0;
   out_1396426837997155367[51] = 0;
   out_1396426837997155367[52] = 0;
   out_1396426837997155367[53] = 0;
   out_1396426837997155367[54] = 0;
   out_1396426837997155367[55] = 0;
   out_1396426837997155367[56] = 0;
   out_1396426837997155367[57] = 0;
   out_1396426837997155367[58] = 0;
   out_1396426837997155367[59] = 0;
   out_1396426837997155367[60] = 1.0;
   out_1396426837997155367[61] = 0;
   out_1396426837997155367[62] = 0;
   out_1396426837997155367[63] = 0;
   out_1396426837997155367[64] = 0;
   out_1396426837997155367[65] = 0;
   out_1396426837997155367[66] = 0;
   out_1396426837997155367[67] = 0;
   out_1396426837997155367[68] = 0;
   out_1396426837997155367[69] = 0;
   out_1396426837997155367[70] = 1.0;
   out_1396426837997155367[71] = 0;
   out_1396426837997155367[72] = 0;
   out_1396426837997155367[73] = 0;
   out_1396426837997155367[74] = 0;
   out_1396426837997155367[75] = 0;
   out_1396426837997155367[76] = 0;
   out_1396426837997155367[77] = 0;
   out_1396426837997155367[78] = 0;
   out_1396426837997155367[79] = 0;
   out_1396426837997155367[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4044180442470054806) {
   out_4044180442470054806[0] = state[0];
   out_4044180442470054806[1] = state[1];
   out_4044180442470054806[2] = state[2];
   out_4044180442470054806[3] = state[3];
   out_4044180442470054806[4] = state[4];
   out_4044180442470054806[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4044180442470054806[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4044180442470054806[7] = state[7];
   out_4044180442470054806[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8407255564466940885) {
   out_8407255564466940885[0] = 1;
   out_8407255564466940885[1] = 0;
   out_8407255564466940885[2] = 0;
   out_8407255564466940885[3] = 0;
   out_8407255564466940885[4] = 0;
   out_8407255564466940885[5] = 0;
   out_8407255564466940885[6] = 0;
   out_8407255564466940885[7] = 0;
   out_8407255564466940885[8] = 0;
   out_8407255564466940885[9] = 0;
   out_8407255564466940885[10] = 1;
   out_8407255564466940885[11] = 0;
   out_8407255564466940885[12] = 0;
   out_8407255564466940885[13] = 0;
   out_8407255564466940885[14] = 0;
   out_8407255564466940885[15] = 0;
   out_8407255564466940885[16] = 0;
   out_8407255564466940885[17] = 0;
   out_8407255564466940885[18] = 0;
   out_8407255564466940885[19] = 0;
   out_8407255564466940885[20] = 1;
   out_8407255564466940885[21] = 0;
   out_8407255564466940885[22] = 0;
   out_8407255564466940885[23] = 0;
   out_8407255564466940885[24] = 0;
   out_8407255564466940885[25] = 0;
   out_8407255564466940885[26] = 0;
   out_8407255564466940885[27] = 0;
   out_8407255564466940885[28] = 0;
   out_8407255564466940885[29] = 0;
   out_8407255564466940885[30] = 1;
   out_8407255564466940885[31] = 0;
   out_8407255564466940885[32] = 0;
   out_8407255564466940885[33] = 0;
   out_8407255564466940885[34] = 0;
   out_8407255564466940885[35] = 0;
   out_8407255564466940885[36] = 0;
   out_8407255564466940885[37] = 0;
   out_8407255564466940885[38] = 0;
   out_8407255564466940885[39] = 0;
   out_8407255564466940885[40] = 1;
   out_8407255564466940885[41] = 0;
   out_8407255564466940885[42] = 0;
   out_8407255564466940885[43] = 0;
   out_8407255564466940885[44] = 0;
   out_8407255564466940885[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8407255564466940885[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8407255564466940885[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8407255564466940885[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8407255564466940885[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8407255564466940885[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8407255564466940885[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8407255564466940885[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8407255564466940885[53] = -9.8000000000000007*dt;
   out_8407255564466940885[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8407255564466940885[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8407255564466940885[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8407255564466940885[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8407255564466940885[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8407255564466940885[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8407255564466940885[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8407255564466940885[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8407255564466940885[62] = 0;
   out_8407255564466940885[63] = 0;
   out_8407255564466940885[64] = 0;
   out_8407255564466940885[65] = 0;
   out_8407255564466940885[66] = 0;
   out_8407255564466940885[67] = 0;
   out_8407255564466940885[68] = 0;
   out_8407255564466940885[69] = 0;
   out_8407255564466940885[70] = 1;
   out_8407255564466940885[71] = 0;
   out_8407255564466940885[72] = 0;
   out_8407255564466940885[73] = 0;
   out_8407255564466940885[74] = 0;
   out_8407255564466940885[75] = 0;
   out_8407255564466940885[76] = 0;
   out_8407255564466940885[77] = 0;
   out_8407255564466940885[78] = 0;
   out_8407255564466940885[79] = 0;
   out_8407255564466940885[80] = 1;
}
void h_25(double *state, double *unused, double *out_5096407301093087859) {
   out_5096407301093087859[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8334896348201987056) {
   out_8334896348201987056[0] = 0;
   out_8334896348201987056[1] = 0;
   out_8334896348201987056[2] = 0;
   out_8334896348201987056[3] = 0;
   out_8334896348201987056[4] = 0;
   out_8334896348201987056[5] = 0;
   out_8334896348201987056[6] = 1;
   out_8334896348201987056[7] = 0;
   out_8334896348201987056[8] = 0;
}
void h_24(double *state, double *unused, double *out_3230224340948962135) {
   out_3230224340948962135[0] = state[4];
   out_3230224340948962135[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3910815045656403134) {
   out_3910815045656403134[0] = 0;
   out_3910815045656403134[1] = 0;
   out_3910815045656403134[2] = 0;
   out_3910815045656403134[3] = 0;
   out_3910815045656403134[4] = 1;
   out_3910815045656403134[5] = 0;
   out_3910815045656403134[6] = 0;
   out_3910815045656403134[7] = 0;
   out_3910815045656403134[8] = 0;
   out_3910815045656403134[9] = 0;
   out_3910815045656403134[10] = 0;
   out_3910815045656403134[11] = 0;
   out_3910815045656403134[12] = 0;
   out_3910815045656403134[13] = 0;
   out_3910815045656403134[14] = 1;
   out_3910815045656403134[15] = 0;
   out_3910815045656403134[16] = 0;
   out_3910815045656403134[17] = 0;
}
void h_30(double *state, double *unused, double *out_5515532985104572826) {
   out_5515532985104572826[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8464235295345227126) {
   out_8464235295345227126[0] = 0;
   out_8464235295345227126[1] = 0;
   out_8464235295345227126[2] = 0;
   out_8464235295345227126[3] = 0;
   out_8464235295345227126[4] = 1;
   out_8464235295345227126[5] = 0;
   out_8464235295345227126[6] = 0;
   out_8464235295345227126[7] = 0;
   out_8464235295345227126[8] = 0;
}
void h_26(double *state, double *unused, double *out_8466670898888994856) {
   out_8466670898888994856[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6370344406633508336) {
   out_6370344406633508336[0] = 0;
   out_6370344406633508336[1] = 0;
   out_6370344406633508336[2] = 0;
   out_6370344406633508336[3] = 0;
   out_6370344406633508336[4] = 0;
   out_6370344406633508336[5] = 0;
   out_6370344406633508336[6] = 0;
   out_6370344406633508336[7] = 1;
   out_6370344406633508336[8] = 0;
}
void h_27(double *state, double *unused, double *out_8348663504955618112) {
   out_8348663504955618112[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7807745466563899579) {
   out_7807745466563899579[0] = 0;
   out_7807745466563899579[1] = 0;
   out_7807745466563899579[2] = 0;
   out_7807745466563899579[3] = 1;
   out_7807745466563899579[4] = 0;
   out_7807745466563899579[5] = 0;
   out_7807745466563899579[6] = 0;
   out_7807745466563899579[7] = 0;
   out_7807745466563899579[8] = 0;
}
void h_29(double *state, double *unused, double *out_2637521804505045179) {
   out_2637521804505045179[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6094382739694348546) {
   out_6094382739694348546[0] = 0;
   out_6094382739694348546[1] = 1;
   out_6094382739694348546[2] = 0;
   out_6094382739694348546[3] = 0;
   out_6094382739694348546[4] = 0;
   out_6094382739694348546[5] = 0;
   out_6094382739694348546[6] = 0;
   out_6094382739694348546[7] = 0;
   out_6094382739694348546[8] = 0;
}
void h_28(double *state, double *unused, double *out_1692423749109706851) {
   out_1692423749109706851[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1011983722624817972) {
   out_1011983722624817972[0] = 1;
   out_1011983722624817972[1] = 0;
   out_1011983722624817972[2] = 0;
   out_1011983722624817972[3] = 0;
   out_1011983722624817972[4] = 0;
   out_1011983722624817972[5] = 0;
   out_1011983722624817972[6] = 0;
   out_1011983722624817972[7] = 0;
   out_1011983722624817972[8] = 0;
}
void h_31(double *state, double *unused, double *out_1188714132811423188) {
   out_1188714132811423188[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8304250386325026628) {
   out_8304250386325026628[0] = 0;
   out_8304250386325026628[1] = 0;
   out_8304250386325026628[2] = 0;
   out_8304250386325026628[3] = 0;
   out_8304250386325026628[4] = 0;
   out_8304250386325026628[5] = 0;
   out_8304250386325026628[6] = 0;
   out_8304250386325026628[7] = 0;
   out_8304250386325026628[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_968838101684317911) {
  err_fun(nom_x, delta_x, out_968838101684317911);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4310908025649758055) {
  inv_err_fun(nom_x, true_x, out_4310908025649758055);
}
void car_H_mod_fun(double *state, double *out_1396426837997155367) {
  H_mod_fun(state, out_1396426837997155367);
}
void car_f_fun(double *state, double dt, double *out_4044180442470054806) {
  f_fun(state,  dt, out_4044180442470054806);
}
void car_F_fun(double *state, double dt, double *out_8407255564466940885) {
  F_fun(state,  dt, out_8407255564466940885);
}
void car_h_25(double *state, double *unused, double *out_5096407301093087859) {
  h_25(state, unused, out_5096407301093087859);
}
void car_H_25(double *state, double *unused, double *out_8334896348201987056) {
  H_25(state, unused, out_8334896348201987056);
}
void car_h_24(double *state, double *unused, double *out_3230224340948962135) {
  h_24(state, unused, out_3230224340948962135);
}
void car_H_24(double *state, double *unused, double *out_3910815045656403134) {
  H_24(state, unused, out_3910815045656403134);
}
void car_h_30(double *state, double *unused, double *out_5515532985104572826) {
  h_30(state, unused, out_5515532985104572826);
}
void car_H_30(double *state, double *unused, double *out_8464235295345227126) {
  H_30(state, unused, out_8464235295345227126);
}
void car_h_26(double *state, double *unused, double *out_8466670898888994856) {
  h_26(state, unused, out_8466670898888994856);
}
void car_H_26(double *state, double *unused, double *out_6370344406633508336) {
  H_26(state, unused, out_6370344406633508336);
}
void car_h_27(double *state, double *unused, double *out_8348663504955618112) {
  h_27(state, unused, out_8348663504955618112);
}
void car_H_27(double *state, double *unused, double *out_7807745466563899579) {
  H_27(state, unused, out_7807745466563899579);
}
void car_h_29(double *state, double *unused, double *out_2637521804505045179) {
  h_29(state, unused, out_2637521804505045179);
}
void car_H_29(double *state, double *unused, double *out_6094382739694348546) {
  H_29(state, unused, out_6094382739694348546);
}
void car_h_28(double *state, double *unused, double *out_1692423749109706851) {
  h_28(state, unused, out_1692423749109706851);
}
void car_H_28(double *state, double *unused, double *out_1011983722624817972) {
  H_28(state, unused, out_1011983722624817972);
}
void car_h_31(double *state, double *unused, double *out_1188714132811423188) {
  h_31(state, unused, out_1188714132811423188);
}
void car_H_31(double *state, double *unused, double *out_8304250386325026628) {
  H_31(state, unused, out_8304250386325026628);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
