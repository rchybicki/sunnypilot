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
void err_fun(double *nom_x, double *delta_x, double *out_4095765816768978677) {
   out_4095765816768978677[0] = delta_x[0] + nom_x[0];
   out_4095765816768978677[1] = delta_x[1] + nom_x[1];
   out_4095765816768978677[2] = delta_x[2] + nom_x[2];
   out_4095765816768978677[3] = delta_x[3] + nom_x[3];
   out_4095765816768978677[4] = delta_x[4] + nom_x[4];
   out_4095765816768978677[5] = delta_x[5] + nom_x[5];
   out_4095765816768978677[6] = delta_x[6] + nom_x[6];
   out_4095765816768978677[7] = delta_x[7] + nom_x[7];
   out_4095765816768978677[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4903182091448254315) {
   out_4903182091448254315[0] = -nom_x[0] + true_x[0];
   out_4903182091448254315[1] = -nom_x[1] + true_x[1];
   out_4903182091448254315[2] = -nom_x[2] + true_x[2];
   out_4903182091448254315[3] = -nom_x[3] + true_x[3];
   out_4903182091448254315[4] = -nom_x[4] + true_x[4];
   out_4903182091448254315[5] = -nom_x[5] + true_x[5];
   out_4903182091448254315[6] = -nom_x[6] + true_x[6];
   out_4903182091448254315[7] = -nom_x[7] + true_x[7];
   out_4903182091448254315[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3077754926992826597) {
   out_3077754926992826597[0] = 1.0;
   out_3077754926992826597[1] = 0;
   out_3077754926992826597[2] = 0;
   out_3077754926992826597[3] = 0;
   out_3077754926992826597[4] = 0;
   out_3077754926992826597[5] = 0;
   out_3077754926992826597[6] = 0;
   out_3077754926992826597[7] = 0;
   out_3077754926992826597[8] = 0;
   out_3077754926992826597[9] = 0;
   out_3077754926992826597[10] = 1.0;
   out_3077754926992826597[11] = 0;
   out_3077754926992826597[12] = 0;
   out_3077754926992826597[13] = 0;
   out_3077754926992826597[14] = 0;
   out_3077754926992826597[15] = 0;
   out_3077754926992826597[16] = 0;
   out_3077754926992826597[17] = 0;
   out_3077754926992826597[18] = 0;
   out_3077754926992826597[19] = 0;
   out_3077754926992826597[20] = 1.0;
   out_3077754926992826597[21] = 0;
   out_3077754926992826597[22] = 0;
   out_3077754926992826597[23] = 0;
   out_3077754926992826597[24] = 0;
   out_3077754926992826597[25] = 0;
   out_3077754926992826597[26] = 0;
   out_3077754926992826597[27] = 0;
   out_3077754926992826597[28] = 0;
   out_3077754926992826597[29] = 0;
   out_3077754926992826597[30] = 1.0;
   out_3077754926992826597[31] = 0;
   out_3077754926992826597[32] = 0;
   out_3077754926992826597[33] = 0;
   out_3077754926992826597[34] = 0;
   out_3077754926992826597[35] = 0;
   out_3077754926992826597[36] = 0;
   out_3077754926992826597[37] = 0;
   out_3077754926992826597[38] = 0;
   out_3077754926992826597[39] = 0;
   out_3077754926992826597[40] = 1.0;
   out_3077754926992826597[41] = 0;
   out_3077754926992826597[42] = 0;
   out_3077754926992826597[43] = 0;
   out_3077754926992826597[44] = 0;
   out_3077754926992826597[45] = 0;
   out_3077754926992826597[46] = 0;
   out_3077754926992826597[47] = 0;
   out_3077754926992826597[48] = 0;
   out_3077754926992826597[49] = 0;
   out_3077754926992826597[50] = 1.0;
   out_3077754926992826597[51] = 0;
   out_3077754926992826597[52] = 0;
   out_3077754926992826597[53] = 0;
   out_3077754926992826597[54] = 0;
   out_3077754926992826597[55] = 0;
   out_3077754926992826597[56] = 0;
   out_3077754926992826597[57] = 0;
   out_3077754926992826597[58] = 0;
   out_3077754926992826597[59] = 0;
   out_3077754926992826597[60] = 1.0;
   out_3077754926992826597[61] = 0;
   out_3077754926992826597[62] = 0;
   out_3077754926992826597[63] = 0;
   out_3077754926992826597[64] = 0;
   out_3077754926992826597[65] = 0;
   out_3077754926992826597[66] = 0;
   out_3077754926992826597[67] = 0;
   out_3077754926992826597[68] = 0;
   out_3077754926992826597[69] = 0;
   out_3077754926992826597[70] = 1.0;
   out_3077754926992826597[71] = 0;
   out_3077754926992826597[72] = 0;
   out_3077754926992826597[73] = 0;
   out_3077754926992826597[74] = 0;
   out_3077754926992826597[75] = 0;
   out_3077754926992826597[76] = 0;
   out_3077754926992826597[77] = 0;
   out_3077754926992826597[78] = 0;
   out_3077754926992826597[79] = 0;
   out_3077754926992826597[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5980213151718331081) {
   out_5980213151718331081[0] = state[0];
   out_5980213151718331081[1] = state[1];
   out_5980213151718331081[2] = state[2];
   out_5980213151718331081[3] = state[3];
   out_5980213151718331081[4] = state[4];
   out_5980213151718331081[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5980213151718331081[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5980213151718331081[7] = state[7];
   out_5980213151718331081[8] = state[8];
}
void F_fun(double *state, double dt, double *out_325952735880629537) {
   out_325952735880629537[0] = 1;
   out_325952735880629537[1] = 0;
   out_325952735880629537[2] = 0;
   out_325952735880629537[3] = 0;
   out_325952735880629537[4] = 0;
   out_325952735880629537[5] = 0;
   out_325952735880629537[6] = 0;
   out_325952735880629537[7] = 0;
   out_325952735880629537[8] = 0;
   out_325952735880629537[9] = 0;
   out_325952735880629537[10] = 1;
   out_325952735880629537[11] = 0;
   out_325952735880629537[12] = 0;
   out_325952735880629537[13] = 0;
   out_325952735880629537[14] = 0;
   out_325952735880629537[15] = 0;
   out_325952735880629537[16] = 0;
   out_325952735880629537[17] = 0;
   out_325952735880629537[18] = 0;
   out_325952735880629537[19] = 0;
   out_325952735880629537[20] = 1;
   out_325952735880629537[21] = 0;
   out_325952735880629537[22] = 0;
   out_325952735880629537[23] = 0;
   out_325952735880629537[24] = 0;
   out_325952735880629537[25] = 0;
   out_325952735880629537[26] = 0;
   out_325952735880629537[27] = 0;
   out_325952735880629537[28] = 0;
   out_325952735880629537[29] = 0;
   out_325952735880629537[30] = 1;
   out_325952735880629537[31] = 0;
   out_325952735880629537[32] = 0;
   out_325952735880629537[33] = 0;
   out_325952735880629537[34] = 0;
   out_325952735880629537[35] = 0;
   out_325952735880629537[36] = 0;
   out_325952735880629537[37] = 0;
   out_325952735880629537[38] = 0;
   out_325952735880629537[39] = 0;
   out_325952735880629537[40] = 1;
   out_325952735880629537[41] = 0;
   out_325952735880629537[42] = 0;
   out_325952735880629537[43] = 0;
   out_325952735880629537[44] = 0;
   out_325952735880629537[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_325952735880629537[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_325952735880629537[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_325952735880629537[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_325952735880629537[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_325952735880629537[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_325952735880629537[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_325952735880629537[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_325952735880629537[53] = -9.8000000000000007*dt;
   out_325952735880629537[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_325952735880629537[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_325952735880629537[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_325952735880629537[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_325952735880629537[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_325952735880629537[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_325952735880629537[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_325952735880629537[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_325952735880629537[62] = 0;
   out_325952735880629537[63] = 0;
   out_325952735880629537[64] = 0;
   out_325952735880629537[65] = 0;
   out_325952735880629537[66] = 0;
   out_325952735880629537[67] = 0;
   out_325952735880629537[68] = 0;
   out_325952735880629537[69] = 0;
   out_325952735880629537[70] = 1;
   out_325952735880629537[71] = 0;
   out_325952735880629537[72] = 0;
   out_325952735880629537[73] = 0;
   out_325952735880629537[74] = 0;
   out_325952735880629537[75] = 0;
   out_325952735880629537[76] = 0;
   out_325952735880629537[77] = 0;
   out_325952735880629537[78] = 0;
   out_325952735880629537[79] = 0;
   out_325952735880629537[80] = 1;
}
void h_25(double *state, double *unused, double *out_7694641462849261130) {
   out_7694641462849261130[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8009217059877103452) {
   out_8009217059877103452[0] = 0;
   out_8009217059877103452[1] = 0;
   out_8009217059877103452[2] = 0;
   out_8009217059877103452[3] = 0;
   out_8009217059877103452[4] = 0;
   out_8009217059877103452[5] = 0;
   out_8009217059877103452[6] = 1;
   out_8009217059877103452[7] = 0;
   out_8009217059877103452[8] = 0;
}
void h_24(double *state, double *unused, double *out_5269556662559138097) {
   out_5269556662559138097[0] = state[4];
   out_5269556662559138097[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2376216019280655464) {
   out_2376216019280655464[0] = 0;
   out_2376216019280655464[1] = 0;
   out_2376216019280655464[2] = 0;
   out_2376216019280655464[3] = 0;
   out_2376216019280655464[4] = 1;
   out_2376216019280655464[5] = 0;
   out_2376216019280655464[6] = 0;
   out_2376216019280655464[7] = 0;
   out_2376216019280655464[8] = 0;
   out_2376216019280655464[9] = 0;
   out_2376216019280655464[10] = 0;
   out_2376216019280655464[11] = 0;
   out_2376216019280655464[12] = 0;
   out_2376216019280655464[13] = 0;
   out_2376216019280655464[14] = 1;
   out_2376216019280655464[15] = 0;
   out_2376216019280655464[16] = 0;
   out_2376216019280655464[17] = 0;
}
void h_30(double *state, double *unused, double *out_7419447400564755241) {
   out_7419447400564755241[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3481520729749495254) {
   out_3481520729749495254[0] = 0;
   out_3481520729749495254[1] = 0;
   out_3481520729749495254[2] = 0;
   out_3481520729749495254[3] = 0;
   out_3481520729749495254[4] = 1;
   out_3481520729749495254[5] = 0;
   out_3481520729749495254[6] = 0;
   out_3481520729749495254[7] = 0;
   out_3481520729749495254[8] = 0;
}
void h_26(double *state, double *unused, double *out_4815042079756057590) {
   out_4815042079756057590[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4267713741003047228) {
   out_4267713741003047228[0] = 0;
   out_4267713741003047228[1] = 0;
   out_4267713741003047228[2] = 0;
   out_4267713741003047228[3] = 0;
   out_4267713741003047228[4] = 0;
   out_4267713741003047228[5] = 0;
   out_4267713741003047228[6] = 0;
   out_4267713741003047228[7] = 1;
   out_4267713741003047228[8] = 0;
}
void h_27(double *state, double *unused, double *out_2468734247013936183) {
   out_2468734247013936183[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5705114800933438471) {
   out_5705114800933438471[0] = 0;
   out_5705114800933438471[1] = 0;
   out_5705114800933438471[2] = 0;
   out_5705114800933438471[3] = 1;
   out_5705114800933438471[4] = 0;
   out_5705114800933438471[5] = 0;
   out_5705114800933438471[6] = 0;
   out_5705114800933438471[7] = 0;
   out_5705114800933438471[8] = 0;
}
void h_29(double *state, double *unused, double *out_1168715281240086167) {
   out_1168715281240086167[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3991752074063887438) {
   out_3991752074063887438[0] = 0;
   out_3991752074063887438[1] = 1;
   out_3991752074063887438[2] = 0;
   out_3991752074063887438[3] = 0;
   out_3991752074063887438[4] = 0;
   out_3991752074063887438[5] = 0;
   out_3991752074063887438[6] = 0;
   out_3991752074063887438[7] = 0;
   out_3991752074063887438[8] = 0;
}
void h_28(double *state, double *unused, double *out_5073139567822633834) {
   out_5073139567822633834[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1090646943005643136) {
   out_1090646943005643136[0] = 1;
   out_1090646943005643136[1] = 0;
   out_1090646943005643136[2] = 0;
   out_1090646943005643136[3] = 0;
   out_1090646943005643136[4] = 0;
   out_1090646943005643136[5] = 0;
   out_1090646943005643136[6] = 0;
   out_1090646943005643136[7] = 0;
   out_1090646943005643136[8] = 0;
}
void h_31(double *state, double *unused, double *out_7114653082606268136) {
   out_7114653082606268136[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3641505638769695752) {
   out_3641505638769695752[0] = 0;
   out_3641505638769695752[1] = 0;
   out_3641505638769695752[2] = 0;
   out_3641505638769695752[3] = 0;
   out_3641505638769695752[4] = 0;
   out_3641505638769695752[5] = 0;
   out_3641505638769695752[6] = 0;
   out_3641505638769695752[7] = 0;
   out_3641505638769695752[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4095765816768978677) {
  err_fun(nom_x, delta_x, out_4095765816768978677);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4903182091448254315) {
  inv_err_fun(nom_x, true_x, out_4903182091448254315);
}
void car_H_mod_fun(double *state, double *out_3077754926992826597) {
  H_mod_fun(state, out_3077754926992826597);
}
void car_f_fun(double *state, double dt, double *out_5980213151718331081) {
  f_fun(state,  dt, out_5980213151718331081);
}
void car_F_fun(double *state, double dt, double *out_325952735880629537) {
  F_fun(state,  dt, out_325952735880629537);
}
void car_h_25(double *state, double *unused, double *out_7694641462849261130) {
  h_25(state, unused, out_7694641462849261130);
}
void car_H_25(double *state, double *unused, double *out_8009217059877103452) {
  H_25(state, unused, out_8009217059877103452);
}
void car_h_24(double *state, double *unused, double *out_5269556662559138097) {
  h_24(state, unused, out_5269556662559138097);
}
void car_H_24(double *state, double *unused, double *out_2376216019280655464) {
  H_24(state, unused, out_2376216019280655464);
}
void car_h_30(double *state, double *unused, double *out_7419447400564755241) {
  h_30(state, unused, out_7419447400564755241);
}
void car_H_30(double *state, double *unused, double *out_3481520729749495254) {
  H_30(state, unused, out_3481520729749495254);
}
void car_h_26(double *state, double *unused, double *out_4815042079756057590) {
  h_26(state, unused, out_4815042079756057590);
}
void car_H_26(double *state, double *unused, double *out_4267713741003047228) {
  H_26(state, unused, out_4267713741003047228);
}
void car_h_27(double *state, double *unused, double *out_2468734247013936183) {
  h_27(state, unused, out_2468734247013936183);
}
void car_H_27(double *state, double *unused, double *out_5705114800933438471) {
  H_27(state, unused, out_5705114800933438471);
}
void car_h_29(double *state, double *unused, double *out_1168715281240086167) {
  h_29(state, unused, out_1168715281240086167);
}
void car_H_29(double *state, double *unused, double *out_3991752074063887438) {
  H_29(state, unused, out_3991752074063887438);
}
void car_h_28(double *state, double *unused, double *out_5073139567822633834) {
  h_28(state, unused, out_5073139567822633834);
}
void car_H_28(double *state, double *unused, double *out_1090646943005643136) {
  H_28(state, unused, out_1090646943005643136);
}
void car_h_31(double *state, double *unused, double *out_7114653082606268136) {
  h_31(state, unused, out_7114653082606268136);
}
void car_H_31(double *state, double *unused, double *out_3641505638769695752) {
  H_31(state, unused, out_3641505638769695752);
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
