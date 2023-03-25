#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_9189878205404737176) {
   out_9189878205404737176[0] = delta_x[0] + nom_x[0];
   out_9189878205404737176[1] = delta_x[1] + nom_x[1];
   out_9189878205404737176[2] = delta_x[2] + nom_x[2];
   out_9189878205404737176[3] = delta_x[3] + nom_x[3];
   out_9189878205404737176[4] = delta_x[4] + nom_x[4];
   out_9189878205404737176[5] = delta_x[5] + nom_x[5];
   out_9189878205404737176[6] = delta_x[6] + nom_x[6];
   out_9189878205404737176[7] = delta_x[7] + nom_x[7];
   out_9189878205404737176[8] = delta_x[8] + nom_x[8];
   out_9189878205404737176[9] = delta_x[9] + nom_x[9];
   out_9189878205404737176[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3224947780549043673) {
   out_3224947780549043673[0] = -nom_x[0] + true_x[0];
   out_3224947780549043673[1] = -nom_x[1] + true_x[1];
   out_3224947780549043673[2] = -nom_x[2] + true_x[2];
   out_3224947780549043673[3] = -nom_x[3] + true_x[3];
   out_3224947780549043673[4] = -nom_x[4] + true_x[4];
   out_3224947780549043673[5] = -nom_x[5] + true_x[5];
   out_3224947780549043673[6] = -nom_x[6] + true_x[6];
   out_3224947780549043673[7] = -nom_x[7] + true_x[7];
   out_3224947780549043673[8] = -nom_x[8] + true_x[8];
   out_3224947780549043673[9] = -nom_x[9] + true_x[9];
   out_3224947780549043673[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_870993508719181486) {
   out_870993508719181486[0] = 1.0;
   out_870993508719181486[1] = 0;
   out_870993508719181486[2] = 0;
   out_870993508719181486[3] = 0;
   out_870993508719181486[4] = 0;
   out_870993508719181486[5] = 0;
   out_870993508719181486[6] = 0;
   out_870993508719181486[7] = 0;
   out_870993508719181486[8] = 0;
   out_870993508719181486[9] = 0;
   out_870993508719181486[10] = 0;
   out_870993508719181486[11] = 0;
   out_870993508719181486[12] = 1.0;
   out_870993508719181486[13] = 0;
   out_870993508719181486[14] = 0;
   out_870993508719181486[15] = 0;
   out_870993508719181486[16] = 0;
   out_870993508719181486[17] = 0;
   out_870993508719181486[18] = 0;
   out_870993508719181486[19] = 0;
   out_870993508719181486[20] = 0;
   out_870993508719181486[21] = 0;
   out_870993508719181486[22] = 0;
   out_870993508719181486[23] = 0;
   out_870993508719181486[24] = 1.0;
   out_870993508719181486[25] = 0;
   out_870993508719181486[26] = 0;
   out_870993508719181486[27] = 0;
   out_870993508719181486[28] = 0;
   out_870993508719181486[29] = 0;
   out_870993508719181486[30] = 0;
   out_870993508719181486[31] = 0;
   out_870993508719181486[32] = 0;
   out_870993508719181486[33] = 0;
   out_870993508719181486[34] = 0;
   out_870993508719181486[35] = 0;
   out_870993508719181486[36] = 1.0;
   out_870993508719181486[37] = 0;
   out_870993508719181486[38] = 0;
   out_870993508719181486[39] = 0;
   out_870993508719181486[40] = 0;
   out_870993508719181486[41] = 0;
   out_870993508719181486[42] = 0;
   out_870993508719181486[43] = 0;
   out_870993508719181486[44] = 0;
   out_870993508719181486[45] = 0;
   out_870993508719181486[46] = 0;
   out_870993508719181486[47] = 0;
   out_870993508719181486[48] = 1.0;
   out_870993508719181486[49] = 0;
   out_870993508719181486[50] = 0;
   out_870993508719181486[51] = 0;
   out_870993508719181486[52] = 0;
   out_870993508719181486[53] = 0;
   out_870993508719181486[54] = 0;
   out_870993508719181486[55] = 0;
   out_870993508719181486[56] = 0;
   out_870993508719181486[57] = 0;
   out_870993508719181486[58] = 0;
   out_870993508719181486[59] = 0;
   out_870993508719181486[60] = 1.0;
   out_870993508719181486[61] = 0;
   out_870993508719181486[62] = 0;
   out_870993508719181486[63] = 0;
   out_870993508719181486[64] = 0;
   out_870993508719181486[65] = 0;
   out_870993508719181486[66] = 0;
   out_870993508719181486[67] = 0;
   out_870993508719181486[68] = 0;
   out_870993508719181486[69] = 0;
   out_870993508719181486[70] = 0;
   out_870993508719181486[71] = 0;
   out_870993508719181486[72] = 1.0;
   out_870993508719181486[73] = 0;
   out_870993508719181486[74] = 0;
   out_870993508719181486[75] = 0;
   out_870993508719181486[76] = 0;
   out_870993508719181486[77] = 0;
   out_870993508719181486[78] = 0;
   out_870993508719181486[79] = 0;
   out_870993508719181486[80] = 0;
   out_870993508719181486[81] = 0;
   out_870993508719181486[82] = 0;
   out_870993508719181486[83] = 0;
   out_870993508719181486[84] = 1.0;
   out_870993508719181486[85] = 0;
   out_870993508719181486[86] = 0;
   out_870993508719181486[87] = 0;
   out_870993508719181486[88] = 0;
   out_870993508719181486[89] = 0;
   out_870993508719181486[90] = 0;
   out_870993508719181486[91] = 0;
   out_870993508719181486[92] = 0;
   out_870993508719181486[93] = 0;
   out_870993508719181486[94] = 0;
   out_870993508719181486[95] = 0;
   out_870993508719181486[96] = 1.0;
   out_870993508719181486[97] = 0;
   out_870993508719181486[98] = 0;
   out_870993508719181486[99] = 0;
   out_870993508719181486[100] = 0;
   out_870993508719181486[101] = 0;
   out_870993508719181486[102] = 0;
   out_870993508719181486[103] = 0;
   out_870993508719181486[104] = 0;
   out_870993508719181486[105] = 0;
   out_870993508719181486[106] = 0;
   out_870993508719181486[107] = 0;
   out_870993508719181486[108] = 1.0;
   out_870993508719181486[109] = 0;
   out_870993508719181486[110] = 0;
   out_870993508719181486[111] = 0;
   out_870993508719181486[112] = 0;
   out_870993508719181486[113] = 0;
   out_870993508719181486[114] = 0;
   out_870993508719181486[115] = 0;
   out_870993508719181486[116] = 0;
   out_870993508719181486[117] = 0;
   out_870993508719181486[118] = 0;
   out_870993508719181486[119] = 0;
   out_870993508719181486[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_2024835571204853601) {
   out_2024835571204853601[0] = dt*state[3] + state[0];
   out_2024835571204853601[1] = dt*state[4] + state[1];
   out_2024835571204853601[2] = dt*state[5] + state[2];
   out_2024835571204853601[3] = state[3];
   out_2024835571204853601[4] = state[4];
   out_2024835571204853601[5] = state[5];
   out_2024835571204853601[6] = dt*state[7] + state[6];
   out_2024835571204853601[7] = dt*state[8] + state[7];
   out_2024835571204853601[8] = state[8];
   out_2024835571204853601[9] = state[9];
   out_2024835571204853601[10] = state[10];
}
void F_fun(double *state, double dt, double *out_1114880998460348898) {
   out_1114880998460348898[0] = 1;
   out_1114880998460348898[1] = 0;
   out_1114880998460348898[2] = 0;
   out_1114880998460348898[3] = dt;
   out_1114880998460348898[4] = 0;
   out_1114880998460348898[5] = 0;
   out_1114880998460348898[6] = 0;
   out_1114880998460348898[7] = 0;
   out_1114880998460348898[8] = 0;
   out_1114880998460348898[9] = 0;
   out_1114880998460348898[10] = 0;
   out_1114880998460348898[11] = 0;
   out_1114880998460348898[12] = 1;
   out_1114880998460348898[13] = 0;
   out_1114880998460348898[14] = 0;
   out_1114880998460348898[15] = dt;
   out_1114880998460348898[16] = 0;
   out_1114880998460348898[17] = 0;
   out_1114880998460348898[18] = 0;
   out_1114880998460348898[19] = 0;
   out_1114880998460348898[20] = 0;
   out_1114880998460348898[21] = 0;
   out_1114880998460348898[22] = 0;
   out_1114880998460348898[23] = 0;
   out_1114880998460348898[24] = 1;
   out_1114880998460348898[25] = 0;
   out_1114880998460348898[26] = 0;
   out_1114880998460348898[27] = dt;
   out_1114880998460348898[28] = 0;
   out_1114880998460348898[29] = 0;
   out_1114880998460348898[30] = 0;
   out_1114880998460348898[31] = 0;
   out_1114880998460348898[32] = 0;
   out_1114880998460348898[33] = 0;
   out_1114880998460348898[34] = 0;
   out_1114880998460348898[35] = 0;
   out_1114880998460348898[36] = 1;
   out_1114880998460348898[37] = 0;
   out_1114880998460348898[38] = 0;
   out_1114880998460348898[39] = 0;
   out_1114880998460348898[40] = 0;
   out_1114880998460348898[41] = 0;
   out_1114880998460348898[42] = 0;
   out_1114880998460348898[43] = 0;
   out_1114880998460348898[44] = 0;
   out_1114880998460348898[45] = 0;
   out_1114880998460348898[46] = 0;
   out_1114880998460348898[47] = 0;
   out_1114880998460348898[48] = 1;
   out_1114880998460348898[49] = 0;
   out_1114880998460348898[50] = 0;
   out_1114880998460348898[51] = 0;
   out_1114880998460348898[52] = 0;
   out_1114880998460348898[53] = 0;
   out_1114880998460348898[54] = 0;
   out_1114880998460348898[55] = 0;
   out_1114880998460348898[56] = 0;
   out_1114880998460348898[57] = 0;
   out_1114880998460348898[58] = 0;
   out_1114880998460348898[59] = 0;
   out_1114880998460348898[60] = 1;
   out_1114880998460348898[61] = 0;
   out_1114880998460348898[62] = 0;
   out_1114880998460348898[63] = 0;
   out_1114880998460348898[64] = 0;
   out_1114880998460348898[65] = 0;
   out_1114880998460348898[66] = 0;
   out_1114880998460348898[67] = 0;
   out_1114880998460348898[68] = 0;
   out_1114880998460348898[69] = 0;
   out_1114880998460348898[70] = 0;
   out_1114880998460348898[71] = 0;
   out_1114880998460348898[72] = 1;
   out_1114880998460348898[73] = dt;
   out_1114880998460348898[74] = 0;
   out_1114880998460348898[75] = 0;
   out_1114880998460348898[76] = 0;
   out_1114880998460348898[77] = 0;
   out_1114880998460348898[78] = 0;
   out_1114880998460348898[79] = 0;
   out_1114880998460348898[80] = 0;
   out_1114880998460348898[81] = 0;
   out_1114880998460348898[82] = 0;
   out_1114880998460348898[83] = 0;
   out_1114880998460348898[84] = 1;
   out_1114880998460348898[85] = dt;
   out_1114880998460348898[86] = 0;
   out_1114880998460348898[87] = 0;
   out_1114880998460348898[88] = 0;
   out_1114880998460348898[89] = 0;
   out_1114880998460348898[90] = 0;
   out_1114880998460348898[91] = 0;
   out_1114880998460348898[92] = 0;
   out_1114880998460348898[93] = 0;
   out_1114880998460348898[94] = 0;
   out_1114880998460348898[95] = 0;
   out_1114880998460348898[96] = 1;
   out_1114880998460348898[97] = 0;
   out_1114880998460348898[98] = 0;
   out_1114880998460348898[99] = 0;
   out_1114880998460348898[100] = 0;
   out_1114880998460348898[101] = 0;
   out_1114880998460348898[102] = 0;
   out_1114880998460348898[103] = 0;
   out_1114880998460348898[104] = 0;
   out_1114880998460348898[105] = 0;
   out_1114880998460348898[106] = 0;
   out_1114880998460348898[107] = 0;
   out_1114880998460348898[108] = 1;
   out_1114880998460348898[109] = 0;
   out_1114880998460348898[110] = 0;
   out_1114880998460348898[111] = 0;
   out_1114880998460348898[112] = 0;
   out_1114880998460348898[113] = 0;
   out_1114880998460348898[114] = 0;
   out_1114880998460348898[115] = 0;
   out_1114880998460348898[116] = 0;
   out_1114880998460348898[117] = 0;
   out_1114880998460348898[118] = 0;
   out_1114880998460348898[119] = 0;
   out_1114880998460348898[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_3645878914521899003) {
   out_3645878914521899003[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_8291397417584581551) {
   out_8291397417584581551[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8291397417584581551[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8291397417584581551[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_8291397417584581551[3] = 0;
   out_8291397417584581551[4] = 0;
   out_8291397417584581551[5] = 0;
   out_8291397417584581551[6] = 1;
   out_8291397417584581551[7] = 0;
   out_8291397417584581551[8] = 0;
   out_8291397417584581551[9] = 0;
   out_8291397417584581551[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_5617236136998162351) {
   out_5617236136998162351[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_5699742714549703355) {
   out_5699742714549703355[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5699742714549703355[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5699742714549703355[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5699742714549703355[3] = 0;
   out_5699742714549703355[4] = 0;
   out_5699742714549703355[5] = 0;
   out_5699742714549703355[6] = 1;
   out_5699742714549703355[7] = 0;
   out_5699742714549703355[8] = 0;
   out_5699742714549703355[9] = 1;
   out_5699742714549703355[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_8085500609040575343) {
   out_8085500609040575343[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_9116371559383232951) {
   out_9116371559383232951[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_9116371559383232951[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_9116371559383232951[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_9116371559383232951[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_9116371559383232951[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_9116371559383232951[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_9116371559383232951[6] = 0;
   out_9116371559383232951[7] = 1;
   out_9116371559383232951[8] = 0;
   out_9116371559383232951[9] = 0;
   out_9116371559383232951[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_8085500609040575343) {
   out_8085500609040575343[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_9116371559383232951) {
   out_9116371559383232951[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_9116371559383232951[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_9116371559383232951[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_9116371559383232951[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_9116371559383232951[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_9116371559383232951[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_9116371559383232951[6] = 0;
   out_9116371559383232951[7] = 1;
   out_9116371559383232951[8] = 0;
   out_9116371559383232951[9] = 0;
   out_9116371559383232951[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9189878205404737176) {
  err_fun(nom_x, delta_x, out_9189878205404737176);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3224947780549043673) {
  inv_err_fun(nom_x, true_x, out_3224947780549043673);
}
void gnss_H_mod_fun(double *state, double *out_870993508719181486) {
  H_mod_fun(state, out_870993508719181486);
}
void gnss_f_fun(double *state, double dt, double *out_2024835571204853601) {
  f_fun(state,  dt, out_2024835571204853601);
}
void gnss_F_fun(double *state, double dt, double *out_1114880998460348898) {
  F_fun(state,  dt, out_1114880998460348898);
}
void gnss_h_6(double *state, double *sat_pos, double *out_3645878914521899003) {
  h_6(state, sat_pos, out_3645878914521899003);
}
void gnss_H_6(double *state, double *sat_pos, double *out_8291397417584581551) {
  H_6(state, sat_pos, out_8291397417584581551);
}
void gnss_h_20(double *state, double *sat_pos, double *out_5617236136998162351) {
  h_20(state, sat_pos, out_5617236136998162351);
}
void gnss_H_20(double *state, double *sat_pos, double *out_5699742714549703355) {
  H_20(state, sat_pos, out_5699742714549703355);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_8085500609040575343) {
  h_7(state, sat_pos_vel, out_8085500609040575343);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_9116371559383232951) {
  H_7(state, sat_pos_vel, out_9116371559383232951);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_8085500609040575343) {
  h_21(state, sat_pos_vel, out_8085500609040575343);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_9116371559383232951) {
  H_21(state, sat_pos_vel, out_9116371559383232951);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
