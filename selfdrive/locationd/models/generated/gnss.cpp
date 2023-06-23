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
void err_fun(double *nom_x, double *delta_x, double *out_5443830885228648884) {
   out_5443830885228648884[0] = delta_x[0] + nom_x[0];
   out_5443830885228648884[1] = delta_x[1] + nom_x[1];
   out_5443830885228648884[2] = delta_x[2] + nom_x[2];
   out_5443830885228648884[3] = delta_x[3] + nom_x[3];
   out_5443830885228648884[4] = delta_x[4] + nom_x[4];
   out_5443830885228648884[5] = delta_x[5] + nom_x[5];
   out_5443830885228648884[6] = delta_x[6] + nom_x[6];
   out_5443830885228648884[7] = delta_x[7] + nom_x[7];
   out_5443830885228648884[8] = delta_x[8] + nom_x[8];
   out_5443830885228648884[9] = delta_x[9] + nom_x[9];
   out_5443830885228648884[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2005231189022139872) {
   out_2005231189022139872[0] = -nom_x[0] + true_x[0];
   out_2005231189022139872[1] = -nom_x[1] + true_x[1];
   out_2005231189022139872[2] = -nom_x[2] + true_x[2];
   out_2005231189022139872[3] = -nom_x[3] + true_x[3];
   out_2005231189022139872[4] = -nom_x[4] + true_x[4];
   out_2005231189022139872[5] = -nom_x[5] + true_x[5];
   out_2005231189022139872[6] = -nom_x[6] + true_x[6];
   out_2005231189022139872[7] = -nom_x[7] + true_x[7];
   out_2005231189022139872[8] = -nom_x[8] + true_x[8];
   out_2005231189022139872[9] = -nom_x[9] + true_x[9];
   out_2005231189022139872[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_3960443324223161311) {
   out_3960443324223161311[0] = 1.0;
   out_3960443324223161311[1] = 0;
   out_3960443324223161311[2] = 0;
   out_3960443324223161311[3] = 0;
   out_3960443324223161311[4] = 0;
   out_3960443324223161311[5] = 0;
   out_3960443324223161311[6] = 0;
   out_3960443324223161311[7] = 0;
   out_3960443324223161311[8] = 0;
   out_3960443324223161311[9] = 0;
   out_3960443324223161311[10] = 0;
   out_3960443324223161311[11] = 0;
   out_3960443324223161311[12] = 1.0;
   out_3960443324223161311[13] = 0;
   out_3960443324223161311[14] = 0;
   out_3960443324223161311[15] = 0;
   out_3960443324223161311[16] = 0;
   out_3960443324223161311[17] = 0;
   out_3960443324223161311[18] = 0;
   out_3960443324223161311[19] = 0;
   out_3960443324223161311[20] = 0;
   out_3960443324223161311[21] = 0;
   out_3960443324223161311[22] = 0;
   out_3960443324223161311[23] = 0;
   out_3960443324223161311[24] = 1.0;
   out_3960443324223161311[25] = 0;
   out_3960443324223161311[26] = 0;
   out_3960443324223161311[27] = 0;
   out_3960443324223161311[28] = 0;
   out_3960443324223161311[29] = 0;
   out_3960443324223161311[30] = 0;
   out_3960443324223161311[31] = 0;
   out_3960443324223161311[32] = 0;
   out_3960443324223161311[33] = 0;
   out_3960443324223161311[34] = 0;
   out_3960443324223161311[35] = 0;
   out_3960443324223161311[36] = 1.0;
   out_3960443324223161311[37] = 0;
   out_3960443324223161311[38] = 0;
   out_3960443324223161311[39] = 0;
   out_3960443324223161311[40] = 0;
   out_3960443324223161311[41] = 0;
   out_3960443324223161311[42] = 0;
   out_3960443324223161311[43] = 0;
   out_3960443324223161311[44] = 0;
   out_3960443324223161311[45] = 0;
   out_3960443324223161311[46] = 0;
   out_3960443324223161311[47] = 0;
   out_3960443324223161311[48] = 1.0;
   out_3960443324223161311[49] = 0;
   out_3960443324223161311[50] = 0;
   out_3960443324223161311[51] = 0;
   out_3960443324223161311[52] = 0;
   out_3960443324223161311[53] = 0;
   out_3960443324223161311[54] = 0;
   out_3960443324223161311[55] = 0;
   out_3960443324223161311[56] = 0;
   out_3960443324223161311[57] = 0;
   out_3960443324223161311[58] = 0;
   out_3960443324223161311[59] = 0;
   out_3960443324223161311[60] = 1.0;
   out_3960443324223161311[61] = 0;
   out_3960443324223161311[62] = 0;
   out_3960443324223161311[63] = 0;
   out_3960443324223161311[64] = 0;
   out_3960443324223161311[65] = 0;
   out_3960443324223161311[66] = 0;
   out_3960443324223161311[67] = 0;
   out_3960443324223161311[68] = 0;
   out_3960443324223161311[69] = 0;
   out_3960443324223161311[70] = 0;
   out_3960443324223161311[71] = 0;
   out_3960443324223161311[72] = 1.0;
   out_3960443324223161311[73] = 0;
   out_3960443324223161311[74] = 0;
   out_3960443324223161311[75] = 0;
   out_3960443324223161311[76] = 0;
   out_3960443324223161311[77] = 0;
   out_3960443324223161311[78] = 0;
   out_3960443324223161311[79] = 0;
   out_3960443324223161311[80] = 0;
   out_3960443324223161311[81] = 0;
   out_3960443324223161311[82] = 0;
   out_3960443324223161311[83] = 0;
   out_3960443324223161311[84] = 1.0;
   out_3960443324223161311[85] = 0;
   out_3960443324223161311[86] = 0;
   out_3960443324223161311[87] = 0;
   out_3960443324223161311[88] = 0;
   out_3960443324223161311[89] = 0;
   out_3960443324223161311[90] = 0;
   out_3960443324223161311[91] = 0;
   out_3960443324223161311[92] = 0;
   out_3960443324223161311[93] = 0;
   out_3960443324223161311[94] = 0;
   out_3960443324223161311[95] = 0;
   out_3960443324223161311[96] = 1.0;
   out_3960443324223161311[97] = 0;
   out_3960443324223161311[98] = 0;
   out_3960443324223161311[99] = 0;
   out_3960443324223161311[100] = 0;
   out_3960443324223161311[101] = 0;
   out_3960443324223161311[102] = 0;
   out_3960443324223161311[103] = 0;
   out_3960443324223161311[104] = 0;
   out_3960443324223161311[105] = 0;
   out_3960443324223161311[106] = 0;
   out_3960443324223161311[107] = 0;
   out_3960443324223161311[108] = 1.0;
   out_3960443324223161311[109] = 0;
   out_3960443324223161311[110] = 0;
   out_3960443324223161311[111] = 0;
   out_3960443324223161311[112] = 0;
   out_3960443324223161311[113] = 0;
   out_3960443324223161311[114] = 0;
   out_3960443324223161311[115] = 0;
   out_3960443324223161311[116] = 0;
   out_3960443324223161311[117] = 0;
   out_3960443324223161311[118] = 0;
   out_3960443324223161311[119] = 0;
   out_3960443324223161311[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_2368823903673001083) {
   out_2368823903673001083[0] = dt*state[3] + state[0];
   out_2368823903673001083[1] = dt*state[4] + state[1];
   out_2368823903673001083[2] = dt*state[5] + state[2];
   out_2368823903673001083[3] = state[3];
   out_2368823903673001083[4] = state[4];
   out_2368823903673001083[5] = state[5];
   out_2368823903673001083[6] = dt*state[7] + state[6];
   out_2368823903673001083[7] = dt*state[8] + state[7];
   out_2368823903673001083[8] = state[8];
   out_2368823903673001083[9] = state[9];
   out_2368823903673001083[10] = state[10];
}
void F_fun(double *state, double dt, double *out_6411103201368190584) {
   out_6411103201368190584[0] = 1;
   out_6411103201368190584[1] = 0;
   out_6411103201368190584[2] = 0;
   out_6411103201368190584[3] = dt;
   out_6411103201368190584[4] = 0;
   out_6411103201368190584[5] = 0;
   out_6411103201368190584[6] = 0;
   out_6411103201368190584[7] = 0;
   out_6411103201368190584[8] = 0;
   out_6411103201368190584[9] = 0;
   out_6411103201368190584[10] = 0;
   out_6411103201368190584[11] = 0;
   out_6411103201368190584[12] = 1;
   out_6411103201368190584[13] = 0;
   out_6411103201368190584[14] = 0;
   out_6411103201368190584[15] = dt;
   out_6411103201368190584[16] = 0;
   out_6411103201368190584[17] = 0;
   out_6411103201368190584[18] = 0;
   out_6411103201368190584[19] = 0;
   out_6411103201368190584[20] = 0;
   out_6411103201368190584[21] = 0;
   out_6411103201368190584[22] = 0;
   out_6411103201368190584[23] = 0;
   out_6411103201368190584[24] = 1;
   out_6411103201368190584[25] = 0;
   out_6411103201368190584[26] = 0;
   out_6411103201368190584[27] = dt;
   out_6411103201368190584[28] = 0;
   out_6411103201368190584[29] = 0;
   out_6411103201368190584[30] = 0;
   out_6411103201368190584[31] = 0;
   out_6411103201368190584[32] = 0;
   out_6411103201368190584[33] = 0;
   out_6411103201368190584[34] = 0;
   out_6411103201368190584[35] = 0;
   out_6411103201368190584[36] = 1;
   out_6411103201368190584[37] = 0;
   out_6411103201368190584[38] = 0;
   out_6411103201368190584[39] = 0;
   out_6411103201368190584[40] = 0;
   out_6411103201368190584[41] = 0;
   out_6411103201368190584[42] = 0;
   out_6411103201368190584[43] = 0;
   out_6411103201368190584[44] = 0;
   out_6411103201368190584[45] = 0;
   out_6411103201368190584[46] = 0;
   out_6411103201368190584[47] = 0;
   out_6411103201368190584[48] = 1;
   out_6411103201368190584[49] = 0;
   out_6411103201368190584[50] = 0;
   out_6411103201368190584[51] = 0;
   out_6411103201368190584[52] = 0;
   out_6411103201368190584[53] = 0;
   out_6411103201368190584[54] = 0;
   out_6411103201368190584[55] = 0;
   out_6411103201368190584[56] = 0;
   out_6411103201368190584[57] = 0;
   out_6411103201368190584[58] = 0;
   out_6411103201368190584[59] = 0;
   out_6411103201368190584[60] = 1;
   out_6411103201368190584[61] = 0;
   out_6411103201368190584[62] = 0;
   out_6411103201368190584[63] = 0;
   out_6411103201368190584[64] = 0;
   out_6411103201368190584[65] = 0;
   out_6411103201368190584[66] = 0;
   out_6411103201368190584[67] = 0;
   out_6411103201368190584[68] = 0;
   out_6411103201368190584[69] = 0;
   out_6411103201368190584[70] = 0;
   out_6411103201368190584[71] = 0;
   out_6411103201368190584[72] = 1;
   out_6411103201368190584[73] = dt;
   out_6411103201368190584[74] = 0;
   out_6411103201368190584[75] = 0;
   out_6411103201368190584[76] = 0;
   out_6411103201368190584[77] = 0;
   out_6411103201368190584[78] = 0;
   out_6411103201368190584[79] = 0;
   out_6411103201368190584[80] = 0;
   out_6411103201368190584[81] = 0;
   out_6411103201368190584[82] = 0;
   out_6411103201368190584[83] = 0;
   out_6411103201368190584[84] = 1;
   out_6411103201368190584[85] = dt;
   out_6411103201368190584[86] = 0;
   out_6411103201368190584[87] = 0;
   out_6411103201368190584[88] = 0;
   out_6411103201368190584[89] = 0;
   out_6411103201368190584[90] = 0;
   out_6411103201368190584[91] = 0;
   out_6411103201368190584[92] = 0;
   out_6411103201368190584[93] = 0;
   out_6411103201368190584[94] = 0;
   out_6411103201368190584[95] = 0;
   out_6411103201368190584[96] = 1;
   out_6411103201368190584[97] = 0;
   out_6411103201368190584[98] = 0;
   out_6411103201368190584[99] = 0;
   out_6411103201368190584[100] = 0;
   out_6411103201368190584[101] = 0;
   out_6411103201368190584[102] = 0;
   out_6411103201368190584[103] = 0;
   out_6411103201368190584[104] = 0;
   out_6411103201368190584[105] = 0;
   out_6411103201368190584[106] = 0;
   out_6411103201368190584[107] = 0;
   out_6411103201368190584[108] = 1;
   out_6411103201368190584[109] = 0;
   out_6411103201368190584[110] = 0;
   out_6411103201368190584[111] = 0;
   out_6411103201368190584[112] = 0;
   out_6411103201368190584[113] = 0;
   out_6411103201368190584[114] = 0;
   out_6411103201368190584[115] = 0;
   out_6411103201368190584[116] = 0;
   out_6411103201368190584[117] = 0;
   out_6411103201368190584[118] = 0;
   out_6411103201368190584[119] = 0;
   out_6411103201368190584[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_5604522824003792303) {
   out_5604522824003792303[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_7299022808000463331) {
   out_7299022808000463331[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7299022808000463331[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7299022808000463331[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7299022808000463331[3] = 0;
   out_7299022808000463331[4] = 0;
   out_7299022808000463331[5] = 0;
   out_7299022808000463331[6] = 1;
   out_7299022808000463331[7] = 0;
   out_7299022808000463331[8] = 0;
   out_7299022808000463331[9] = 0;
   out_7299022808000463331[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_4199610910471093309) {
   out_4199610910471093309[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_6323319594045795058) {
   out_6323319594045795058[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6323319594045795058[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6323319594045795058[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6323319594045795058[3] = 0;
   out_6323319594045795058[4] = 0;
   out_6323319594045795058[5] = 0;
   out_6323319594045795058[6] = 1;
   out_6323319594045795058[7] = 0;
   out_6323319594045795058[8] = 0;
   out_6323319594045795058[9] = 1;
   out_6323319594045795058[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_9139131007069815115) {
   out_9139131007069815115[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_8858933048253277504) {
   out_8858933048253277504[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8858933048253277504[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8858933048253277504[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8858933048253277504[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8858933048253277504[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8858933048253277504[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8858933048253277504[6] = 0;
   out_8858933048253277504[7] = 1;
   out_8858933048253277504[8] = 0;
   out_8858933048253277504[9] = 0;
   out_8858933048253277504[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_9139131007069815115) {
   out_9139131007069815115[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_8858933048253277504) {
   out_8858933048253277504[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8858933048253277504[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8858933048253277504[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8858933048253277504[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8858933048253277504[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8858933048253277504[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8858933048253277504[6] = 0;
   out_8858933048253277504[7] = 1;
   out_8858933048253277504[8] = 0;
   out_8858933048253277504[9] = 0;
   out_8858933048253277504[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_5443830885228648884) {
  err_fun(nom_x, delta_x, out_5443830885228648884);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_2005231189022139872) {
  inv_err_fun(nom_x, true_x, out_2005231189022139872);
}
void gnss_H_mod_fun(double *state, double *out_3960443324223161311) {
  H_mod_fun(state, out_3960443324223161311);
}
void gnss_f_fun(double *state, double dt, double *out_2368823903673001083) {
  f_fun(state,  dt, out_2368823903673001083);
}
void gnss_F_fun(double *state, double dt, double *out_6411103201368190584) {
  F_fun(state,  dt, out_6411103201368190584);
}
void gnss_h_6(double *state, double *sat_pos, double *out_5604522824003792303) {
  h_6(state, sat_pos, out_5604522824003792303);
}
void gnss_H_6(double *state, double *sat_pos, double *out_7299022808000463331) {
  H_6(state, sat_pos, out_7299022808000463331);
}
void gnss_h_20(double *state, double *sat_pos, double *out_4199610910471093309) {
  h_20(state, sat_pos, out_4199610910471093309);
}
void gnss_H_20(double *state, double *sat_pos, double *out_6323319594045795058) {
  H_20(state, sat_pos, out_6323319594045795058);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_9139131007069815115) {
  h_7(state, sat_pos_vel, out_9139131007069815115);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8858933048253277504) {
  H_7(state, sat_pos_vel, out_8858933048253277504);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_9139131007069815115) {
  h_21(state, sat_pos_vel, out_9139131007069815115);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8858933048253277504) {
  H_21(state, sat_pos_vel, out_8858933048253277504);
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
