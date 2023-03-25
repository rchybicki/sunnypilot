#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_3017386499956164812);
void live_err_fun(double *nom_x, double *delta_x, double *out_1361250254903338847);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_171595851375091826);
void live_H_mod_fun(double *state, double *out_859259826444489067);
void live_f_fun(double *state, double dt, double *out_3099034127879974829);
void live_F_fun(double *state, double dt, double *out_3444937982289135225);
void live_h_4(double *state, double *unused, double *out_4711157112635749769);
void live_H_4(double *state, double *unused, double *out_9137183678080118037);
void live_h_9(double *state, double *unused, double *out_6060102139174664302);
void live_H_9(double *state, double *unused, double *out_9068370748999842934);
void live_h_10(double *state, double *unused, double *out_3769887592000088354);
void live_H_10(double *state, double *unused, double *out_6994148659333627741);
void live_h_12(double *state, double *unused, double *out_4657937196283319724);
void live_H_12(double *state, double *unused, double *out_4290103987597471784);
void live_h_35(double *state, double *unused, double *out_1808108796454482122);
void live_H_35(double *state, double *unused, double *out_5942898338256826203);
void live_h_32(double *state, double *unused, double *out_2251136288058511181);
void live_H_32(double *state, double *unused, double *out_6031686465560362514);
void live_h_13(double *state, double *unused, double *out_5570011810361195730);
void live_H_13(double *state, double *unused, double *out_3779641119465792009);
void live_h_14(double *state, double *unused, double *out_6060102139174664302);
void live_H_14(double *state, double *unused, double *out_9068370748999842934);
void live_h_33(double *state, double *unused, double *out_5381142625228283606);
void live_H_33(double *state, double *unused, double *out_2792341333617968599);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}