#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_9189878205404737176);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3224947780549043673);
void gnss_H_mod_fun(double *state, double *out_870993508719181486);
void gnss_f_fun(double *state, double dt, double *out_2024835571204853601);
void gnss_F_fun(double *state, double dt, double *out_1114880998460348898);
void gnss_h_6(double *state, double *sat_pos, double *out_3645878914521899003);
void gnss_H_6(double *state, double *sat_pos, double *out_8291397417584581551);
void gnss_h_20(double *state, double *sat_pos, double *out_5617236136998162351);
void gnss_H_20(double *state, double *sat_pos, double *out_5699742714549703355);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_8085500609040575343);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_9116371559383232951);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_8085500609040575343);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_9116371559383232951);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}