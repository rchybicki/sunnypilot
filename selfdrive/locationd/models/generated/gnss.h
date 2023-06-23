#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_5443830885228648884);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_2005231189022139872);
void gnss_H_mod_fun(double *state, double *out_3960443324223161311);
void gnss_f_fun(double *state, double dt, double *out_2368823903673001083);
void gnss_F_fun(double *state, double dt, double *out_6411103201368190584);
void gnss_h_6(double *state, double *sat_pos, double *out_5604522824003792303);
void gnss_H_6(double *state, double *sat_pos, double *out_7299022808000463331);
void gnss_h_20(double *state, double *sat_pos, double *out_4199610910471093309);
void gnss_H_20(double *state, double *sat_pos, double *out_6323319594045795058);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_9139131007069815115);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8858933048253277504);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_9139131007069815115);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8858933048253277504);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}