#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_968838101684317911);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4310908025649758055);
void car_H_mod_fun(double *state, double *out_1396426837997155367);
void car_f_fun(double *state, double dt, double *out_4044180442470054806);
void car_F_fun(double *state, double dt, double *out_8407255564466940885);
void car_h_25(double *state, double *unused, double *out_5096407301093087859);
void car_H_25(double *state, double *unused, double *out_8334896348201987056);
void car_h_24(double *state, double *unused, double *out_3230224340948962135);
void car_H_24(double *state, double *unused, double *out_3910815045656403134);
void car_h_30(double *state, double *unused, double *out_5515532985104572826);
void car_H_30(double *state, double *unused, double *out_8464235295345227126);
void car_h_26(double *state, double *unused, double *out_8466670898888994856);
void car_H_26(double *state, double *unused, double *out_6370344406633508336);
void car_h_27(double *state, double *unused, double *out_8348663504955618112);
void car_H_27(double *state, double *unused, double *out_7807745466563899579);
void car_h_29(double *state, double *unused, double *out_2637521804505045179);
void car_H_29(double *state, double *unused, double *out_6094382739694348546);
void car_h_28(double *state, double *unused, double *out_1692423749109706851);
void car_H_28(double *state, double *unused, double *out_1011983722624817972);
void car_h_31(double *state, double *unused, double *out_1188714132811423188);
void car_H_31(double *state, double *unused, double *out_8304250386325026628);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}