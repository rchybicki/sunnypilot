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
void car_err_fun(double *nom_x, double *delta_x, double *out_4095765816768978677);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4903182091448254315);
void car_H_mod_fun(double *state, double *out_3077754926992826597);
void car_f_fun(double *state, double dt, double *out_5980213151718331081);
void car_F_fun(double *state, double dt, double *out_325952735880629537);
void car_h_25(double *state, double *unused, double *out_7694641462849261130);
void car_H_25(double *state, double *unused, double *out_8009217059877103452);
void car_h_24(double *state, double *unused, double *out_5269556662559138097);
void car_H_24(double *state, double *unused, double *out_2376216019280655464);
void car_h_30(double *state, double *unused, double *out_7419447400564755241);
void car_H_30(double *state, double *unused, double *out_3481520729749495254);
void car_h_26(double *state, double *unused, double *out_4815042079756057590);
void car_H_26(double *state, double *unused, double *out_4267713741003047228);
void car_h_27(double *state, double *unused, double *out_2468734247013936183);
void car_H_27(double *state, double *unused, double *out_5705114800933438471);
void car_h_29(double *state, double *unused, double *out_1168715281240086167);
void car_H_29(double *state, double *unused, double *out_3991752074063887438);
void car_h_28(double *state, double *unused, double *out_5073139567822633834);
void car_H_28(double *state, double *unused, double *out_1090646943005643136);
void car_h_31(double *state, double *unused, double *out_7114653082606268136);
void car_H_31(double *state, double *unused, double *out_3641505638769695752);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}