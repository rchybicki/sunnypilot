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
void live_H(double *in_vec, double *out_2823155123848402991);
void live_err_fun(double *nom_x, double *delta_x, double *out_7226446291374266530);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_3650096757331326943);
void live_H_mod_fun(double *state, double *out_1017595647546641607);
void live_f_fun(double *state, double dt, double *out_1572252167703847428);
void live_F_fun(double *state, double dt, double *out_4661722811712315816);
void live_h_4(double *state, double *unused, double *out_301170939575007164);
void live_H_4(double *state, double *unused, double *out_4386249715678046629);
void live_h_9(double *state, double *unused, double *out_8101849099089557939);
void live_H_9(double *state, double *unused, double *out_6773275422767057517);
void live_h_10(double *state, double *unused, double *out_2439254096414030147);
void live_H_10(double *state, double *unused, double *out_347608672828194809);
void live_h_12(double *state, double *unused, double *out_4212404837284500669);
void live_H_12(double *state, double *unused, double *out_9041037949999543192);
void live_h_35(double *state, double *unused, double *out_1097865055430795382);
void live_H_35(double *state, double *unused, double *out_7752911773050654005);
void live_h_32(double *state, double *unused, double *out_6792077341972714598);
void live_H_32(double *state, double *unused, double *out_7884076557991394474);
void live_h_13(double *state, double *unused, double *out_625194612726844095);
void live_H_13(double *state, double *unused, double *out_4592999320050799082);
void live_h_14(double *state, double *unused, double *out_8101849099089557939);
void live_H_14(double *state, double *unused, double *out_6773275422767057517);
void live_h_33(double *state, double *unused, double *out_655443362248511566);
void live_H_33(double *state, double *unused, double *out_7543275296020040007);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}