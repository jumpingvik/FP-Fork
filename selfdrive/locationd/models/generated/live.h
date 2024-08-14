#pragma once
#include "rednose/helpers/ekf.h"
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
void live_H(double *in_vec, double *out_7420092385227711648);
void live_err_fun(double *nom_x, double *delta_x, double *out_7384372108169230752);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_7123812327112091225);
void live_H_mod_fun(double *state, double *out_5900653292609602750);
void live_f_fun(double *state, double dt, double *out_8270287573456865384);
void live_F_fun(double *state, double dt, double *out_468430115656579259);
void live_h_4(double *state, double *unused, double *out_5343601880999944077);
void live_H_4(double *state, double *unused, double *out_8690355848183064223);
void live_h_9(double *state, double *unused, double *out_8563674156499137782);
void live_H_9(double *state, double *unused, double *out_2469169290262039923);
void live_h_10(double *state, double *unused, double *out_1058046278782088996);
void live_H_10(double *state, double *unused, double *out_4703589435547461641);
void live_h_12(double *state, double *unused, double *out_7885917772179439573);
void live_H_12(double *state, double *unused, double *out_2309097471140331227);
void live_h_35(double *state, double *unused, double *out_7952497595820258194);
void live_H_35(double *state, double *unused, double *out_656303120480976808);
void live_h_32(double *state, double *unused, double *out_8117811263559019773);
void live_H_32(double *state, double *unused, double *out_4742038808577899908);
void live_h_13(double *state, double *unused, double *out_3379484723930244350);
void live_H_13(double *state, double *unused, double *out_2869722698847849975);
void live_h_14(double *state, double *unused, double *out_8563674156499137782);
void live_H_14(double *state, double *unused, double *out_2469169290262039923);
void live_h_33(double *state, double *unused, double *out_3903239874447069051);
void live_H_33(double *state, double *unused, double *out_3806860125119834412);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}