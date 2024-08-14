#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_567201996152504854);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1340978382637353211);
void car_H_mod_fun(double *state, double *out_6250763146856361285);
void car_f_fun(double *state, double dt, double *out_934165908068110690);
void car_F_fun(double *state, double dt, double *out_2317510077348164005);
void car_h_25(double *state, double *unused, double *out_96969927905659930);
void car_H_25(double *state, double *unused, double *out_5148880695130896176);
void car_h_24(double *state, double *unused, double *out_8757236806031252516);
void car_H_24(double *state, double *unused, double *out_7374588479109764738);
void car_h_30(double *state, double *unused, double *out_8736847404624201814);
void car_H_30(double *state, double *unused, double *out_6381173037087038685);
void car_h_26(double *state, double *unused, double *out_4708223401424504157);
void car_H_26(double *state, double *unused, double *out_1407377376256839952);
void car_h_27(double *state, double *unused, double *out_2909522031175513313);
void car_H_27(double *state, double *unused, double *out_8555936348887463596);
void car_h_29(double *state, double *unused, double *out_8542152868009208065);
void car_H_29(double *state, double *unused, double *out_5870941692772646501);
void car_h_28(double *state, double *unused, double *out_5886787433090483827);
void car_H_28(double *state, double *unused, double *out_7493403363867374541);
void car_h_31(double *state, double *unused, double *out_372163990190165819);
void car_H_31(double *state, double *unused, double *out_5179526657007856604);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}