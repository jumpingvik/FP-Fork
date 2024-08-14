#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_567201996152504854) {
   out_567201996152504854[0] = delta_x[0] + nom_x[0];
   out_567201996152504854[1] = delta_x[1] + nom_x[1];
   out_567201996152504854[2] = delta_x[2] + nom_x[2];
   out_567201996152504854[3] = delta_x[3] + nom_x[3];
   out_567201996152504854[4] = delta_x[4] + nom_x[4];
   out_567201996152504854[5] = delta_x[5] + nom_x[5];
   out_567201996152504854[6] = delta_x[6] + nom_x[6];
   out_567201996152504854[7] = delta_x[7] + nom_x[7];
   out_567201996152504854[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1340978382637353211) {
   out_1340978382637353211[0] = -nom_x[0] + true_x[0];
   out_1340978382637353211[1] = -nom_x[1] + true_x[1];
   out_1340978382637353211[2] = -nom_x[2] + true_x[2];
   out_1340978382637353211[3] = -nom_x[3] + true_x[3];
   out_1340978382637353211[4] = -nom_x[4] + true_x[4];
   out_1340978382637353211[5] = -nom_x[5] + true_x[5];
   out_1340978382637353211[6] = -nom_x[6] + true_x[6];
   out_1340978382637353211[7] = -nom_x[7] + true_x[7];
   out_1340978382637353211[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6250763146856361285) {
   out_6250763146856361285[0] = 1.0;
   out_6250763146856361285[1] = 0;
   out_6250763146856361285[2] = 0;
   out_6250763146856361285[3] = 0;
   out_6250763146856361285[4] = 0;
   out_6250763146856361285[5] = 0;
   out_6250763146856361285[6] = 0;
   out_6250763146856361285[7] = 0;
   out_6250763146856361285[8] = 0;
   out_6250763146856361285[9] = 0;
   out_6250763146856361285[10] = 1.0;
   out_6250763146856361285[11] = 0;
   out_6250763146856361285[12] = 0;
   out_6250763146856361285[13] = 0;
   out_6250763146856361285[14] = 0;
   out_6250763146856361285[15] = 0;
   out_6250763146856361285[16] = 0;
   out_6250763146856361285[17] = 0;
   out_6250763146856361285[18] = 0;
   out_6250763146856361285[19] = 0;
   out_6250763146856361285[20] = 1.0;
   out_6250763146856361285[21] = 0;
   out_6250763146856361285[22] = 0;
   out_6250763146856361285[23] = 0;
   out_6250763146856361285[24] = 0;
   out_6250763146856361285[25] = 0;
   out_6250763146856361285[26] = 0;
   out_6250763146856361285[27] = 0;
   out_6250763146856361285[28] = 0;
   out_6250763146856361285[29] = 0;
   out_6250763146856361285[30] = 1.0;
   out_6250763146856361285[31] = 0;
   out_6250763146856361285[32] = 0;
   out_6250763146856361285[33] = 0;
   out_6250763146856361285[34] = 0;
   out_6250763146856361285[35] = 0;
   out_6250763146856361285[36] = 0;
   out_6250763146856361285[37] = 0;
   out_6250763146856361285[38] = 0;
   out_6250763146856361285[39] = 0;
   out_6250763146856361285[40] = 1.0;
   out_6250763146856361285[41] = 0;
   out_6250763146856361285[42] = 0;
   out_6250763146856361285[43] = 0;
   out_6250763146856361285[44] = 0;
   out_6250763146856361285[45] = 0;
   out_6250763146856361285[46] = 0;
   out_6250763146856361285[47] = 0;
   out_6250763146856361285[48] = 0;
   out_6250763146856361285[49] = 0;
   out_6250763146856361285[50] = 1.0;
   out_6250763146856361285[51] = 0;
   out_6250763146856361285[52] = 0;
   out_6250763146856361285[53] = 0;
   out_6250763146856361285[54] = 0;
   out_6250763146856361285[55] = 0;
   out_6250763146856361285[56] = 0;
   out_6250763146856361285[57] = 0;
   out_6250763146856361285[58] = 0;
   out_6250763146856361285[59] = 0;
   out_6250763146856361285[60] = 1.0;
   out_6250763146856361285[61] = 0;
   out_6250763146856361285[62] = 0;
   out_6250763146856361285[63] = 0;
   out_6250763146856361285[64] = 0;
   out_6250763146856361285[65] = 0;
   out_6250763146856361285[66] = 0;
   out_6250763146856361285[67] = 0;
   out_6250763146856361285[68] = 0;
   out_6250763146856361285[69] = 0;
   out_6250763146856361285[70] = 1.0;
   out_6250763146856361285[71] = 0;
   out_6250763146856361285[72] = 0;
   out_6250763146856361285[73] = 0;
   out_6250763146856361285[74] = 0;
   out_6250763146856361285[75] = 0;
   out_6250763146856361285[76] = 0;
   out_6250763146856361285[77] = 0;
   out_6250763146856361285[78] = 0;
   out_6250763146856361285[79] = 0;
   out_6250763146856361285[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_934165908068110690) {
   out_934165908068110690[0] = state[0];
   out_934165908068110690[1] = state[1];
   out_934165908068110690[2] = state[2];
   out_934165908068110690[3] = state[3];
   out_934165908068110690[4] = state[4];
   out_934165908068110690[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_934165908068110690[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_934165908068110690[7] = state[7];
   out_934165908068110690[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2317510077348164005) {
   out_2317510077348164005[0] = 1;
   out_2317510077348164005[1] = 0;
   out_2317510077348164005[2] = 0;
   out_2317510077348164005[3] = 0;
   out_2317510077348164005[4] = 0;
   out_2317510077348164005[5] = 0;
   out_2317510077348164005[6] = 0;
   out_2317510077348164005[7] = 0;
   out_2317510077348164005[8] = 0;
   out_2317510077348164005[9] = 0;
   out_2317510077348164005[10] = 1;
   out_2317510077348164005[11] = 0;
   out_2317510077348164005[12] = 0;
   out_2317510077348164005[13] = 0;
   out_2317510077348164005[14] = 0;
   out_2317510077348164005[15] = 0;
   out_2317510077348164005[16] = 0;
   out_2317510077348164005[17] = 0;
   out_2317510077348164005[18] = 0;
   out_2317510077348164005[19] = 0;
   out_2317510077348164005[20] = 1;
   out_2317510077348164005[21] = 0;
   out_2317510077348164005[22] = 0;
   out_2317510077348164005[23] = 0;
   out_2317510077348164005[24] = 0;
   out_2317510077348164005[25] = 0;
   out_2317510077348164005[26] = 0;
   out_2317510077348164005[27] = 0;
   out_2317510077348164005[28] = 0;
   out_2317510077348164005[29] = 0;
   out_2317510077348164005[30] = 1;
   out_2317510077348164005[31] = 0;
   out_2317510077348164005[32] = 0;
   out_2317510077348164005[33] = 0;
   out_2317510077348164005[34] = 0;
   out_2317510077348164005[35] = 0;
   out_2317510077348164005[36] = 0;
   out_2317510077348164005[37] = 0;
   out_2317510077348164005[38] = 0;
   out_2317510077348164005[39] = 0;
   out_2317510077348164005[40] = 1;
   out_2317510077348164005[41] = 0;
   out_2317510077348164005[42] = 0;
   out_2317510077348164005[43] = 0;
   out_2317510077348164005[44] = 0;
   out_2317510077348164005[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2317510077348164005[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2317510077348164005[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2317510077348164005[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2317510077348164005[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2317510077348164005[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2317510077348164005[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2317510077348164005[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2317510077348164005[53] = -9.8000000000000007*dt;
   out_2317510077348164005[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2317510077348164005[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2317510077348164005[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2317510077348164005[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2317510077348164005[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2317510077348164005[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2317510077348164005[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2317510077348164005[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2317510077348164005[62] = 0;
   out_2317510077348164005[63] = 0;
   out_2317510077348164005[64] = 0;
   out_2317510077348164005[65] = 0;
   out_2317510077348164005[66] = 0;
   out_2317510077348164005[67] = 0;
   out_2317510077348164005[68] = 0;
   out_2317510077348164005[69] = 0;
   out_2317510077348164005[70] = 1;
   out_2317510077348164005[71] = 0;
   out_2317510077348164005[72] = 0;
   out_2317510077348164005[73] = 0;
   out_2317510077348164005[74] = 0;
   out_2317510077348164005[75] = 0;
   out_2317510077348164005[76] = 0;
   out_2317510077348164005[77] = 0;
   out_2317510077348164005[78] = 0;
   out_2317510077348164005[79] = 0;
   out_2317510077348164005[80] = 1;
}
void h_25(double *state, double *unused, double *out_96969927905659930) {
   out_96969927905659930[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5148880695130896176) {
   out_5148880695130896176[0] = 0;
   out_5148880695130896176[1] = 0;
   out_5148880695130896176[2] = 0;
   out_5148880695130896176[3] = 0;
   out_5148880695130896176[4] = 0;
   out_5148880695130896176[5] = 0;
   out_5148880695130896176[6] = 1;
   out_5148880695130896176[7] = 0;
   out_5148880695130896176[8] = 0;
}
void h_24(double *state, double *unused, double *out_8757236806031252516) {
   out_8757236806031252516[0] = state[4];
   out_8757236806031252516[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7374588479109764738) {
   out_7374588479109764738[0] = 0;
   out_7374588479109764738[1] = 0;
   out_7374588479109764738[2] = 0;
   out_7374588479109764738[3] = 0;
   out_7374588479109764738[4] = 1;
   out_7374588479109764738[5] = 0;
   out_7374588479109764738[6] = 0;
   out_7374588479109764738[7] = 0;
   out_7374588479109764738[8] = 0;
   out_7374588479109764738[9] = 0;
   out_7374588479109764738[10] = 0;
   out_7374588479109764738[11] = 0;
   out_7374588479109764738[12] = 0;
   out_7374588479109764738[13] = 0;
   out_7374588479109764738[14] = 1;
   out_7374588479109764738[15] = 0;
   out_7374588479109764738[16] = 0;
   out_7374588479109764738[17] = 0;
}
void h_30(double *state, double *unused, double *out_8736847404624201814) {
   out_8736847404624201814[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6381173037087038685) {
   out_6381173037087038685[0] = 0;
   out_6381173037087038685[1] = 0;
   out_6381173037087038685[2] = 0;
   out_6381173037087038685[3] = 0;
   out_6381173037087038685[4] = 1;
   out_6381173037087038685[5] = 0;
   out_6381173037087038685[6] = 0;
   out_6381173037087038685[7] = 0;
   out_6381173037087038685[8] = 0;
}
void h_26(double *state, double *unused, double *out_4708223401424504157) {
   out_4708223401424504157[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1407377376256839952) {
   out_1407377376256839952[0] = 0;
   out_1407377376256839952[1] = 0;
   out_1407377376256839952[2] = 0;
   out_1407377376256839952[3] = 0;
   out_1407377376256839952[4] = 0;
   out_1407377376256839952[5] = 0;
   out_1407377376256839952[6] = 0;
   out_1407377376256839952[7] = 1;
   out_1407377376256839952[8] = 0;
}
void h_27(double *state, double *unused, double *out_2909522031175513313) {
   out_2909522031175513313[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8555936348887463596) {
   out_8555936348887463596[0] = 0;
   out_8555936348887463596[1] = 0;
   out_8555936348887463596[2] = 0;
   out_8555936348887463596[3] = 1;
   out_8555936348887463596[4] = 0;
   out_8555936348887463596[5] = 0;
   out_8555936348887463596[6] = 0;
   out_8555936348887463596[7] = 0;
   out_8555936348887463596[8] = 0;
}
void h_29(double *state, double *unused, double *out_8542152868009208065) {
   out_8542152868009208065[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5870941692772646501) {
   out_5870941692772646501[0] = 0;
   out_5870941692772646501[1] = 1;
   out_5870941692772646501[2] = 0;
   out_5870941692772646501[3] = 0;
   out_5870941692772646501[4] = 0;
   out_5870941692772646501[5] = 0;
   out_5870941692772646501[6] = 0;
   out_5870941692772646501[7] = 0;
   out_5870941692772646501[8] = 0;
}
void h_28(double *state, double *unused, double *out_5886787433090483827) {
   out_5886787433090483827[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7493403363867374541) {
   out_7493403363867374541[0] = 1;
   out_7493403363867374541[1] = 0;
   out_7493403363867374541[2] = 0;
   out_7493403363867374541[3] = 0;
   out_7493403363867374541[4] = 0;
   out_7493403363867374541[5] = 0;
   out_7493403363867374541[6] = 0;
   out_7493403363867374541[7] = 0;
   out_7493403363867374541[8] = 0;
}
void h_31(double *state, double *unused, double *out_372163990190165819) {
   out_372163990190165819[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5179526657007856604) {
   out_5179526657007856604[0] = 0;
   out_5179526657007856604[1] = 0;
   out_5179526657007856604[2] = 0;
   out_5179526657007856604[3] = 0;
   out_5179526657007856604[4] = 0;
   out_5179526657007856604[5] = 0;
   out_5179526657007856604[6] = 0;
   out_5179526657007856604[7] = 0;
   out_5179526657007856604[8] = 1;
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

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_567201996152504854) {
  err_fun(nom_x, delta_x, out_567201996152504854);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1340978382637353211) {
  inv_err_fun(nom_x, true_x, out_1340978382637353211);
}
void car_H_mod_fun(double *state, double *out_6250763146856361285) {
  H_mod_fun(state, out_6250763146856361285);
}
void car_f_fun(double *state, double dt, double *out_934165908068110690) {
  f_fun(state,  dt, out_934165908068110690);
}
void car_F_fun(double *state, double dt, double *out_2317510077348164005) {
  F_fun(state,  dt, out_2317510077348164005);
}
void car_h_25(double *state, double *unused, double *out_96969927905659930) {
  h_25(state, unused, out_96969927905659930);
}
void car_H_25(double *state, double *unused, double *out_5148880695130896176) {
  H_25(state, unused, out_5148880695130896176);
}
void car_h_24(double *state, double *unused, double *out_8757236806031252516) {
  h_24(state, unused, out_8757236806031252516);
}
void car_H_24(double *state, double *unused, double *out_7374588479109764738) {
  H_24(state, unused, out_7374588479109764738);
}
void car_h_30(double *state, double *unused, double *out_8736847404624201814) {
  h_30(state, unused, out_8736847404624201814);
}
void car_H_30(double *state, double *unused, double *out_6381173037087038685) {
  H_30(state, unused, out_6381173037087038685);
}
void car_h_26(double *state, double *unused, double *out_4708223401424504157) {
  h_26(state, unused, out_4708223401424504157);
}
void car_H_26(double *state, double *unused, double *out_1407377376256839952) {
  H_26(state, unused, out_1407377376256839952);
}
void car_h_27(double *state, double *unused, double *out_2909522031175513313) {
  h_27(state, unused, out_2909522031175513313);
}
void car_H_27(double *state, double *unused, double *out_8555936348887463596) {
  H_27(state, unused, out_8555936348887463596);
}
void car_h_29(double *state, double *unused, double *out_8542152868009208065) {
  h_29(state, unused, out_8542152868009208065);
}
void car_H_29(double *state, double *unused, double *out_5870941692772646501) {
  H_29(state, unused, out_5870941692772646501);
}
void car_h_28(double *state, double *unused, double *out_5886787433090483827) {
  h_28(state, unused, out_5886787433090483827);
}
void car_H_28(double *state, double *unused, double *out_7493403363867374541) {
  H_28(state, unused, out_7493403363867374541);
}
void car_h_31(double *state, double *unused, double *out_372163990190165819) {
  h_31(state, unused, out_372163990190165819);
}
void car_H_31(double *state, double *unused, double *out_5179526657007856604) {
  H_31(state, unused, out_5179526657007856604);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
