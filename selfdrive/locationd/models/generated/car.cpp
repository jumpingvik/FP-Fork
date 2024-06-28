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
void err_fun(double *nom_x, double *delta_x, double *out_8674688528389785859) {
   out_8674688528389785859[0] = delta_x[0] + nom_x[0];
   out_8674688528389785859[1] = delta_x[1] + nom_x[1];
   out_8674688528389785859[2] = delta_x[2] + nom_x[2];
   out_8674688528389785859[3] = delta_x[3] + nom_x[3];
   out_8674688528389785859[4] = delta_x[4] + nom_x[4];
   out_8674688528389785859[5] = delta_x[5] + nom_x[5];
   out_8674688528389785859[6] = delta_x[6] + nom_x[6];
   out_8674688528389785859[7] = delta_x[7] + nom_x[7];
   out_8674688528389785859[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_9192190503432797144) {
   out_9192190503432797144[0] = -nom_x[0] + true_x[0];
   out_9192190503432797144[1] = -nom_x[1] + true_x[1];
   out_9192190503432797144[2] = -nom_x[2] + true_x[2];
   out_9192190503432797144[3] = -nom_x[3] + true_x[3];
   out_9192190503432797144[4] = -nom_x[4] + true_x[4];
   out_9192190503432797144[5] = -nom_x[5] + true_x[5];
   out_9192190503432797144[6] = -nom_x[6] + true_x[6];
   out_9192190503432797144[7] = -nom_x[7] + true_x[7];
   out_9192190503432797144[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7340844205401277523) {
   out_7340844205401277523[0] = 1.0;
   out_7340844205401277523[1] = 0;
   out_7340844205401277523[2] = 0;
   out_7340844205401277523[3] = 0;
   out_7340844205401277523[4] = 0;
   out_7340844205401277523[5] = 0;
   out_7340844205401277523[6] = 0;
   out_7340844205401277523[7] = 0;
   out_7340844205401277523[8] = 0;
   out_7340844205401277523[9] = 0;
   out_7340844205401277523[10] = 1.0;
   out_7340844205401277523[11] = 0;
   out_7340844205401277523[12] = 0;
   out_7340844205401277523[13] = 0;
   out_7340844205401277523[14] = 0;
   out_7340844205401277523[15] = 0;
   out_7340844205401277523[16] = 0;
   out_7340844205401277523[17] = 0;
   out_7340844205401277523[18] = 0;
   out_7340844205401277523[19] = 0;
   out_7340844205401277523[20] = 1.0;
   out_7340844205401277523[21] = 0;
   out_7340844205401277523[22] = 0;
   out_7340844205401277523[23] = 0;
   out_7340844205401277523[24] = 0;
   out_7340844205401277523[25] = 0;
   out_7340844205401277523[26] = 0;
   out_7340844205401277523[27] = 0;
   out_7340844205401277523[28] = 0;
   out_7340844205401277523[29] = 0;
   out_7340844205401277523[30] = 1.0;
   out_7340844205401277523[31] = 0;
   out_7340844205401277523[32] = 0;
   out_7340844205401277523[33] = 0;
   out_7340844205401277523[34] = 0;
   out_7340844205401277523[35] = 0;
   out_7340844205401277523[36] = 0;
   out_7340844205401277523[37] = 0;
   out_7340844205401277523[38] = 0;
   out_7340844205401277523[39] = 0;
   out_7340844205401277523[40] = 1.0;
   out_7340844205401277523[41] = 0;
   out_7340844205401277523[42] = 0;
   out_7340844205401277523[43] = 0;
   out_7340844205401277523[44] = 0;
   out_7340844205401277523[45] = 0;
   out_7340844205401277523[46] = 0;
   out_7340844205401277523[47] = 0;
   out_7340844205401277523[48] = 0;
   out_7340844205401277523[49] = 0;
   out_7340844205401277523[50] = 1.0;
   out_7340844205401277523[51] = 0;
   out_7340844205401277523[52] = 0;
   out_7340844205401277523[53] = 0;
   out_7340844205401277523[54] = 0;
   out_7340844205401277523[55] = 0;
   out_7340844205401277523[56] = 0;
   out_7340844205401277523[57] = 0;
   out_7340844205401277523[58] = 0;
   out_7340844205401277523[59] = 0;
   out_7340844205401277523[60] = 1.0;
   out_7340844205401277523[61] = 0;
   out_7340844205401277523[62] = 0;
   out_7340844205401277523[63] = 0;
   out_7340844205401277523[64] = 0;
   out_7340844205401277523[65] = 0;
   out_7340844205401277523[66] = 0;
   out_7340844205401277523[67] = 0;
   out_7340844205401277523[68] = 0;
   out_7340844205401277523[69] = 0;
   out_7340844205401277523[70] = 1.0;
   out_7340844205401277523[71] = 0;
   out_7340844205401277523[72] = 0;
   out_7340844205401277523[73] = 0;
   out_7340844205401277523[74] = 0;
   out_7340844205401277523[75] = 0;
   out_7340844205401277523[76] = 0;
   out_7340844205401277523[77] = 0;
   out_7340844205401277523[78] = 0;
   out_7340844205401277523[79] = 0;
   out_7340844205401277523[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1286214365289940422) {
   out_1286214365289940422[0] = state[0];
   out_1286214365289940422[1] = state[1];
   out_1286214365289940422[2] = state[2];
   out_1286214365289940422[3] = state[3];
   out_1286214365289940422[4] = state[4];
   out_1286214365289940422[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1286214365289940422[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1286214365289940422[7] = state[7];
   out_1286214365289940422[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8185406033884487895) {
   out_8185406033884487895[0] = 1;
   out_8185406033884487895[1] = 0;
   out_8185406033884487895[2] = 0;
   out_8185406033884487895[3] = 0;
   out_8185406033884487895[4] = 0;
   out_8185406033884487895[5] = 0;
   out_8185406033884487895[6] = 0;
   out_8185406033884487895[7] = 0;
   out_8185406033884487895[8] = 0;
   out_8185406033884487895[9] = 0;
   out_8185406033884487895[10] = 1;
   out_8185406033884487895[11] = 0;
   out_8185406033884487895[12] = 0;
   out_8185406033884487895[13] = 0;
   out_8185406033884487895[14] = 0;
   out_8185406033884487895[15] = 0;
   out_8185406033884487895[16] = 0;
   out_8185406033884487895[17] = 0;
   out_8185406033884487895[18] = 0;
   out_8185406033884487895[19] = 0;
   out_8185406033884487895[20] = 1;
   out_8185406033884487895[21] = 0;
   out_8185406033884487895[22] = 0;
   out_8185406033884487895[23] = 0;
   out_8185406033884487895[24] = 0;
   out_8185406033884487895[25] = 0;
   out_8185406033884487895[26] = 0;
   out_8185406033884487895[27] = 0;
   out_8185406033884487895[28] = 0;
   out_8185406033884487895[29] = 0;
   out_8185406033884487895[30] = 1;
   out_8185406033884487895[31] = 0;
   out_8185406033884487895[32] = 0;
   out_8185406033884487895[33] = 0;
   out_8185406033884487895[34] = 0;
   out_8185406033884487895[35] = 0;
   out_8185406033884487895[36] = 0;
   out_8185406033884487895[37] = 0;
   out_8185406033884487895[38] = 0;
   out_8185406033884487895[39] = 0;
   out_8185406033884487895[40] = 1;
   out_8185406033884487895[41] = 0;
   out_8185406033884487895[42] = 0;
   out_8185406033884487895[43] = 0;
   out_8185406033884487895[44] = 0;
   out_8185406033884487895[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8185406033884487895[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8185406033884487895[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8185406033884487895[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8185406033884487895[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8185406033884487895[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8185406033884487895[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8185406033884487895[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8185406033884487895[53] = -9.8000000000000007*dt;
   out_8185406033884487895[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8185406033884487895[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8185406033884487895[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8185406033884487895[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8185406033884487895[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8185406033884487895[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8185406033884487895[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8185406033884487895[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8185406033884487895[62] = 0;
   out_8185406033884487895[63] = 0;
   out_8185406033884487895[64] = 0;
   out_8185406033884487895[65] = 0;
   out_8185406033884487895[66] = 0;
   out_8185406033884487895[67] = 0;
   out_8185406033884487895[68] = 0;
   out_8185406033884487895[69] = 0;
   out_8185406033884487895[70] = 1;
   out_8185406033884487895[71] = 0;
   out_8185406033884487895[72] = 0;
   out_8185406033884487895[73] = 0;
   out_8185406033884487895[74] = 0;
   out_8185406033884487895[75] = 0;
   out_8185406033884487895[76] = 0;
   out_8185406033884487895[77] = 0;
   out_8185406033884487895[78] = 0;
   out_8185406033884487895[79] = 0;
   out_8185406033884487895[80] = 1;
}
void h_25(double *state, double *unused, double *out_8560173101671562352) {
   out_8560173101671562352[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1192887874864391825) {
   out_1192887874864391825[0] = 0;
   out_1192887874864391825[1] = 0;
   out_1192887874864391825[2] = 0;
   out_1192887874864391825[3] = 0;
   out_1192887874864391825[4] = 0;
   out_1192887874864391825[5] = 0;
   out_1192887874864391825[6] = 1;
   out_1192887874864391825[7] = 0;
   out_1192887874864391825[8] = 0;
}
void h_24(double *state, double *unused, double *out_4400767935302587282) {
   out_4400767935302587282[0] = state[4];
   out_4400767935302587282[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3365537473869891391) {
   out_3365537473869891391[0] = 0;
   out_3365537473869891391[1] = 0;
   out_3365537473869891391[2] = 0;
   out_3365537473869891391[3] = 0;
   out_3365537473869891391[4] = 1;
   out_3365537473869891391[5] = 0;
   out_3365537473869891391[6] = 0;
   out_3365537473869891391[7] = 0;
   out_3365537473869891391[8] = 0;
   out_3365537473869891391[9] = 0;
   out_3365537473869891391[10] = 0;
   out_3365537473869891391[11] = 0;
   out_3365537473869891391[12] = 0;
   out_3365537473869891391[13] = 0;
   out_3365537473869891391[14] = 1;
   out_3365537473869891391[15] = 0;
   out_3365537473869891391[16] = 0;
   out_3365537473869891391[17] = 0;
}
void h_30(double *state, double *unused, double *out_1246693495319447380) {
   out_1246693495319447380[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1325445083642856802) {
   out_1325445083642856802[0] = 0;
   out_1325445083642856802[1] = 0;
   out_1325445083642856802[2] = 0;
   out_1325445083642856802[3] = 0;
   out_1325445083642856802[4] = 1;
   out_1325445083642856802[5] = 0;
   out_1325445083642856802[6] = 0;
   out_1325445083642856802[7] = 0;
   out_1325445083642856802[8] = 0;
}
void h_26(double *state, double *unused, double *out_264609401984149268) {
   out_264609401984149268[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4934391193738448049) {
   out_4934391193738448049[0] = 0;
   out_4934391193738448049[1] = 0;
   out_4934391193738448049[2] = 0;
   out_4934391193738448049[3] = 0;
   out_4934391193738448049[4] = 0;
   out_4934391193738448049[5] = 0;
   out_4934391193738448049[6] = 0;
   out_4934391193738448049[7] = 1;
   out_4934391193738448049[8] = 0;
}
void h_27(double *state, double *unused, double *out_7763742406666673270) {
   out_7763742406666673270[0] = state[3];
}
void H_27(double *state, double *unused, double *out_849318228157568109) {
   out_849318228157568109[0] = 0;
   out_849318228157568109[1] = 0;
   out_849318228157568109[2] = 0;
   out_849318228157568109[3] = 1;
   out_849318228157568109[4] = 0;
   out_849318228157568109[5] = 0;
   out_849318228157568109[6] = 0;
   out_849318228157568109[7] = 0;
   out_849318228157568109[8] = 0;
}
void h_29(double *state, double *unused, double *out_4319407688741062485) {
   out_4319407688741062485[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1835676427957248986) {
   out_1835676427957248986[0] = 0;
   out_1835676427957248986[1] = 1;
   out_1835676427957248986[2] = 0;
   out_1835676427957248986[3] = 0;
   out_1835676427957248986[4] = 0;
   out_1835676427957248986[5] = 0;
   out_1835676427957248986[6] = 0;
   out_1835676427957248986[7] = 0;
   out_1835676427957248986[8] = 0;
}
void h_28(double *state, double *unused, double *out_5681328399382032701) {
   out_5681328399382032701[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3246722589112281588) {
   out_3246722589112281588[0] = 1;
   out_3246722589112281588[1] = 0;
   out_3246722589112281588[2] = 0;
   out_3246722589112281588[3] = 0;
   out_3246722589112281588[4] = 0;
   out_3246722589112281588[5] = 0;
   out_3246722589112281588[6] = 0;
   out_3246722589112281588[7] = 0;
   out_3246722589112281588[8] = 0;
}
void h_31(double *state, double *unused, double *out_1099614337422674027) {
   out_1099614337422674027[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5560599295971799525) {
   out_5560599295971799525[0] = 0;
   out_5560599295971799525[1] = 0;
   out_5560599295971799525[2] = 0;
   out_5560599295971799525[3] = 0;
   out_5560599295971799525[4] = 0;
   out_5560599295971799525[5] = 0;
   out_5560599295971799525[6] = 0;
   out_5560599295971799525[7] = 0;
   out_5560599295971799525[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8674688528389785859) {
  err_fun(nom_x, delta_x, out_8674688528389785859);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9192190503432797144) {
  inv_err_fun(nom_x, true_x, out_9192190503432797144);
}
void car_H_mod_fun(double *state, double *out_7340844205401277523) {
  H_mod_fun(state, out_7340844205401277523);
}
void car_f_fun(double *state, double dt, double *out_1286214365289940422) {
  f_fun(state,  dt, out_1286214365289940422);
}
void car_F_fun(double *state, double dt, double *out_8185406033884487895) {
  F_fun(state,  dt, out_8185406033884487895);
}
void car_h_25(double *state, double *unused, double *out_8560173101671562352) {
  h_25(state, unused, out_8560173101671562352);
}
void car_H_25(double *state, double *unused, double *out_1192887874864391825) {
  H_25(state, unused, out_1192887874864391825);
}
void car_h_24(double *state, double *unused, double *out_4400767935302587282) {
  h_24(state, unused, out_4400767935302587282);
}
void car_H_24(double *state, double *unused, double *out_3365537473869891391) {
  H_24(state, unused, out_3365537473869891391);
}
void car_h_30(double *state, double *unused, double *out_1246693495319447380) {
  h_30(state, unused, out_1246693495319447380);
}
void car_H_30(double *state, double *unused, double *out_1325445083642856802) {
  H_30(state, unused, out_1325445083642856802);
}
void car_h_26(double *state, double *unused, double *out_264609401984149268) {
  h_26(state, unused, out_264609401984149268);
}
void car_H_26(double *state, double *unused, double *out_4934391193738448049) {
  H_26(state, unused, out_4934391193738448049);
}
void car_h_27(double *state, double *unused, double *out_7763742406666673270) {
  h_27(state, unused, out_7763742406666673270);
}
void car_H_27(double *state, double *unused, double *out_849318228157568109) {
  H_27(state, unused, out_849318228157568109);
}
void car_h_29(double *state, double *unused, double *out_4319407688741062485) {
  h_29(state, unused, out_4319407688741062485);
}
void car_H_29(double *state, double *unused, double *out_1835676427957248986) {
  H_29(state, unused, out_1835676427957248986);
}
void car_h_28(double *state, double *unused, double *out_5681328399382032701) {
  h_28(state, unused, out_5681328399382032701);
}
void car_H_28(double *state, double *unused, double *out_3246722589112281588) {
  H_28(state, unused, out_3246722589112281588);
}
void car_h_31(double *state, double *unused, double *out_1099614337422674027) {
  h_31(state, unused, out_1099614337422674027);
}
void car_H_31(double *state, double *unused, double *out_5560599295971799525) {
  H_31(state, unused, out_5560599295971799525);
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
