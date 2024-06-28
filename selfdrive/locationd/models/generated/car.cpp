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
void err_fun(double *nom_x, double *delta_x, double *out_2599240773775886843) {
   out_2599240773775886843[0] = delta_x[0] + nom_x[0];
   out_2599240773775886843[1] = delta_x[1] + nom_x[1];
   out_2599240773775886843[2] = delta_x[2] + nom_x[2];
   out_2599240773775886843[3] = delta_x[3] + nom_x[3];
   out_2599240773775886843[4] = delta_x[4] + nom_x[4];
   out_2599240773775886843[5] = delta_x[5] + nom_x[5];
   out_2599240773775886843[6] = delta_x[6] + nom_x[6];
   out_2599240773775886843[7] = delta_x[7] + nom_x[7];
   out_2599240773775886843[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5090377595086658096) {
   out_5090377595086658096[0] = -nom_x[0] + true_x[0];
   out_5090377595086658096[1] = -nom_x[1] + true_x[1];
   out_5090377595086658096[2] = -nom_x[2] + true_x[2];
   out_5090377595086658096[3] = -nom_x[3] + true_x[3];
   out_5090377595086658096[4] = -nom_x[4] + true_x[4];
   out_5090377595086658096[5] = -nom_x[5] + true_x[5];
   out_5090377595086658096[6] = -nom_x[6] + true_x[6];
   out_5090377595086658096[7] = -nom_x[7] + true_x[7];
   out_5090377595086658096[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8489705339772509506) {
   out_8489705339772509506[0] = 1.0;
   out_8489705339772509506[1] = 0;
   out_8489705339772509506[2] = 0;
   out_8489705339772509506[3] = 0;
   out_8489705339772509506[4] = 0;
   out_8489705339772509506[5] = 0;
   out_8489705339772509506[6] = 0;
   out_8489705339772509506[7] = 0;
   out_8489705339772509506[8] = 0;
   out_8489705339772509506[9] = 0;
   out_8489705339772509506[10] = 1.0;
   out_8489705339772509506[11] = 0;
   out_8489705339772509506[12] = 0;
   out_8489705339772509506[13] = 0;
   out_8489705339772509506[14] = 0;
   out_8489705339772509506[15] = 0;
   out_8489705339772509506[16] = 0;
   out_8489705339772509506[17] = 0;
   out_8489705339772509506[18] = 0;
   out_8489705339772509506[19] = 0;
   out_8489705339772509506[20] = 1.0;
   out_8489705339772509506[21] = 0;
   out_8489705339772509506[22] = 0;
   out_8489705339772509506[23] = 0;
   out_8489705339772509506[24] = 0;
   out_8489705339772509506[25] = 0;
   out_8489705339772509506[26] = 0;
   out_8489705339772509506[27] = 0;
   out_8489705339772509506[28] = 0;
   out_8489705339772509506[29] = 0;
   out_8489705339772509506[30] = 1.0;
   out_8489705339772509506[31] = 0;
   out_8489705339772509506[32] = 0;
   out_8489705339772509506[33] = 0;
   out_8489705339772509506[34] = 0;
   out_8489705339772509506[35] = 0;
   out_8489705339772509506[36] = 0;
   out_8489705339772509506[37] = 0;
   out_8489705339772509506[38] = 0;
   out_8489705339772509506[39] = 0;
   out_8489705339772509506[40] = 1.0;
   out_8489705339772509506[41] = 0;
   out_8489705339772509506[42] = 0;
   out_8489705339772509506[43] = 0;
   out_8489705339772509506[44] = 0;
   out_8489705339772509506[45] = 0;
   out_8489705339772509506[46] = 0;
   out_8489705339772509506[47] = 0;
   out_8489705339772509506[48] = 0;
   out_8489705339772509506[49] = 0;
   out_8489705339772509506[50] = 1.0;
   out_8489705339772509506[51] = 0;
   out_8489705339772509506[52] = 0;
   out_8489705339772509506[53] = 0;
   out_8489705339772509506[54] = 0;
   out_8489705339772509506[55] = 0;
   out_8489705339772509506[56] = 0;
   out_8489705339772509506[57] = 0;
   out_8489705339772509506[58] = 0;
   out_8489705339772509506[59] = 0;
   out_8489705339772509506[60] = 1.0;
   out_8489705339772509506[61] = 0;
   out_8489705339772509506[62] = 0;
   out_8489705339772509506[63] = 0;
   out_8489705339772509506[64] = 0;
   out_8489705339772509506[65] = 0;
   out_8489705339772509506[66] = 0;
   out_8489705339772509506[67] = 0;
   out_8489705339772509506[68] = 0;
   out_8489705339772509506[69] = 0;
   out_8489705339772509506[70] = 1.0;
   out_8489705339772509506[71] = 0;
   out_8489705339772509506[72] = 0;
   out_8489705339772509506[73] = 0;
   out_8489705339772509506[74] = 0;
   out_8489705339772509506[75] = 0;
   out_8489705339772509506[76] = 0;
   out_8489705339772509506[77] = 0;
   out_8489705339772509506[78] = 0;
   out_8489705339772509506[79] = 0;
   out_8489705339772509506[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2090420721336331077) {
   out_2090420721336331077[0] = state[0];
   out_2090420721336331077[1] = state[1];
   out_2090420721336331077[2] = state[2];
   out_2090420721336331077[3] = state[3];
   out_2090420721336331077[4] = state[4];
   out_2090420721336331077[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2090420721336331077[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2090420721336331077[7] = state[7];
   out_2090420721336331077[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6020752249717324052) {
   out_6020752249717324052[0] = 1;
   out_6020752249717324052[1] = 0;
   out_6020752249717324052[2] = 0;
   out_6020752249717324052[3] = 0;
   out_6020752249717324052[4] = 0;
   out_6020752249717324052[5] = 0;
   out_6020752249717324052[6] = 0;
   out_6020752249717324052[7] = 0;
   out_6020752249717324052[8] = 0;
   out_6020752249717324052[9] = 0;
   out_6020752249717324052[10] = 1;
   out_6020752249717324052[11] = 0;
   out_6020752249717324052[12] = 0;
   out_6020752249717324052[13] = 0;
   out_6020752249717324052[14] = 0;
   out_6020752249717324052[15] = 0;
   out_6020752249717324052[16] = 0;
   out_6020752249717324052[17] = 0;
   out_6020752249717324052[18] = 0;
   out_6020752249717324052[19] = 0;
   out_6020752249717324052[20] = 1;
   out_6020752249717324052[21] = 0;
   out_6020752249717324052[22] = 0;
   out_6020752249717324052[23] = 0;
   out_6020752249717324052[24] = 0;
   out_6020752249717324052[25] = 0;
   out_6020752249717324052[26] = 0;
   out_6020752249717324052[27] = 0;
   out_6020752249717324052[28] = 0;
   out_6020752249717324052[29] = 0;
   out_6020752249717324052[30] = 1;
   out_6020752249717324052[31] = 0;
   out_6020752249717324052[32] = 0;
   out_6020752249717324052[33] = 0;
   out_6020752249717324052[34] = 0;
   out_6020752249717324052[35] = 0;
   out_6020752249717324052[36] = 0;
   out_6020752249717324052[37] = 0;
   out_6020752249717324052[38] = 0;
   out_6020752249717324052[39] = 0;
   out_6020752249717324052[40] = 1;
   out_6020752249717324052[41] = 0;
   out_6020752249717324052[42] = 0;
   out_6020752249717324052[43] = 0;
   out_6020752249717324052[44] = 0;
   out_6020752249717324052[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6020752249717324052[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6020752249717324052[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6020752249717324052[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6020752249717324052[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6020752249717324052[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6020752249717324052[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6020752249717324052[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6020752249717324052[53] = -9.8000000000000007*dt;
   out_6020752249717324052[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6020752249717324052[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6020752249717324052[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6020752249717324052[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6020752249717324052[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6020752249717324052[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6020752249717324052[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6020752249717324052[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6020752249717324052[62] = 0;
   out_6020752249717324052[63] = 0;
   out_6020752249717324052[64] = 0;
   out_6020752249717324052[65] = 0;
   out_6020752249717324052[66] = 0;
   out_6020752249717324052[67] = 0;
   out_6020752249717324052[68] = 0;
   out_6020752249717324052[69] = 0;
   out_6020752249717324052[70] = 1;
   out_6020752249717324052[71] = 0;
   out_6020752249717324052[72] = 0;
   out_6020752249717324052[73] = 0;
   out_6020752249717324052[74] = 0;
   out_6020752249717324052[75] = 0;
   out_6020752249717324052[76] = 0;
   out_6020752249717324052[77] = 0;
   out_6020752249717324052[78] = 0;
   out_6020752249717324052[79] = 0;
   out_6020752249717324052[80] = 1;
}
void h_25(double *state, double *unused, double *out_6342563028342440243) {
   out_6342563028342440243[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5751011039510315989) {
   out_5751011039510315989[0] = 0;
   out_5751011039510315989[1] = 0;
   out_5751011039510315989[2] = 0;
   out_5751011039510315989[3] = 0;
   out_5751011039510315989[4] = 0;
   out_5751011039510315989[5] = 0;
   out_5751011039510315989[6] = 1;
   out_5751011039510315989[7] = 0;
   out_5751011039510315989[8] = 0;
}
void h_24(double *state, double *unused, double *out_8427152767156752697) {
   out_8427152767156752697[0] = state[4];
   out_8427152767156752697[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1749298960879807356) {
   out_1749298960879807356[0] = 0;
   out_1749298960879807356[1] = 0;
   out_1749298960879807356[2] = 0;
   out_1749298960879807356[3] = 0;
   out_1749298960879807356[4] = 1;
   out_1749298960879807356[5] = 0;
   out_1749298960879807356[6] = 0;
   out_1749298960879807356[7] = 0;
   out_1749298960879807356[8] = 0;
   out_1749298960879807356[9] = 0;
   out_1749298960879807356[10] = 0;
   out_1749298960879807356[11] = 0;
   out_1749298960879807356[12] = 0;
   out_1749298960879807356[13] = 0;
   out_1749298960879807356[14] = 1;
   out_1749298960879807356[15] = 0;
   out_1749298960879807356[16] = 0;
   out_1749298960879807356[17] = 0;
}
void h_30(double *state, double *unused, double *out_428272198007910693) {
   out_428272198007910693[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1165679301981300766) {
   out_1165679301981300766[0] = 0;
   out_1165679301981300766[1] = 0;
   out_1165679301981300766[2] = 0;
   out_1165679301981300766[3] = 0;
   out_1165679301981300766[4] = 1;
   out_1165679301981300766[5] = 0;
   out_1165679301981300766[6] = 0;
   out_1165679301981300766[7] = 0;
   out_1165679301981300766[8] = 0;
}
void h_26(double *state, double *unused, double *out_5050458227464019007) {
   out_5050458227464019007[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1951872313234852740) {
   out_1951872313234852740[0] = 0;
   out_1951872313234852740[1] = 0;
   out_1951872313234852740[2] = 0;
   out_1951872313234852740[3] = 0;
   out_1951872313234852740[4] = 0;
   out_1951872313234852740[5] = 0;
   out_1951872313234852740[6] = 0;
   out_1951872313234852740[7] = 1;
   out_1951872313234852740[8] = 0;
}
void h_27(double *state, double *unused, double *out_7102507777706265728) {
   out_7102507777706265728[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1009084009819124145) {
   out_1009084009819124145[0] = 0;
   out_1009084009819124145[1] = 0;
   out_1009084009819124145[2] = 0;
   out_1009084009819124145[3] = 1;
   out_1009084009819124145[4] = 0;
   out_1009084009819124145[5] = 0;
   out_1009084009819124145[6] = 0;
   out_1009084009819124145[7] = 0;
   out_1009084009819124145[8] = 0;
}
void h_29(double *state, double *unused, double *out_3465058249452243378) {
   out_3465058249452243378[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1675910646295692950) {
   out_1675910646295692950[0] = 0;
   out_1675910646295692950[1] = 1;
   out_1675910646295692950[2] = 0;
   out_1675910646295692950[3] = 0;
   out_1675910646295692950[4] = 0;
   out_1675910646295692950[5] = 0;
   out_1675910646295692950[6] = 0;
   out_1675910646295692950[7] = 0;
   out_1675910646295692950[8] = 0;
}
void h_28(double *state, double *unused, double *out_8270251578782407465) {
   out_8270251578782407465[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3406488370773837624) {
   out_3406488370773837624[0] = 1;
   out_3406488370773837624[1] = 0;
   out_3406488370773837624[2] = 0;
   out_3406488370773837624[3] = 0;
   out_3406488370773837624[4] = 0;
   out_3406488370773837624[5] = 0;
   out_3406488370773837624[6] = 0;
   out_3406488370773837624[7] = 0;
   out_3406488370773837624[8] = 0;
}
void h_31(double *state, double *unused, double *out_187115832501780114) {
   out_187115832501780114[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1325664211001501264) {
   out_1325664211001501264[0] = 0;
   out_1325664211001501264[1] = 0;
   out_1325664211001501264[2] = 0;
   out_1325664211001501264[3] = 0;
   out_1325664211001501264[4] = 0;
   out_1325664211001501264[5] = 0;
   out_1325664211001501264[6] = 0;
   out_1325664211001501264[7] = 0;
   out_1325664211001501264[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2599240773775886843) {
  err_fun(nom_x, delta_x, out_2599240773775886843);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5090377595086658096) {
  inv_err_fun(nom_x, true_x, out_5090377595086658096);
}
void car_H_mod_fun(double *state, double *out_8489705339772509506) {
  H_mod_fun(state, out_8489705339772509506);
}
void car_f_fun(double *state, double dt, double *out_2090420721336331077) {
  f_fun(state,  dt, out_2090420721336331077);
}
void car_F_fun(double *state, double dt, double *out_6020752249717324052) {
  F_fun(state,  dt, out_6020752249717324052);
}
void car_h_25(double *state, double *unused, double *out_6342563028342440243) {
  h_25(state, unused, out_6342563028342440243);
}
void car_H_25(double *state, double *unused, double *out_5751011039510315989) {
  H_25(state, unused, out_5751011039510315989);
}
void car_h_24(double *state, double *unused, double *out_8427152767156752697) {
  h_24(state, unused, out_8427152767156752697);
}
void car_H_24(double *state, double *unused, double *out_1749298960879807356) {
  H_24(state, unused, out_1749298960879807356);
}
void car_h_30(double *state, double *unused, double *out_428272198007910693) {
  h_30(state, unused, out_428272198007910693);
}
void car_H_30(double *state, double *unused, double *out_1165679301981300766) {
  H_30(state, unused, out_1165679301981300766);
}
void car_h_26(double *state, double *unused, double *out_5050458227464019007) {
  h_26(state, unused, out_5050458227464019007);
}
void car_H_26(double *state, double *unused, double *out_1951872313234852740) {
  H_26(state, unused, out_1951872313234852740);
}
void car_h_27(double *state, double *unused, double *out_7102507777706265728) {
  h_27(state, unused, out_7102507777706265728);
}
void car_H_27(double *state, double *unused, double *out_1009084009819124145) {
  H_27(state, unused, out_1009084009819124145);
}
void car_h_29(double *state, double *unused, double *out_3465058249452243378) {
  h_29(state, unused, out_3465058249452243378);
}
void car_H_29(double *state, double *unused, double *out_1675910646295692950) {
  H_29(state, unused, out_1675910646295692950);
}
void car_h_28(double *state, double *unused, double *out_8270251578782407465) {
  h_28(state, unused, out_8270251578782407465);
}
void car_H_28(double *state, double *unused, double *out_3406488370773837624) {
  H_28(state, unused, out_3406488370773837624);
}
void car_h_31(double *state, double *unused, double *out_187115832501780114) {
  h_31(state, unused, out_187115832501780114);
}
void car_H_31(double *state, double *unused, double *out_1325664211001501264) {
  H_31(state, unused, out_1325664211001501264);
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
