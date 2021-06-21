//
// Created by jun on 20-3-17.
//

#ifndef PLANTOOLKIT_H
#define PLANTOOLKIT_H

#include <cmath>        // std::fabs(), atan(), sin(), cos()
#include <algorithm>    // std::max() & min()
#include <vector>
#include <Eigen/Dense>

#ifndef PI
    #define PI 3.141592654
#endif // PI


namespace plan {

// Clamp value between two bounds.
double clamp(double num, double lim1, double lim2);
// Compute scalar (0 to 1) representing forces in leg.
double scaleFactor(double f, double tl, double tu);

// a less time consuming power function
double my_pow(double x, int n);

// factorial
double fact(int n);

// Diagonal Matrix Generation
Eigen::MatrixXd diag(const std::vector<double>& diagElement);

bool line_s(double x1, double x2, double y1, double y2, double x, double dx, double& y, double& dy, double& ddy);
bool line_s(double x1, double x2, double y1, double y2, double x, double dx, double& y, double& dy);
bool line_s(double x1, double x2, double y1, double y2, double x, double dx, double& y);
bool line_s(double x1, double x2, Eigen::Vector2d vec_y1, Eigen::Vector2d vec_y2, double x, double dx, Eigen::Vector2d & vec_y);

/**
 * @brief cubic_s: Cubic interpolation function.
 * @param x1, x2 The coordinates of the two points.
 * @param y1, y2 The values of the two points.
 * @param dy1, dy2 The slope of the two points.
 * @param x The point to be interpolated.
 * @param dx The First derivative with respect to time.
 * @param ddx The Second derivative with respect to time.
 * @return y, dy, ddy : The interpolated value and its derivative.
 */
bool cubic_s(double x1, double x2, double y1, double y2, double dy1, double dy2, double x, double dx, double& y, double& dy, double& ddy);
bool cubic_s(double x1, double x2, double y1, double y2, double dy1, double dy2, double x, double dx, double& y, double& dy);


/**
 * @brief quintic_s: Quintic interpolation function.
 * @param x1, x2 The coordinates of the two points.
 * @param y1, y2 The values of the two points.
 * @param dy1, dy2 The slope of the two points.
 * @param ddy1, ddy2 the acceleration of the two points
 * @param x The point to be interpolated.
 * @param dx The First derivative with respect to time.
 * @param ddx The Second derivative with respect to time.
 * @return y, dy, ddy : The interpolated value and its derivative.
 */
bool quintic_s(double x1, double x2, double y1, double y2, double dy1, double dy2, double ddy1, double ddy2, double x, double dx, double& y, double& dy, double& ddy);
bool quintic_s(double x1, double x2, double y1, double y2, double dy1, double dy2, double ddy1, double ddy2, double x, double dx, double& y, double& dy);


/**
 * @brief cal_eta
 * @param v1
 * @param eta1
 * @param v2
 * @param eta2
 * @param dx_real_time
 * @param dx_ave_step
 * @param velTrkFlag
 * @param etaRes
 * @return etaRes
 */
bool cal_eta_order1(double v1, double eta1, double v2, double eta2, double dx_real_time, double dx_ave_step, double dx_target, int velTrkFlag, double & etaRes);

// hyperbolic function, which Not included in <cmath>
double coth(double x);
double sech(double x);
double csch(double x);
// hyperbolic function

/**
 * @brief get_HIPM_P1_Params_Rom0
 *          Assume that "ddx == 0" in DSP
 * @param g
 * @param H
 * @param Ts
 * @param Td
 * @param lambda
 * @param sigma
 * @param Kp_star
 * @param xi
 * @return
 */
bool get_HIPM_P1_Params_Rom0(double g, double H, double Ts, double Td,
                     double &lambda, double &sigma, double &Kp_star, double &xi);

/**
 * @brief get_HIPM_P1_VelCtrl_Rom0
 *          Get the step length for H-IPM model P1-orbit Velocity-Control, and assume the ddx in DSP is ZERO.
 *          The goal is let 'dxE_SSP == dxE_SSP_d' in only one step.
 * @param xE_SSP
 * @param dxE_SSP
 * @param dxE_SSP_d
 * @param Kp_star
 * @param lambda
 * @param sigma
 * @param Ts
 * @param Td
 * @param H
 * @param x_S_step
 * @param x_G_step
 * @param x_step_DSP
 * @param x_step_SSP
 * @param delta_dx_DSP
 * @return
 */
bool get_HIPM_P1_VelCtrl_Rom0(double xE_SSP, double dxE_SSP, double dxE_SSP_d, double Kp_star, double lambda, double sigma, double Ts, double Td, double H,
                               double &x_S_step, double &x_G_step, double &x_step_DSP, double &x_step_SSP, double &delta_dx_DSP );

/**
 * @brief get_HIPM_P1_Params_Rom1
 *          ddx is real-like in DSP
 * @param g
 * @param H
 * @param Ts
 * @param Td
 * @param lambda
 * @param sigma
 * @param Kp_star
 * @param xi
 * @return
 */
bool get_HIPM_P1_Params_Rom1(double g, double H, double Ts, double Td,
                     double &lambda, double &sigma, double &Kp_star, double &xi);
/**
 * @brief get_HIPM_P1_VelCtrl_Rom1
 *          Get the step length for H-IPM model P1-orbit Velocity-Control, and the ddx in DSP is real-like, not ZERO.
 *          The goal is let 'dxE_SSP == dxE_SSP_d' in only one step.
 * Note :
 *       (xE_SSP, dxE_SSP) are in 'Fi', mean 'xE_SSP' is 'xlocalE_SSP'
 *
 *       x_S_step = xE_SSP + x_G_step = xE_SSP + x_step_DSP + x_step_SSP
 *       x_G_step = x_step_DSP + x_step_SSP
 *       x_step_SSP = (1/sigma)*dxE_SSP + Kp*(dxE_SSP - dxE_SSP_d)
 *                   = (1/sigma)*dxE_SSP + Kp_star*(dxE_SSP - dxE_SSP_d) + (coth(lambda*Ts)/lambda)*delta_dx_DSP
 *       Kp = Kp_star + coth(lambda*Ts)/(lambda*(dxE_SSP - dxE_d))*delta_dx_DSP
 *       Kp_star = csch(lambda*Ts)/lambda
 */
bool get_HIPM_P1_VelCtrl_Rom1(double xE_SSP, double dxE_SSP, double dxE_SSP_d, double Kp_star, double lambda, double sigma, double Ts, double Td, double H,
                               double &x_S_step, double &x_G_step, double &x_step_DSP, double &x_step_SSP, double &delta_dx_DSP );

// cw2021 for LIP
/**
 * @brief planLipForward
 *          plan the position and velocity of CoM of LIP, given the state at t1, predict the state at t2, t1 < t2
 * @return
 */
bool planLipForward(double t1, double t2, double x1, double xDot1, double H, double & x2, double & xDot2);
bool planLipForward(double t1, double t2, double x1, double xDot1, double H, double & x2, double & xDot2, double & xDDot2);

/**
 * @brief planLipBackward
 *          plan the position and velocity of CoM of LIP, given the state at t2, derive the state at t1, t1 < t2
 * @return
 */
bool planLipBackward(double t1, double t2, double x2, double xDot2, double H, double & x1, double & xDot1);

bool getHlipParams(double H, double Ts, double Td, double stepLength_hlip,
                   double & lambda, double & sigma, double & xi, double & Kp_star, double & xDot_init, double & x_init);
// end of cw2021 for LIP

/**
 * @brief Compute the torque in the limits of Friction Cone and Support, for TBC in Polar.
 *          fzmin <= fz <= fzmax && -mu*fz <= fx <= mu*fz, get 4 Unilateral Constraints
 * @param u_tor
 * @param fr
 * @param r
 * @param a
 * @param mu
 * @param fzmin
 * @param fzmax
 * @return u : Processed variables u_tor
 */
double coneLimitPolar(double u_tor, double fr, double r, double a, double mu, double fzmin, double fzmax);


/**
 * @brief fzLimitCart
 *          Compute the fz_W_d = -fz_grf_d in the limits of Support, for SSP stance leg in Cartesian.
 *          fzmin <= fz_grf <= fzmax, get 2 Unilateral Constraints
 * @param fz_W_d = -fz_grf_d
 * @param fzmin
 * @param fzmax
 * @return fz_W_d
 */
double fzLimitCart(double fz_W_d, double fzmin, double fzmax);

/**
 * @brief coneLimitCart
 *          Compute the fx_grf_d in the limits of Support, for TBC in Cartesian.
 *          -mu*fz <= fx <= mu*fz, get 2 Unilateral Constraints
 * @param fx_grf_d, the value to be modified
 * @param fz_grf_d, the known value
 * @param mu
 * @return fx_tbc : Processed variables fx_tbc
 */
double coneLimitCart(double fx_grf_d, double fz_grf_d, double mu);

/**
 * @brief calKneeAngle
 *          compute knee angle, for Emergency (Mechanical joint limits)
 * @param l0
 * @param l1
 * @param l2
 * @param x_foot
 * @param z_foot
 * @param q_hip_fore
 * @param q_hip_rear
 * @param q_knee_fore
 * @param q_knee_rear
 * @return
 */
bool calKneeAngle(double l0, double l1, double l2, double x_foot, double z_foot, double q_hip_fore, double q_hip_rear, double & q_knee_fore, double & q_knee_rear);


}   // namespace plan

#endif // PLANTOOLKIT_H
