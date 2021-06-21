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
 * @brief planLipForward
 *          plan the position and velocity of CoM of LIP, given the state at t1, predict the state at t2, t1 < t2
 * @return
 */
bool planLipForward(double t1, double t2, double x1, double xDot1, double H, double & x2, double & xDot2);
bool planLipForward(double t1, double t2, double x1, double xDot1, double H, double & x2, double & xDot2, double & xDDot2);

}   // namespace plan

#endif // PLANTOOLKIT_H
