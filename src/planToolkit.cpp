//
// Created by jun on 20-3-17.
//

#include "planToolkit.h"

namespace plan {

double clamp(double num, double lim1, double lim2) {
    auto min = std::min(lim1, lim2);
    auto max = std::max(lim1, lim2);

    if (num < min)
        return min;

    if (max < num)
        return max;

    return num;
}

double scaleFactor(double f, double tl, double tu){
    double s = (clamp(f, tl, tu) - tl)/(tu - tl);
    return s;
}

double my_pow(double x, int n){
    if(n == 0)
        return 1.0;
    if(n < 0)
        return 1.0/my_pow(x,-n);
    double half = my_pow(x,n>>1);

    if(n%2 == 0)
        return half*half;
    else
    {
        return half*half*x;
    }

}

double fact(int n){
    double result = 1;
    if (n == 0)
        result = 1;
    else
        for (int i = 1;i <= n;result *= i, i++);
    return result;
}


Eigen::MatrixXd diag(const std::vector<double>& diagElement){
    int dim = static_cast<int>(diagElement.size());
    Eigen::MatrixXd diagM(dim, dim);
    diagM = Eigen::MatrixXd::Zero(dim, dim);
    for (int i = 0; i != dim; i++){
        diagM(i, i) =  diagElement.at(i);
    }
    return diagM;
}

bool line_s(double x1, double x2, double y1, double y2, double x, double dx, double& y, double& dy, double& ddy){
    // Limit range since curve fit is only valid within range
    x = clamp(x, x1, x2);

    // Declare variables
    double t, t1, t2, deltaY, deltaT, a0, a1;

    // variable substitution
    t = x/dx;
    t1 = x1/dx;
    t2 = x2/dx;
    // interpolate
    deltaY = y2 - y1;
    deltaT = t2 - t1;
    a0 = y1;
    a1 = deltaY/deltaT;

    // position
    y = a0 + a1*(t - t1);
    // velocity
    dy = a1;
    // acceleration
    ddy = 0.0;

    return true;
}

bool line_s(double x1, double x2, double y1, double y2, double x, double dx, double& y, double& dy){
    double ddy;
    line_s(x1, x2, y1, y2, x, dx, y, dy, ddy);
    return true;
}

bool line_s(double x1, double x2, double y1, double y2, double x, double dx, double& y){
    double dy, ddy;
    line_s(x1, x2, y1, y2, x, dx, y, dy, ddy);
    return true;
}

bool line_s(double x1, double x2, Eigen::Vector2d vec_y1, Eigen::Vector2d vec_y2, double x, double dx, Eigen::Vector2d & vec_y){
    line_s(x1, x2, vec_y1(0), vec_y2(0), x, dx, vec_y(0));
    line_s(x1, x2, vec_y1(1), vec_y2(1), x, dx, vec_y(1));
    return true;
}

bool cubic_s(double x1, double x2, double y1, double y2, double dy1, double dy2, double x, double dx, double& y, double& dy, double& ddy){
    // Limit range since curve fit is only valid within range
    x = clamp(x, x1, x2);

    // Declare variables
    double t, t1, t2, deltaY, deltaT, a0, a1, a2, a3;

    // variable substitution
    t = x/dx;
    t1 = x1/dx;
    t2 = x2/dx;
    // interpolate
    deltaY = y2 - y1;
    deltaT = t2 - t1;
    a0 = y1;
    a1 = dy1;
    a2 = (3*deltaY - (2*dy1 + dy2)*deltaT)/(deltaT*deltaT);
    a3 = (- 2*deltaY + (dy1 + dy2)*deltaT)/(deltaT*deltaT*deltaT);

    // position
    y = a0 + a1*my_pow((t - t1),1) + a2*my_pow((t - t1),2) + a3*my_pow((t - t1),3);
    // velocity
    dy = a1 + 2*a2*my_pow((t - t1),1) + 3*a3*my_pow((t - t1),2);
    // acceleration
    ddy = 2*a2 + 6*a3*my_pow((t - t1),1);

    return true;
}

bool cubic_s(double x1, double x2, double y1, double y2, double dy1, double dy2, double x, double dx, double& y, double& dy){
    double ddy;
    cubic_s(x1, x2, y1, y2, dy1, dy2, x, dx, y, dy, ddy);
    return true;
}

bool quintic_s(double x1, double x2, double y1, double y2, double dy1, double dy2, double ddy1, double ddy2, double x, double dx, double& y, double& dy, double& ddy){
    // Limit range since curve fit is only valid within range
    x = clamp(x, x1, x2);

    // Declare variables
    double t, t1, t2, deltaY, deltaT, a0, a1, a2, a3, a4, a5;

    // variable substitution
    t = x/dx;
    t1 = x1/dx;
    t2 = x2/dx;
    // interpolate
    deltaY = y2 - y1;
    deltaT = t2 - t1;
    a0 = y1;
    a1 = dy1;
    a2 = 1.0/2*ddy1;
    a3 = 1.0/(2*deltaT*deltaT*deltaT)*(20*deltaY - (8*dy2 + 12*dy1)*deltaT + (ddy2 - 3*ddy1)*(deltaT*deltaT));
    a4 = 1.0/(2*deltaT*deltaT*deltaT*deltaT)*(-30*deltaY + (14*dy2 + 16*dy1)*deltaT + (3*ddy1 - 2*ddy2)*(deltaT*deltaT));
    a5 = 1.0/(2*deltaT*deltaT*deltaT*deltaT*deltaT)*(12*deltaY - 6*(dy2 + dy1)*deltaT + (ddy2 - ddy1)*(deltaT*deltaT));

    // position
    y = a0 + a1*my_pow((t - t1),1) + a2*my_pow((t - t1),2) + a3*my_pow((t - t1),3) + a4*my_pow(t - t1,4) + a5*my_pow(t - t1,5);
    // velocity
    dy = a1 + 2*a2*my_pow((t - t1),1) + 3*a3*my_pow((t - t1),2) + 4*a4*my_pow(t - t1,3) + 5*a5*my_pow(t - t1,4);
    // acceleration
    ddy = 2*a2 +6*a3*my_pow((t - t1),1) + 12*a4*my_pow(t - t1,2) + 20*a5*my_pow(t - t1,3);

    return true;
}

bool quintic_s(double x1, double x2, double y1, double y2, double dy1, double dy2, double ddy1, double ddy2, double x, double dx, double& y, double& dy){
    double ddy;
    quintic_s(x1, x2, y1, y2, dy1, dy2, ddy1, ddy2, x, dx, y, dy, ddy);
    return true;
}

bool cal_eta_order1(double v1, double eta1, double v2, double eta2, double dx_real_time, double dx_ave_step, double dx_target, int velTrkFlag, double & etaRes){
    double v;
    double deltaVx_max = 0.2;   // 0.2
    if (velTrkFlag == 1){
        v = dx_real_time;
    }else if (velTrkFlag == 2){
        v = dx_ave_step;
    }else{
        v = clamp(dx_target, dx_ave_step - deltaVx_max, dx_ave_step + deltaVx_max);
    }

    // eta = a0 + a1*v;
    double a0, a1;
    if (v1 >= v2 - 0.000001 && v1 <= v2 + 0.000001){
        etaRes = eta1;
    }else{
        a0 = (eta1*v2 - eta2*v1)/(v2 - v1);
        a1 = (eta2 - eta1)/(v2 - v1);
        etaRes = a0 + a1*v;
    }
    etaRes = clamp(etaRes, eta1, eta2);

    return true;
}

// hyperbolic function, which Not included in <cmath>
double coth(double x){
    return 1/std::tanh(x);
}
double sech(double x){
    return 1/std::cosh(x);
}
double csch(double x){
    return 1/std::sinh(x);
}
// end of hyperbolic function

bool get_HIPM_P1_Params_Rom0(double g, double H, double Ts, double Td,
                     double &lambda, double &sigma, double &Kp_star, double &xi){
    // lambda
    lambda = std::sqrt(g/H);
    // sigma
    sigma = lambda*coth(Ts/2*lambda);
    // xi
    xi = (Ts + Td)/(Td + 2/sigma);
    // Kp_star,  Kp should be within "0 < Kp < 2*Kp_star"
    Kp_star = csch(lambda*Ts)/lambda;

    return true;
}

bool get_HIPM_P1_VelCtrl_Rom0(double xE_SSP, double dxE_SSP, double dxE_SSP_d, double Kp_star, double lambda, double sigma, double Ts, double Td, double H,
                               double &x_S_step, double &x_G_step, double &x_step_DSP, double &x_step_SSP, double &delta_dx_DSP ){
    // DSP
    delta_dx_DSP = 0.0;

    x_step_DSP = dxE_SSP*Td;    // i.e. delta_x_DSP

    // step length
    x_step_SSP = (1/sigma)*dxE_SSP + Kp_star*(dxE_SSP - dxE_SSP_d);

    x_G_step = x_step_DSP + x_step_SSP;

    x_S_step = xE_SSP + x_G_step;

    return true;
}

bool planLipForward(double t1, double t2, double x1, double xDot1, double H, double & x2, double & xDot2){

    double g = 9.80665;
    double lambda = std::sqrt(g/H);
    double a = lambda*(t2-t1);

    Eigen::Matrix2d P = Eigen::Matrix2d::Zero();
    P << std::cosh(a) , 1/lambda*std::sinh(a),
        lambda*std::sinh(a), std::cosh(a);
    Eigen::Vector2d x = Eigen::Vector2d::Zero();
    x = P * Eigen::Vector2d(x1, xDot1);

    x2 = x(0);
    xDot2 = x(1);

    return true;
}

bool planLipForward(double t1, double t2, double x1, double xDot1, double H, double & x2, double & xDot2, double & xDDot2){
    double g = 9.80665;
    double lambda = std::sqrt(g/H);
    double a = lambda*(t2-t1);

    Eigen::Matrix2d P = Eigen::Matrix2d::Zero();
    P << std::cosh(a) , 1/lambda*std::sinh(a),
        lambda*std::sinh(a), std::cosh(a);
    Eigen::Vector2d x = Eigen::Vector2d::Zero();
    x = P * Eigen::Vector2d(x1, xDot1);

    x2 = x(0);
    xDot2 = x(1);
    xDDot2 = g/H*x2;

    return true;
}

}   // namespace plan
