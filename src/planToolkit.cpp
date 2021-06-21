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

bool get_HIPM_P1_Params_Rom1(double g, double H, double Ts, double Td,
                     double &lambda, double &sigma, double &Kp_star, double &xi){
    // lambda
    lambda = std::sqrt(g/H);
    // sigma
    sigma = lambda*coth(Ts/2*lambda);
    // xi
    double D = 2*lambda*Td/(std::exp(lambda*Td) - std::exp(-lambda*Td))
                *(0.5*(1/sigma + 1/lambda - 2/(sigma*lambda*Td))*(std::exp(lambda*Td) - 1)
                + 0.5*(1/sigma - 1/lambda + 2/(sigma*lambda*Td))*(std::exp(-lambda*Td) - 1)
                + 2/sigma);
    xi = (Ts + Td)/(D + 2/sigma);
    // Kp_star
    Kp_star = csch(lambda*Ts)/lambda;

    return true;
}

bool get_HIPM_P1_VelCtrl_Rom1(double xE_SSP, double dxE_SSP, double dxE_SSP_d, double Kp_star, double lambda, double sigma, double Ts, double Td, double H,
                               double &x_S_step, double &x_G_step, double &x_step_DSP, double &x_step_SSP, double &delta_dx_DSP ){
    // tuned well!

    delta_dx_DSP = ((xE_SSP + Kp_star*(dxE_SSP - dxE_SSP_d) + dxE_SSP/sigma + (Td*lambda*(2*xE_SSP + 2*Kp_star*(dxE_SSP - dxE_SSP_d)
        + 2*(std::exp(Td*lambda) - 1)*(xE_SSP/2 + dxE_SSP/(2*lambda) - (xE_SSP + Kp_star*(dxE_SSP - dxE_SSP_d) + dxE_SSP/sigma)/(2*Td*lambda))
        + 2*(std::exp(-Td*lambda) - 1)*(xE_SSP/2 - dxE_SSP/(2*lambda) + (xE_SSP + Kp_star*(dxE_SSP - dxE_SSP_d) + dxE_SSP/sigma)/(2*Td*lambda))
        + (2*dxE_SSP)/sigma))/(std::exp(Td*lambda) - std::exp(-Td*lambda)))/Td - dxE_SSP + lambda*(std::exp(Td*lambda)*(xE_SSP/2 + dxE_SSP/(2*lambda)
        - (xE_SSP + Kp_star*(dxE_SSP - dxE_SSP_d) + dxE_SSP/sigma + (Td*lambda*(2*xE_SSP + 2*Kp_star*(dxE_SSP - dxE_SSP_d) + 2*(std::exp(Td*lambda)
        - 1)*(xE_SSP/2 + dxE_SSP/(2*lambda) - (xE_SSP + Kp_star*(dxE_SSP - dxE_SSP_d) + dxE_SSP/sigma)/(2*Td*lambda)) + 2*(std::exp(-Td*lambda)
        - 1)*(xE_SSP/2 - dxE_SSP/(2*lambda) + (xE_SSP + Kp_star*(dxE_SSP - dxE_SSP_d) + dxE_SSP/sigma)/(2*Td*lambda)) + (2*dxE_SSP)/sigma))
        /(std::exp(Td*lambda) - std::exp(-Td*lambda)))/(2*Td*lambda)) - std::exp(-Td*lambda)*(xE_SSP/2 - dxE_SSP/(2*lambda) + (xE_SSP + Kp_star*(dxE_SSP
        - dxE_SSP_d) + dxE_SSP/sigma + (Td*lambda*(2*xE_SSP + 2*Kp_star*(dxE_SSP - dxE_SSP_d) + 2*(std::exp(Td*lambda) - 1)*(xE_SSP/2 + dxE_SSP/(2*lambda)
        - (xE_SSP + Kp_star*(dxE_SSP - dxE_SSP_d) + dxE_SSP/sigma)/(2*Td*lambda)) + 2*(std::exp(-Td*lambda) - 1)*(xE_SSP/2 - dxE_SSP/(2*lambda)
        + (xE_SSP + Kp_star*(dxE_SSP - dxE_SSP_d) + dxE_SSP/sigma)/(2*Td*lambda)) + (2*dxE_SSP)/sigma))/(std::exp(Td*lambda) - std::exp(-Td*lambda)))
        /(2*Td*lambda))))/(lambda*((std::exp(Td*lambda)*(coth(Ts*lambda)/lambda + (Td*lambda*((2*coth(Ts*lambda))/lambda - (coth(Ts*lambda)*(std::exp(Td*lambda)
        - 1))/(Td*lambda*lambda) + (coth(Ts*lambda)*(std::exp(-Td*lambda) - 1))/(Td*lambda*lambda)))/(std::exp(Td*lambda) - std::exp(-Td*lambda))))/(2*Td*lambda)
        + (std::exp(-Td*lambda)*(coth(Ts*lambda)/lambda + (Td*lambda*((2*coth(Ts*lambda))/lambda - (coth(Ts*lambda)*(std::exp(Td*lambda) - 1))/(Td*lambda*lambda)
        + (coth(Ts*lambda)*(std::exp(-Td*lambda) - 1))/(Td*lambda*lambda)))/(std::exp(Td*lambda) - std::exp(-Td*lambda))))/(2*Td*lambda)) - (coth(Ts*lambda)/lambda
        + (Td*lambda*((2*coth(Ts*lambda))/lambda - (coth(Ts*lambda)*(std::exp(Td*lambda) - 1))/(Td*lambda*lambda) + (coth(Ts*lambda)*(std::exp(-Td*lambda) - 1))
        /(Td*lambda*lambda)))/(std::exp(Td*lambda) - std::exp(-Td*lambda)))/Td + 1);

    // step length
    x_step_SSP = (1/sigma)*dxE_SSP + Kp_star*(dxE_SSP - dxE_SSP_d) + (coth(lambda*Ts)/lambda)*delta_dx_DSP;

    x_step_DSP = (0.5*(xE_SSP + dxE_SSP/lambda - (xE_SSP + x_step_SSP)/(lambda*Td))*(std::exp(lambda*Td) - 1)
                + 0.5*(xE_SSP - dxE_SSP/lambda + (xE_SSP + x_step_SSP)/(lambda*Td))*(std::exp(-lambda*Td) - 1)
                + xE_SSP + x_step_SSP)
                *2*lambda*Td/(std::exp(lambda*Td) - std::exp(-lambda*Td));

    x_G_step = x_step_DSP + x_step_SSP;

    x_S_step = xE_SSP + x_G_step;

    return true;
}

// cw2021 for LIP
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

bool planLipBackward(double t1, double t2, double x2, double xDot2, double H, double & x1, double & xDot1){

    double g = 9.80665;
    double lambda = std::sqrt(g/H);
    double a = lambda*(t2-t1);

    Eigen::Matrix2d P = Eigen::Matrix2d::Zero();
    P << std::cosh(a) , -1/lambda*std::sinh(a),
        -lambda*std::sinh(a), std::cosh(a);
    Eigen::Vector2d x = Eigen::Vector2d::Zero();
    x = P * Eigen::Vector2d(x2, xDot2);

    x1 = x(0);
    xDot1 = x(1);

    return true;
}

bool getHlipParams(double H, double Ts, double Td, double stepLength_hlip,
                   double & lambda, double & sigma, double & xi, double & Kp_star, double & xDot_init, double & x_init){
    double g = 9.80665;
    lambda = std::sqrt(g/H);
    sigma = lambda*coth(Ts/2*lambda);
    xi = (Ts + Td)/(Td + 2/sigma);
    Kp_star = csch(lambda*Ts)/lambda;

    double v_d = stepLength_hlip/(Ts + Td);
    xDot_init = v_d*xi;
    x_init = -xDot_init/sigma;

    return true;
}
// end of cw2021 for LIP

double fzLimitCart(double fz_W_d, double fzmin, double fzmax){
    double fz_grf_d = -fz_W_d;
    if(fz_grf_d <= fzmin){
        return -fzmin;
    }else if(fz_grf_d >= fzmax){
        return -fzmax;
    }else{
        return fz_W_d;
    }
}

double coneLimitCart(double fx_grf_d, double fz_grf_d, double mu){
    double abs_fz_grf_d = std::fabs(fz_grf_d);

    if(fx_grf_d <= -mu*abs_fz_grf_d){
        return -mu*abs_fz_grf_d;
    }else if(fx_grf_d >= mu*abs_fz_grf_d){
        return mu*abs_fz_grf_d;
    }else{
        return fx_grf_d;
    }
}

double coneLimitPolar(double u_tor, double fr, double r, double a, double mu, double fzmin, double fzmax){

    double epsilon = 1e-6;
    double eta = std::atan(1/mu);
    double u_min = 0, u_max = 0, u = 0;
    /**
     * u*cos(a) >= r*(fzmin - fr*sin(a)) = UC[0]
     * u*cos(a) <= r*(fzmax - fr*sin(a)) = UC[1]
     * u*cos(a+eta) >= -r*fl*sin(a+eta) = UC[2]
     * u*cos(a-eta) >= -r*fl*sin(a-eta) = UC[3]
     */
    // get 4 Unilateral Constraints
    double UC[4] = { r*(fzmin - fr*std::sin(a)), r*(fzmax - fr*std::sin(a)), -r*fr*std::sin(a+eta), -r*fr*std::sin(a-eta) };

    if (a > 0 && a < PI/2-eta-epsilon){
        u_min = std::max(std::max(UC[0]/std::cos(a), UC[2]/std::cos(a+eta)), UC[3]/std::cos(a-eta));
        u_max = UC[1]/std::cos(a);
        u = clamp(u_tor, u_min, u_max);
    } else if (a >= PI/2-eta-epsilon && a <= PI/2-eta+epsilon){
        u_min = std::max(UC[0]/std::cos(a), UC[3]/std::cos(a-eta));
        u_max = UC[1]/std::cos(a);
        u = clamp(u_tor, u_min, u_max);
    } else if (a > PI/2-eta+epsilon && a < PI/2-epsilon){
        u_min = std::max(UC[0]/std::cos(a), UC[3]/std::cos(a-eta));
        u_max = std::min(UC[1]/std::cos(a), UC[2]/std::cos(a+eta));
        u = clamp(u_tor, u_min, u_max);
    } else if (a >= PI/2-epsilon && a <= PI/2+epsilon){
        u_min = UC[3]/std::cos(a-eta);
        u_max = UC[2]/std::cos(a+eta);
        u = clamp(u_tor, u_min, u_max);
    } else if (a > PI/2+epsilon && a < PI/2+eta-epsilon){
        u_min = std::max(UC[1]/std::cos(a), UC[3]/std::cos(a-eta));
        u_max = std::min(UC[0]/std::cos(a), UC[2]/std::cos(a+eta));
        u = clamp(u_tor, u_min, u_max);
    } else if (a >= PI/2+eta-epsilon && a <= PI/2+eta+epsilon){
        u_min = UC[1]/std::cos(a);
        u_max = std::min(UC[0]/std::cos(a), UC[2]/std::cos(a+eta));
        u = clamp(u_tor, u_min, u_max);
    } else if (a > PI/2+eta+epsilon && a < PI){
        u_min = UC[1]/std::cos(a);
        u_max = std::min(std::min(UC[0]/std::cos(a), UC[2]/std::cos(a+eta)), UC[3]/std::cos(a-eta));
        u = clamp(u_tor, u_min, u_max);
    } else{
        u = 0;
    }
    return u;

}

bool calKneeAngle(double l0, double l1, double l2, double x_foot, double z_foot, double q_hip_fore, double q_hip_rear, double &q_knee_fore, double &q_knee_rear){
    double x_foot_fore = x_foot - 0.5*l0;
    double x_foot_rear = x_foot + 0.5*l0;
    q_knee_fore = PI/2 - q_hip_fore - std::atan2(z_foot - l1*cos(q_hip_fore), x_foot_fore - l1*sin(q_hip_fore));
    q_knee_rear = PI/2 - q_hip_rear - std::atan2(z_foot - l1*cos(q_hip_rear), x_foot_rear - l1*sin(q_hip_rear));
    return true;
}

}   // namespace plan
