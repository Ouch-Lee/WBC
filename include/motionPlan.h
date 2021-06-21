//
// Created by jun on 2020-8-19.
//

#ifndef MOTION_PLAN_H
#define MOTION_PLAN_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "robotMessage.h"
#include "planToolkit.h"

class motionPlan{

public:
    motionPlan() = default;
    ~motionPlan() = default;

    // parse command
    bool parseCmd(double Vx_cmd, int H_flag_cmd, robotStateMachine & _rsm, const robotState & _rs, robotDesired & _rd);

    // walk planning Center of Mass
    bool walkTrajectoryPlan(const robotStateMachine & _rsm, const robotState & _rs, robotDesired & _rd);
    // walk planning Upper Body
    bool walkTrajectoryPlan_Ub(const robotStateMachine & _rsm, const robotState & _rs, robotDesired & _rd);

    // hold in air plan
    bool holdInAirPlan(const robotStateMachine & _rsm, const robotState & _rs, robotDesired & _rd);
    // stand up plan
    bool standUpPlan(const robotStateMachine & _rsm, const robotState & _rs, robotDesired & _rd);

    // stand balance plan Center of Mass
    bool standBalancePlan(const robotStateMachine & _rsm, const robotState & _rs, robotDesired & _rd);
    // stand balance plan Upper Body
    bool standBalancePlan_Ub(const robotStateMachine & _rsm, const robotState & _rs, robotDesired & _rd);

private:

    // ------------------------ stand up plan -------------------------------
    double x_R_W_d, xDot_R_W_d;
    double z_R_W_d, zDot_R_W_d;
    double x_L_W_d, xDot_L_W_d;
    double z_L_W_d, zDot_L_W_d;
    double z_sw_S_d, dz_sw_S_d; // for standUp_flag == 1
    // ------------------------ stand up plan -------------------------------

    // ------------------------- stand balance plan -------------------------
    int ctrl_flag{1};   // 0: hold; 1: swing head; 2: prepare for walk "init = 0"

    double T_cycle{2.0};
    double phase_init{0.5*PI};
    double t_start{2.0};
    double pitch_amplitude{5.0*DEG2RAD};
    double x_amplitude{0.005};
    double z_amplitude{0.02};
    double pitch_bias{0.0};
    double x_bias{0.1};
    double z_bias{0.42 - z_amplitude};

    double t_{0.0};
    double t_last_{0.0};
    // ------------------------- stand balance plan -------------------------

    // ------------------------- walk plan record -------------------------------
    int steps{0};                                       // for build-in speed control
    int stepm{0};                                       // step in local loop

    double sf_cmd = 0.001;                              // vx_tgt, 2dSLIP: 0.0003

    double H_tgt_0{0.0};                    // target height of body at the end of DSP or the beginning of SSP
    double Hdot_tgt_0{0.0};

    double z_sw_S_d_TD{0.0}, zDot_sw_S_d_TD{0.0}, zDDot_sw_S_d_TD{0.0};
    double z_sw_W_d_TD{0.0}, zDot_sw_W_d_TD{0.0}, zDDot_sw_W_d_TD{0.0};

    double z_compensating{0.0};             // the value compensate of z_Fi_sw_d
    double vz_S_sw_TD_d{0.0};               // the desired velocity of Z at TD of swing-foot, -0.2

    double z_Com_Snext_d_TD{0.0};           // desired value at TD, in W frame
    double zDot_Com_Snext_d_TD{0.0};        // desired value at TD, in W frame

    double deltaZ_Com_Snext_DSP_d{0.0};     // desired body's displacement change in Z, w.r.t. 'sw'
    double deltaZ_Com_S_DSP_d{0.0};         // desired body's displacement change in Z, w.r.t. 'st'

    double z_U_Snext_d_TD{0.0};             // desired value at TD, in W frame
    double zDot_U_Snext_d_TD{0.0};          // desired value at TD, in W frame

    double deltaZ_U_Snext_DSP_d{0.0};       // desired body's displacement change in Z, w.r.t. 'sw'
    double deltaZ_U_S_DSP_d{0.0};           // desired body's displacement change in Z, w.r.t. 'st'
    // ------------------------- walk plan record -------------------------------

    // ----------------------------- hIPM Gaits ----------------------------------
    // hIPM parameters, for prediction of hIPM stepping control
//    double z_B_CoM{0.09938};                // Centroid offset in Z, get value from robotDynamics_Mario2D.cpp, for 'v_est = cDot_U_S' (vel_flag == 1)
    double H_predict{0.0}, Ts_predict{0.0}, Td_predict{0.0};

    // the velocity P gain : Kp should be within "0 < Kp < 2*Kp_star"
    double lambda_predict{0.0}, sigma_predict{0.0}, Kp_star_predict{0.0}, xi_predict{0.0};
    double VxA_d_predict{0.0};
    double VxE_SSP_d_predict{0.0};  //  = xi_predict * VxA_d_predict;
    double xE_SSP_predict{0.0};
    double VxE_SSP_predict{0.0};

    // outputs of hIPM stepping control
    double x_S_step_{0.0}, x_G_step_{0.0}, x_step_DSP_{0.0}, x_step_SSP_{0.0}, delta_vx_DSP_{0.0};
    double x_G_step_amended{0.0};
    double x_G_step_d{0.0}, x_S_step_d{0.0};

    // trick for circular walking, correction factors for [inner, outer] legs support.
    std::vector<double> rho_circular{0., 0.};
    // Security Guarantee
    double safety_margin{0.002};
    double x_sw_W_limit{0.20};
    double x_L_B_d_limit{0.20};
    double x_R_B_d_limit{0.20};

    // velocity control parameters
    double Eta_v{1.0};                                  // for 'SGLC'
    double eta_v{1.0};                                  // kv_v = eta_v*(1/sigma) in rom_flag = 2 & SGLC
    double kv_v{0.0};                                   // feedforward coefficient: kv_v = eta_v*(1/sigma),in rom_flag = 2 & SGLC
    double zeta_v{1.5};                                 // kp_v = zeta_v * Kp_star, in 'rom_flag = 0 or 2';
    double kp_v{0.0};                                   // X velocity error P gain (s)
    double rho{0.3};                                    // trick : kp_v_add = rho/lambda, not the best solution.
    double kd_v{0.1};                                   // X velocity error D gain (s) , no use now.
    double ki_v{0.0};                                   // X velocity error I gain (s) , no use now.

    // Simplified Model Stepping Plan
    bool hlipStepPlan(const robotStateMachine & _rsm, const robotState & _rs, robotDesired & _rd);
    // ----------------------------- hIPM Gaits ----------------------------------

};


#endif //MOTION_PLAN_H
