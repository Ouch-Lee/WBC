//
// Created by jun on 2020-8-19.
//

#ifndef ROBOT_MESSAGE_H
#define ROBOT_MESSAGE_H

#include <vector>
#include <Eigen/Dense>

#ifndef PI
    #define PI 3.141592654
#endif // PI

#ifndef GRAVITY
    #define GRAVITY 9.80665
#endif // GRAVITY-CONST

#ifndef DEG2RAD
    #define DEG2RAD 0.0174532925
#endif // DEG2RAD

#ifndef RAD2DEG
    #define RAD2DEG 57.29577951308232
#endif // RAD2DEG

#ifndef EPSILON
    #define EPSILON 1e-6
#endif // EPSILON

#ifndef DT
    #define DT 1e-3
#endif // DELTA-T


/**
 * @brief The PidParam struct
 *          for actuatorMode_flag, the nominal control parameters of PID
 * @note
 * - in light-legs:     P = 65, I = 0, D = 0.15
 * - in weight-legs:    P = 100, I = 0, D = 0.2
 */
struct PidParam {
    double P = 100.0;
    double I = 0.0;
    double D = 0.2;
    double D_qDot_integral = 0.05;
    std::vector<double> p_pid_Lst_{ P,P,P,P };
    std::vector<double> i_pid_Lst_{ I,I,I,I };
    std::vector<double> d_pid_Lst_{ D,D,D,D };
    std::vector<double> p_pid_Rst_{ P,P,P,P };
    std::vector<double> i_pid_Rst_{ I,I,I,I };
    std::vector<double> d_pid_Rst_{ D,D,D,D };
};

struct robotStateMachine
{

    /**
     *  velHgtCmd_flag, in walking:
     *      0: filter specified velocity value, no height-changing, just for testing
     *      1: built-in steps-drived Velocity Tracking (Demo), no height-changing
     *      2: Remoter send 'Vx_cmd' for Push Recovery (Demo)
     *      5: Remoter send 'Vx_cmd' & 'H_flag_cmd'
     */
    int velHgtCmd_flag{1};

    /**
     *  vel_flag, in walking:
     *      0: v_est = cDot_Com_S;
     *      1: v_est = cDot_U_S;
     */
    int vel_flag{1};



    // ========================================= DO NOT MODIFY BELOW ====================================

    /**
     *  behavior_flag: old-ctrlStatus:
     *      0: hold in air ( both legs keep the ZERO state ) ;
     *      1: try to Stand-up in place ( in preparation for subsequent case : "walking" and "test in air" ) ;
     *      2: test in air ( both legs are always in swing phase ) ;
     *      3: walking ;
     *  set value by calling 'setBehaviorFlag(int)'
     */
    int behavior_flag{0};


    /**
     *  actuatorMode_flag :
     *      0: stanceLeg in Torque-Mode, swingLeg in Torque-Mode
     *      1: stanceLeg in Torque-Mode, swingLeg in PVT-PID-Mode
     *  set value by calling 'setActuatorModeFlag(int)'
     */
    int actuatorMode_flag{0};

    /**
     *  standUp_flag:
     *      0: prepare for walk, "s_init = 0.0"
     *      1: prepare for walk, "s_init = 0.5"
     *      2: prepare for stand-balance-control
     *  set value by calling 'setStandUpFlag(int)'
     */
    int standUp_flag{0};

    /**
     *  init_flag:
     *      0: "s_init = 0.0" for walking
     *      1: "s_init = 0.5" for walking
     *  set value by calling 'setInitFlag(int)'
     */
    int init_flag{0};

    /**
     *  massRatio_flag: leg-to-upper-body mass ratio = M_legs / M_upperBody
     *      0: 10.5% , without counterweight;
     *      1: 40% , mass_shin = 1.5kg
     *      2: 40% , mass_thigh = 1.5kg
     *      3: 70% , mass_shin = 1.5kg, mass_thigh = 1.5kg
     *  set value by calling constructor 'newController(int)'
     */
    int massRatio_flag{0};


    /**
     *  rom_flag : the ROM (Reduced Order Model) which prediction choose
     *      0: IROS2019 paper, ddx = 0 in DSP (if Td is small enough, this is ok);
     *      1: ddx != 0 in DSP, is real-like and smooth (in simulation, this is good);
     *      2: base on '0', and modify the kv_v, kp_v with the real Robot situation (filter, model error, and so on ...);       better ~
     */
    int rom_flag{2};

    /**
     *  velTrk_flag, in walking:
     *      0: 'single-gait-limit-cycle', it's unnecessary for h-LIP model, as the 'sigma' is only depended on 'Ts';
     *      1: 'velocity-tracking' , utilize 'kp_v * v_err', 'kv_v = eta(vx_est)';
     *      2: 'velocity-tracking' , utilize 'kp_v * v_err', 'kv_v = eta(vx_full_pre)';
     *      3: 'velocity-tracking' , utilize 'kp_v * v_err', 'kv_v = eta(vx_tgt)';      for "speed tracking Demo" & "Mario-Squat-Rise Demo"
     *  Do not set 'velTrk_flag = 0', at the beginning.
     */
    int velTrk_flag{3};

    /**
     *  horizontalCtrl_flag, in walking:
     *      0: x-d , xDot-d set value as x-now, xDot-now;
     *      1: x-d , xDot-d set value as hLIP prediction;
     */
    int horizontalCtrl_flag{0};

    /**
     *  forcePfRef_flag :
     *      0: rtr.forcePfRef = Eigen::Vector4d::Zero();   minimize force of contact, better~
     *      1: rtr.forcePfRef << rd.fc_grf_R_S_ff, rd.fc_grf_L_S_ff;
     */
    int forcePfRef_flag{0};

    /**
     *  JcTruncation_flag :
     *      true: Jc is truncated;              bad!
     *      false: Jc is NOT truncated;         better~
     */
    bool JcTruncation_flag{false};

    /**
     *  debug_flag : for Debug
     *      0: not debug, the Real-Robot case;
     *      1: debug mode, use force-sensor to estimate GRF, in simulation;
     */
    int debug_flag{1};

    /**
     *  circular_flag : state estimation for circular motion in 2D
     *      0 : like simulation, no circular motion;
     *      1 : deal with the circular motion error;         essential ~
     */
    int circular_flag{0};

    //* ------------------------------------------------------------------------------------------*//

    int firstStance_flag{1};                        // 1: left; -1:right
    int innerLeg_flag{1};                           // 1: left is inner; -1:right is inner

    int remark_flag{0};                             // 1: remark the leg labels 'st'-'sw'
    int state{1};                                   // walking state, 1:SSP; 2:DSP
    int stanceLeg{firstStance_flag};                // mark which leg being at stance phase, 1 : left ;  -1:  right
    int stanceLeg_lastStatus{firstStance_flag};     // mark 'stanceLeg' of the last status, when change 'behavior_flag'

    int emergency_flag{0};                          // 1: set all PVT-PID to be ZERO.
    double q_Knee_Margin{DEG2RAD*1};                // Emergency / Mechanical joint limits (rad)

    // time constant
    double Ts{0.35};                                // nominal SSP duration (sec), 0.35s
    double Td{0.02};                                // nominal DSP duration (sec), 0.02s

    double timeCs{0.0};                             // time of CS (Control System)
    int tick{0};                                    // the tick-tack for time accumulation, Re-initialize tick-tack at every moment of changing the control-status
    double Time{0.0};                               // run time (sec), the 'T'
    double t{0.0};                                  // passed-time (sec), since last Phase switch (PS-)
    double tt{0.0};                                 // passed-time (sec), since last TD
    double s{0.0};                                  // the time-variant parameter: s = clamp(t/Ts, 0, 1)
    double ss{0.0};                                 // the time-variant parameter: ss = clamp(tt/Ts, 0, 1)

    int steps_total{0};                             // total steps

    int TD_flag{0};                                 // set to be '1', when the first TD happens
    double s_td{0.0};                               // the s value at the moment of current touchdown
    double t_td{Ts + Td};                           // the t value at the moment of current touchdown

    double s_R{0.0}, s_L{0.0};                      // the Fuzzy Sets parameters of vGRF
    double s_st{0.0}, s_sw{0.0};                    // the Fuzzy Sets parameters of vGRF
    double s_sw_pre{0.0};                           // previous s_sw: the scale s of vGRF of swing leg; Update after Detection of TD & LO

    // ---------- for 'test in air' ---------
    double Ts_testInAir{0.35};                      // Ts, in air testing
    double totalTestTime{10};
    int init_flagTestInAir{1};                      // tuning case, set to be "1" for the case "s_init = 0.5"
    int steps_test{0};
    // -------- end of 'test in air' --------

    // ---------- for 'try to stand' ---------
    double Hn_tryStand{0.44};                       // nominal body height, for 'walking', 0.44m by default
    double t_interpolating{5};                      // interpolating duration, for starting-state to Standing
    double totalStandTime{15};
    // 0: normal walking, has torsoCtrl(); 1: test Fx_*Leg_offset, no torsoCtrl(); 2: test Fz_*Leg_offset, no torsoCtrl();set value by function - 'setTestOffsetFlag(int)'
    int testOffset_flag{0};
    // -------- end of 'try to stand' --------

    // ---------- for 'test squat' ---------
    int squat1D_flag{0};                            // 0: normal walking, by default; 1: test squat in 1D. set value by function - 'setSquatFlag(int)'
    double Ts_testSquat{1.0};                       // T (s), squat testing, not T_interpolating
    double Hn_swingSquat{0.30};
    double H_lastSquatEnd{0.44};
    int steps_squat{0};
    int steps_squatStop{8};
    // -------- end of 'test in air' --------

};


struct robotState
{
    int nJg{11};

    // Leg zero position matches qZero in joint coordinate, it is CONSTANT, FR-RR-FL-RL order, "frrl"
    std::vector<double> qZero_frrl{ 4.3915, 1.8917, 1.8917, 4.3915 };

    Eigen::VectorXd q_ = Eigen::VectorXd::Zero(nJg);                    // nJg*1
    Eigen::VectorXd qDot_ = Eigen::VectorXd::Zero(nJg);                 // nJg*1
    Eigen::VectorXd qDDot_ = Eigen::VectorXd::Zero(nJg);                // nJg*1
    Eigen::VectorXd u_ = Eigen::VectorXd::Zero(nJg);                    // nJg*1

    // Upper-body pitch
    double pitch{0.0}, pitchDot{0.0};                   // actual value; used in Kinematics (FK & IK)
    double pitch_0{0.0}, pitchDot_0{0.0};
    // Ry: i.e. R_WB, here, Rotation Transformation of pitch, in 2D, for Cartesian frame computing
    Eigen::Matrix2d Ry = Eigen::Matrix2d::Zero(), RyDot = Eigen::Matrix2d::Zero();

    Eigen::Matrix2d Jc_R = Eigen::Matrix2d::Zero(), Jc_L = Eigen::Matrix2d::Zero();
    Eigen::Matrix2d Jc_2bar_R = Eigen::Matrix2d::Zero(), Jc_2bar_L = Eigen::Matrix2d::Zero();

    Eigen::Vector2d q_R_A = Eigen::Vector2d::Zero(), q_L_A = Eigen::Vector2d::Zero();
    Eigen::Vector2d c_R_B = Eigen::Vector2d::Zero(), c_L_B = Eigen::Vector2d::Zero();
    Eigen::Vector2d c_R_W = Eigen::Vector2d::Zero(), c_L_W = Eigen::Vector2d::Zero();
    Eigen::Vector2d c_R_G = Eigen::Vector2d::Zero(), c_L_G = Eigen::Vector2d::Zero();
    Eigen::Vector2d c_R_S = Eigen::Vector2d::Zero(), c_L_S = Eigen::Vector2d::Zero();
    Eigen::Vector2d c_U_S = Eigen::Vector2d::Zero(), c_Com_S = Eigen::Vector2d::Zero();
    Eigen::Vector2d qDot_R_A = Eigen::Vector2d::Zero(), qDot_L_A = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDot_R_B = Eigen::Vector2d::Zero(), cDot_L_B = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDot_R_W = Eigen::Vector2d::Zero(), cDot_L_W = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDot_R_G = Eigen::Vector2d::Zero(), cDot_L_G = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDot_R_S = Eigen::Vector2d::Zero(), cDot_L_S = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDot_U_S = Eigen::Vector2d::Zero(), cDot_Com_S = Eigen::Vector2d::Zero();

    Eigen::Vector2d c_Com_W = Eigen::Vector2d::Zero(), cDot_Com_W = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDDot_Com_S = Eigen::Vector2d::Zero(), cDot_Com_S_pre = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDDot_U_S = Eigen::Vector2d::Zero(), cDot_U_S_pre = Eigen::Vector2d::Zero();

    Eigen::Vector2d c_R_W_0 = Eigen::Vector2d::Zero(), c_L_W_0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d c_R_G_0 = Eigen::Vector2d::Zero(), c_L_G_0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d c_R_S_0 = Eigen::Vector2d::Zero(), c_L_S_0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d c_U_S_0 = Eigen::Vector2d::Zero(), c_Com_S_0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDot_R_W_0 = Eigen::Vector2d::Zero(), cDot_L_W_0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDot_R_G_0 = Eigen::Vector2d::Zero(), cDot_L_G_0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDot_R_S_0 = Eigen::Vector2d::Zero(), cDot_L_S_0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDot_U_S_0 = Eigen::Vector2d::Zero(), cDot_Com_S_0 = Eigen::Vector2d::Zero();

    Eigen::Vector2d c_R_W_td = Eigen::Vector2d::Zero(), c_L_W_td = Eigen::Vector2d::Zero();
    Eigen::Vector2d c_R_G_td = Eigen::Vector2d::Zero(), c_L_G_td = Eigen::Vector2d::Zero();
    Eigen::Vector2d c_R_S_td = Eigen::Vector2d::Zero(), c_L_S_td = Eigen::Vector2d::Zero();
    Eigen::Vector2d c_U_S_td = Eigen::Vector2d::Zero(), c_Com_S_td = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDot_R_W_td = Eigen::Vector2d::Zero(), cDot_L_W_td = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDot_R_G_td = Eigen::Vector2d::Zero(), cDot_L_G_td = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDot_R_S_td = Eigen::Vector2d::Zero(), cDot_L_S_td = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDot_U_S_td = Eigen::Vector2d::Zero(), cDot_Com_S_td = Eigen::Vector2d::Zero();

    Eigen::Vector2d u_R_A = Eigen::Vector2d::Zero(), u_L_A = Eigen::Vector2d::Zero();

    // GRF, applied from ground to foot
    Eigen::Vector2d fc_grf_R_S = Eigen::Vector2d::Zero(), fc_grf_L_S = Eigen::Vector2d::Zero();
    Eigen::Vector2d fc_grf_st_S = Eigen::Vector2d::Zero(), fc_grf_sw_S = Eigen::Vector2d::Zero();

    // mass
    double massTotal{0.0};

    // CoM position & velocity
    Eigen::Vector2d p_Center_0 = Eigen::Vector2d::Zero(), v_Center_0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d p_Center = Eigen::Vector2d::Zero(), v_Center = Eigen::Vector2d::Zero();
    Eigen::Vector2d deltaC_foot2_minus_foot1 = Eigen::Vector2d::Zero();

    double x_est_total_pre{0.0};                    // estimated position of x, till last LO, total travel
    double x_est_total{0.0};                        // estimated position of x, total travel
    double z_est_total_pre{0.0};                    // estimated position of z, till last LO, total travel
    double z_est_total{0.0};                        // estimated position of z, total travel

    double vx_inner{0.0};                           // FK-computed velocity of x on inner vertical-plane, when inner leg is supporting
    double vx_outer{0.0};                           // FK-computed velocity of x on inner vertical-plane, when outer leg is supporting
    double vx_est{0.0};                             // estimated velocity of x
    double vx_est_pre{0.0};                         // estimated velocity of x, at last Phase Switch (PS-)
    double vx_est_nf{0.0};                          // estimated velocity of x, no filter
    double vx_est_nf_pre{0.0};                      // estimated velocity of x, no filter, at last Phase Switch (PS-)
    double vz_est{0.0};                             // estimated velocity of z
    double vz_est_nf{0.0};                          // estimated velocity of z, no filter

    double T_full_pre{0.0};                         // the moment of Time, when last Phase Switch(PS-) happens; (dx_full computation)
    double x_est_full_pre{0.0};                     // estimated position, from PS-(last PS) to PS+(new PS)
    double vx_full_pre{0.0};                        // average velocity of previous full-step, from PS- to PS+
    double vx_full_pre_pre{0.0};                    // average velocity of previous previous full-step, from PS-- to PS-
    double vx_double{0.0};                          // average velocity of previous double full-step
    double z_est_full_pre{0.0};                     // estimated position, from PS-(last PS) to PS+(new PS)
    double vz_full_pre{0.0};                        // average velocity of previous full-step, from PS- to PS+

    // odometry the origin of body frame 'B' w.r.t. the ground inertial frame 'I'
    Eigen::Vector2d pos_odo = Eigen::Vector2d::Zero();
    Eigen::Vector2d pos_odo_lastLO = Eigen::Vector2d::Zero();   // the odo value at last LO
    Eigen::Vector2d pos_odo_0 = Eigen::Vector2d::Zero();        // the initial value at 'tick == 0'
    Eigen::Vector2d vel_odo_preStep = Eigen::Vector2d::Zero();  // average velocity of previous full-step, from last LO to new LO

    double vx_preStep{0.};                          // vel_odo_preStep(0) OR vx_full_pre

    // hIPM states
    double x_init_SSP{0.0};
    double vx_init_SSP{0.0};
    double xE_SSP{0.0};
    double vxE_SSP{0.0};
    double xE_DSP{0.0};
    double vxE_DSP{0.0};

};

struct robotDesired
{
    // remoter command
    double vx_cmd{0.0};                     // the Velocity of x Commanded (m/s)
    int H_delta_cmd{0};                     // the Change-Hb flag Commanded, 0: Hb = 0.44m; 1: Hb = 0.34m; only work in "velHgtCmd_flag = 4 or 5"

    double vx_tgt{0.0};                     // target velocity of x

    double H_tgt{0.0};                      // target height of CoM or UpperBody (m)
    double H_tgt_pre{0.0};                  // target height of CoM or UpperBody in last step
    double H_ret{0.06};                     // nominal swing Leg retraction height (m)

    double X_offset{0.1};                   // standBalance (m)

    int deltaH_signal{0};                   // 0: H stay constant; 1: prepare for H changing; 2: carry out H changing
    double deltaH{0.0};                     // (m)

    double H_tgt_nestStep{H_tgt};           // nominal body height in nest step (m);
    double H_ret_nestStep{H_ret};           // nominal swing Leg retraction height in nest step (m)

    double Ts_nextStep{0.35};               // nominal SSP duration in nest step (sec)
    double Td_nextStep{0.02};               // nominal DSP duration in nest step (sec)

    double pitch_d{0.0}, pitchDot_d{0.0}, pitchDDot_d{0.0};   // desired value, if 'x_TorsoCom_U != 0.0', pitch_d = -theta_c
    Eigen::Matrix2d Ry_d = Eigen::Matrix2d::Zero(), RyDot_d = Eigen::Matrix2d::Zero();

    // ------------------------------- results -------------------------------------------------
    int nJg{11};

    Eigen::Vector2d c_R_B_d = Eigen::Vector2d::Zero(), c_L_B_d = Eigen::Vector2d::Zero();
    Eigen::Vector2d c_R_W_d = Eigen::Vector2d::Zero(), c_L_W_d = Eigen::Vector2d::Zero();
    Eigen::Vector2d c_R_G_d = Eigen::Vector2d::Zero(), c_L_G_d = Eigen::Vector2d::Zero();
    Eigen::Vector2d c_R_S_d = Eigen::Vector2d::Zero(), c_L_S_d = Eigen::Vector2d::Zero();
    Eigen::Vector2d c_U_S_d = Eigen::Vector2d::Zero(), c_Com_S_d = Eigen::Vector2d::Zero();

    Eigen::Vector2d cDot_R_B_d = Eigen::Vector2d::Zero(), cDot_L_B_d = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDot_R_W_d = Eigen::Vector2d::Zero(), cDot_L_W_d = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDot_R_G_d = Eigen::Vector2d::Zero(), cDot_L_G_d = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDot_R_S_d = Eigen::Vector2d::Zero(), cDot_L_S_d = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDot_U_S_d = Eigen::Vector2d::Zero(), cDot_Com_S_d = Eigen::Vector2d::Zero();

    Eigen::Vector2d cDDot_R_S_d = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDDot_L_S_d = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDDot_Com_S_d = Eigen::Vector2d::Zero();
    Eigen::Vector2d cDDot_U_S_d = Eigen::Vector2d::Zero();

    Eigen::Vector2d fc_grf_R_S_ff = Eigen::Vector2d::Zero(), fc_grf_L_S_ff = Eigen::Vector2d::Zero();

    // pid : reference, FR-RR-FL-RR order: fore->rear->right->left, "frrl"
    std::vector<double> p_pid_frrl{ 0.0, 0.0, 0.0, 0.0 };
    std::vector<double> i_pid_frrl{ 0.0, 0.0, 0.0, 0.0 };
    std::vector<double> d_pid_frrl{ 0.0, 0.0, 0.0, 0.0 };

    // Intermediate Variables for PID-transition
    PidParam PP_;     // the nominal control parameters of PV-PID
    // pvt-pid Gains in case of 'LeftLeg is stanceLeg', i.e. 'stanceLeg == 1', "frrl"
    std::vector<double> p_pid_Lst{ 0.0, 0.0, 0.0, 0.0 };
    std::vector<double> i_pid_Lst{ 0.0, 0.0, 0.0, 0.0 };
    std::vector<double> d_pid_Lst{ 0.0, 0.0, 0.0, 0.0 };
    // pvt-pid Gains in case of 'RightLeg is stanceLeg', i.e. 'stanceLeg == -1', "frrl"
    std::vector<double> p_pid_Rst{ 0.0, 0.0, 0.0, 0.0 };
    std::vector<double> i_pid_Rst{ 0.0, 0.0, 0.0, 0.0 };
    std::vector<double> d_pid_Rst{ 0.0, 0.0, 0.0, 0.0 };

};

struct robotTaskRef
{
    // reference
    Eigen::Vector3d hgDotRef = Eigen::Vector3d::Zero();
    Eigen::Vector3d qfDDotRef = Eigen::Vector3d::Zero();
    Eigen::Vector4d cPfDDotRef = Eigen::Vector4d::Zero();
    Eigen::Vector4d forcePfRef = Eigen::Vector4d::Zero();
    Eigen::Vector4d loopRef = Eigen::Vector4d::Zero();
    Eigen::Vector4d torOptPreRef = Eigen::Vector4d::Zero();
    Eigen::Vector4d fcOptPreRef = Eigen::Vector4d::Zero();

    // optimization-result
    Eigen::Vector3d hgDot_opt = Eigen::Vector3d::Zero();
    Eigen::Vector3d qfDDot_opt = Eigen::Vector3d::Zero();
    Eigen::Vector4d cPfDDot_opt = Eigen::Vector4d::Zero();

    // Result cDDot of internal constraints : closed-loops
    Eigen::Vector4d xRzRxLzL_ddot_ClosedLoop = Eigen::Vector4d::Zero();

    // Result of QP-WBC
    int nJg{11};
    int nFc{4};
    int nJa{4};
    Eigen::VectorXd qDDot_opt = Eigen::VectorXd::Zero(nJg);         // nJg*1
    Eigen::VectorXd fcPf_opt = Eigen::VectorXd::Zero(nFc);          // nFc*1
    Eigen::VectorXd tauA_opt = Eigen::VectorXd::Zero(nJa);          // nJa*1

    // qDot_A_integral : integral of qDDot_opt of actuated joints
    Eigen::Vector2d qDot_A_sw_integral = Eigen::Vector2d::Zero(2);          // nJa*1

    // command
    Eigen::VectorXd tauA_cmd = Eigen::VectorXd::Zero(nJa);          // nJa*1

    Eigen::Vector2d u_R_A_d = Eigen::Vector2d::Zero(), u_L_A_d = Eigen::Vector2d::Zero();
    Eigen::Vector2d q_R_A_d = Eigen::Vector2d::Zero(), q_L_A_d = Eigen::Vector2d::Zero();
    Eigen::Vector2d qDot_R_A_d = Eigen::Vector2d::Zero(), qDot_L_A_d = Eigen::Vector2d::Zero();

    Eigen::VectorXd q_d_ = Eigen::VectorXd::Zero(nJg);                    // nJg*1
    Eigen::VectorXd qDot_d_ = Eigen::VectorXd::Zero(nJg);                 // nJg*1

    /*
     *  simpleStatus:
     *      0:  QP was solved;
     *      1:  QP could not be solved within the given number of iterations;
     *      -1: QP could not be solved due to an internal error;
     *      -2: QP is infeasible and thus could not be solved;
     *      -3: QP is unbounded and thus could not be solved;
     */
    int simpleStatus{0};        // 0: solve; 1: fail
    int nWSR_res{0};            // the number of working set recalculations actually performed
    double cputime_res{0.};     // the cputime actually performed
    double cost_opt{0.};        // QP cost function value

    double time_TcUpdate{0.};   // time-consuming of task and constraint update (ms)
    double time_RoDy{0.};       // time-consuming of robot dynamics update (ms)
    double time_QP{0.};         // time-consuming of QP slove (ms)
    double time_WBC{0.};        // time-consuming of WBC (ms)

    // reference joint states in "frrl" : FR->RR->FL->RR order: fore->rear->right->left, "frrl"
    std::vector<double> q_frrl_d = { 0.0, 0.0, 0.0, 0.0 };
    std::vector<double> qDot_frrl_d = { 0.0, 0.0, 0.0, 0.0 };
    std::vector<double> qDDot_frrl_d = { 0.0, 0.0, 0.0, 0.0 };
    std::vector<double> tauA_frrl_d = { 0.0, 0.0, 0.0, 0.0 };
    // reference joint states in Real-Robot, "rr" : (-)RR->(-)FR->(+)FL->(+)RR, (+/- is caused by human installation error)
    std::vector<double> q_rr_d = { 0.0, 0.0, 0.0, 0.0 };
    std::vector<double> qDot_rr_d = { 0.0, 0.0, 0.0, 0.0 };
    std::vector<double> qDDot_rr_d = { 0.0, 0.0, 0.0, 0.0 };
    std::vector<double> tauA_rr_d = { 0.0, 0.0, 0.0, 0.0 };
    // reference, Real-Robot frame, "rr"
    std::vector<double> p_pid_rr{ 0.0, 0.0, 0.0, 0.0 };
    std::vector<double> i_pid_rr{ 0.0, 0.0, 0.0, 0.0 };
    std::vector<double> d_pid_rr{ 0.0, 0.0, 0.0, 0.0 };

};

struct robotMessage
{
    robotStateMachine rsm;
    robotState rs;
    robotDesired rd;
    robotTaskRef rtr;
};


#endif  // ROBOT_MESSAGE_H
