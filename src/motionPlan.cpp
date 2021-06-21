//
// Created by jun on 2020-8-19.
//

#include "motionPlan.h"

//motionPlan::motionPlan(){
//}

//motionPlan::~motionPlan(){
//}

bool motionPlan::parseCmd(double Vx_cmd, int H_flag_cmd, robotStateMachine &_rsm, const robotState & _rs, robotDesired &_rd){

    // ----------------------- Remoter Command Read -----------------------------
    // only update Remoter Command at the moment of "Lift-Off" or "s == 0", a new SSP beginning
    if (_rsm.s >= 0 - EPSILON && _rsm.s <= 0 + EPSILON){
        // get the CS (Control System) command of Velocity of x
        _rd.vx_cmd = Vx_cmd;
        if (_rsm.velHgtCmd_flag == 5 || _rsm.velHgtCmd_flag == 4){
            if (_rd.H_delta_cmd != H_flag_cmd && _rd.deltaH_signal == 0){
                // get the CS (Control System) command of Height-changing of body
                _rd.H_delta_cmd = H_flag_cmd;
                if(H_flag_cmd == -1){
                    _rd.deltaH_signal = 1;
                    _rd.deltaH = -0.08;
                }else if(H_flag_cmd == 0){
                    _rd.deltaH_signal = 1;
                    _rd.deltaH = 0.08;
                }else{
                    _rd.deltaH_signal = 0;
                    _rd.deltaH = 0.0;
                }
            }else{
                if (_rd.deltaH_signal == 1){
                    _rd.deltaH_signal = 2;
                    _rd.H_tgt_pre = _rd.H_tgt;
                    _rd.H_tgt = _rd.H_tgt + _rd.deltaH;
                }else if (_rd.deltaH_signal == 2){
                    _rd.deltaH_signal = 0;
                    _rd.deltaH = 0.0;               // reset deltaH
                }else{
                    _rd.deltaH = 0.0;               // reset deltaH
                }
            }
        }else{
            // deltaH_signal, for 'velHgtCmd_flag == 3'
            if (_rd.deltaH_signal == 1){
                _rd.deltaH_signal = 2;
                _rd.H_tgt_pre = _rd.H_tgt;
                _rd.H_tgt = _rd.H_tgt + _rd.deltaH;
            }else if (_rd.deltaH_signal == 2){
                _rd.deltaH_signal = 0;
                _rd.deltaH = 0.0;               // reset deltaH
            }else{
                _rd.deltaH = 0.0;               // reset deltaH
            }
        }

        // Ts,Td setting
        if (_rsm.Ts < _rd.Ts_nextStep - EPSILON || _rsm.Ts > _rd.Ts_nextStep + EPSILON){
            _rsm.Ts = _rd.Ts_nextStep;
        }
//        if (_rs.vx_preStep > 0.5 + EPSILON){
//            _rsm.Ts = 0.32;
//        }else {
//            if (_rsm.Ts < _rd.Ts_nextStep - EPSILON || _rsm.Ts > _rd.Ts_nextStep + EPSILON){
//                _rsm.Ts = _rd.Ts_nextStep;
//            }
//        }
    }
    // -------------------- end of Remoter Command Read ---------------------------

    // -------------- mark initial value ------------------
    if (_rsm.tick == 0){
        _rd.vx_tgt = 0.;
        if(_rsm.vel_flag == 0){
            H_tgt_0 = _rs.c_Com_S(1);
            Hdot_tgt_0 = _rs.cDot_Com_S(1);
        }else{
            H_tgt_0 = _rs.c_U_S(1);
            Hdot_tgt_0 = _rs.cDot_U_S(1);
        }
    }
    // ------------ end of mark initial value ---------------

    // ------------------------------- set the initial value of new DSP ---------------------------------------------
    // s == s_td, a new DSP beginning
    if (_rsm.TD_flag == 1 && _rsm.s >= _rsm.s_td - EPSILON && _rsm.s <= _rsm.s_td + EPSILON){

//        double z_sw_S_d_TD, zDot_sw_S_d_TD;
        plan::cubic_s(0.5, 1.0, _rd.H_ret, z_compensating, 0.0, vz_S_sw_TD_d, _rsm.s, 1/_rsm.Ts, z_sw_S_d_TD, zDot_sw_S_d_TD);
        if(_rsm.vel_flag == 0){
            z_Com_Snext_d_TD = _rd.c_Com_S_d(1) - z_sw_S_d_TD;
            zDot_Com_Snext_d_TD = _rd.cDot_Com_S_d(1) - zDot_sw_S_d_TD;
            // w.r.t. 'swing foot-point' i.e. 'the next stance foot-point' "Snext" in DSP, not set to huge
            deltaZ_Com_Snext_DSP_d = plan::clamp(_rd.H_tgt - z_Com_Snext_d_TD, -0.01, 0.01);
        }else{
            z_U_Snext_d_TD = _rd.c_U_S_d(1) - z_sw_S_d_TD;
            zDot_U_Snext_d_TD = _rd.cDot_U_S_d(1) - zDot_sw_S_d_TD;
            // w.r.t. 'swing foot-point' i.e. 'the next stance foot-point' "Snext" in DSP, not set to huge
            deltaZ_U_Snext_DSP_d = plan::clamp(_rd.H_tgt - z_U_Snext_d_TD, -0.01, 0.01);
        }

        // w.r.t. 'stance foot-point' "S" in DSP
        if (_rsm.s_td >= 1 - EPSILON){
            deltaZ_Com_S_DSP_d = 0.0;       // set to be -0.01 for down stairs/slope; -0.01
            deltaZ_U_S_DSP_d = 0.0;       // set to be -0.01 for down stairs/slope; -0.01
        }else{
//            deltaZ_Com_S_DSP_d = plan::clamp(deltaZ_foot2_minus_foot1, -0.01, 0.01);
            deltaZ_Com_S_DSP_d = 0.0;
            deltaZ_U_S_DSP_d = 0.0;
        }
    }
    // ----------------------------- end of set the initial value of new DSP -------------------------------------------

    // ---------------------------------- vx_tgt, deltaH, Ts_nextStep, Td_nextStep -------------------------------
    if (_rsm.velHgtCmd_flag == 0) {             // target velocity Filter 1st-Order
        _rd.vx_tgt = _rd.vx_tgt + sf_cmd * (_rd.vx_cmd - _rd.vx_tgt);
    } else if(_rsm.velHgtCmd_flag == 1){        // built-in, speed tracking 0.0-0.3-0.6-0.3-0.6-0.3-0.0
        int step_begin = 14;
        int SpR_cycle = 1;  // 6

        // 0.0-0.3-0.6-0.3-0.0
        if (_rsm.steps_total <= 4) {
            _rd.vx_tgt = 0.0;
        } else if (_rsm.steps_total > 4 && _rsm.steps_total <= step_begin){
            stepm = _rsm.steps_total - 4;
            _rd.vx_tgt = stepm*0.03 + 0.0;
        } else if (_rsm.steps_total >= step_begin + 50 * SpR_cycle) {
            stepm = _rsm.steps_total - (step_begin + 50 * SpR_cycle);
            if (stepm > 10){
                _rd.vx_tgt = 0.0;
            } else{
                _rd.vx_tgt = 0.3 - stepm * 0.03;
            }
        } else {
            steps = _rsm.steps_total - step_begin;
            stepm = steps - static_cast<int>(std::floor(steps/50))*50;

            if (stepm > 8 && stepm <= 14){
                _rd.vx_tgt = (stepm - 8)*0.05 + 0.3;
            }else if (stepm > 14 && stepm <= 26){
                _rd.vx_tgt = 0.6;
            }else if (stepm > 26 && stepm <= 38){
                _rd.vx_tgt = 0.6 - (stepm - 26)*0.025;
            }else if (stepm > 38){
                _rd.vx_tgt = 0.3;
            }
        }

        // 0.0-0.4-0.7-0.4-0.0
//        if (_rsm.steps_total <= 10) {
//            _rd.vx_tgt = 0.0;
//        } else if (_rsm.steps_total > 10 && _rsm.steps_total <= step_begin){
//            stepm = _rsm.steps_total - 10;
//            _rd.vx_tgt = stepm*0.04 + 0.0;
//        } else if (_rsm.steps_total >= step_begin + 50 * SpR_cycle) {
//            stepm = _rsm.steps_total - (step_begin + 50 * SpR_cycle);
//            if (stepm > 10){
//                _rd.vx_tgt = 0.0;
//            } else{
//                _rd.vx_tgt = 0.4 - stepm * 0.04;
//            }
//        } else {
//            steps = _rsm.steps_total - step_begin;
//            stepm = steps - static_cast<int>(std::floor(steps/50))*50;

//            if (stepm > 8 && stepm <= 14){
//                _rd.vx_tgt = (stepm - 8)*0.05 + 0.4;
//            }else if (stepm > 14 && stepm <= 26){
//                _rd.vx_tgt = 0.7;
//            }else if (stepm > 26 && stepm <= 38){
//                _rd.vx_tgt = 0.7 - (stepm - 26)*0.025;
//            }else if (stepm > 38){
//                _rd.vx_tgt = 0.4;
//            }
//        }

        // go to Remoter!
        if(_rsm.steps_total >= step_begin + 50 * SpR_cycle + 15){
            _rsm.velHgtCmd_flag = 5;
        }
    } else if(_rsm.velHgtCmd_flag == 2){        // Remoter  OR  Push Recovery
        _rd.vx_tgt = plan::clamp(_rd.vx_cmd, _rs.vx_preStep - 0.2, _rs.vx_preStep + 0.2);       // +-0.1 , the smaller the more stable
        _rd.vx_tgt = plan::clamp(_rd.vx_tgt, -0.6, 0.6);
    } else if(_rsm.velHgtCmd_flag == 3){        // built-in, hIPM-stepping & deltaH
        int DeltaH_cycle = 2;
        int Num_StepInPlace = 6;
        int Step_VelocityReady = Num_StepInPlace + 2;

        // vx_tgt
        if (_rsm.steps_total <= Num_StepInPlace){
            _rd.vx_tgt = 0.0;
        }else if (_rsm.steps_total > Num_StepInPlace && _rsm.steps_total <= Step_VelocityReady){
            stepm = _rsm.steps_total - Num_StepInPlace;
            if(Step_VelocityReady > Num_StepInPlace){
                _rd.vx_tgt = stepm*0.3/(Step_VelocityReady - Num_StepInPlace) + 0.0;
            }
        }else if (_rsm.steps_total >= Step_VelocityReady + 31 * DeltaH_cycle) {
            stepm = _rsm.steps_total - (Step_VelocityReady + 31 * DeltaH_cycle);
            if (stepm > 10){
                _rd.vx_tgt = 0.0;
            } else{
                _rd.vx_tgt = 0.3 - stepm * 0.03;
            }
        }else{
            // velocity
            _rd.vx_tgt = 0.3;
            // delta-H
            steps = _rsm.steps_total - Step_VelocityReady;
            stepm = steps - static_cast<int>(std::floor(steps/31))*31;
            if(stepm == 10){
                _rd.deltaH_signal = 1;
                _rd.deltaH = -0.08;
            }else if(stepm == 25){
                _rd.deltaH_signal = 1;
                _rd.deltaH = 0.08;
            }
        }
    } else if(_rsm.velHgtCmd_flag == 4){         // built-in, Mario-Squat-Rise
        int Num_StepInPlace = 6;
        int Step_VelocityReady = Num_StepInPlace + 5;
        int Step_VelTrk_begin = Step_VelocityReady + 45;
        int Step_VelTrk_end = Step_VelTrk_begin + 33;
        int Step_goToRemoter = Step_VelTrk_end + 5;

        // vx_tgt
        if (_rsm.steps_total <= Num_StepInPlace){
            _rd.vx_tgt = 0.0;
        }else if (_rsm.steps_total > Num_StepInPlace && _rsm.steps_total <= Step_VelocityReady){
            stepm = _rsm.steps_total - Num_StepInPlace;
            if(Step_VelocityReady > Num_StepInPlace){
                _rd.vx_tgt = stepm*0.3/(Step_VelocityReady - Num_StepInPlace) + 0.0;
            }
        }else if (_rsm.steps_total > Step_VelocityReady && _rsm.steps_total <= Step_VelTrk_begin) {
            _rd.vx_tgt = 0.3;
        }else if(_rsm.steps_total > Step_VelTrk_begin && _rsm.steps_total <= Step_VelTrk_end){
            stepm = _rsm.steps_total - Step_VelTrk_begin;
            if (stepm > 0 && stepm <= 6){
                _rd.vx_tgt = (stepm - 0)*0.05 + 0.3;
            }else if (stepm > 6 && stepm <= 18){
                _rd.vx_tgt = 0.6;
            }else if (stepm > 18 && stepm <= 26){
                _rd.vx_tgt = 0.6 - (stepm - 18)*0.0375;
            }else if (stepm > 26){
                _rd.vx_tgt = 0.3;
            }
        }else if(_rsm.steps_total > Step_VelTrk_end && _rsm.steps_total <= Step_goToRemoter){
            stepm = _rsm.steps_total - Step_VelTrk_end;
            if (stepm > 5){
                _rd.vx_tgt = 0.0;
            } else{
                _rd.vx_tgt = 0.3 - stepm * 0.06;
            }
        }else{
            _rsm.velHgtCmd_flag = 5;
        }
    } else if(_rsm.velHgtCmd_flag == 5){         // Remoter  OR  stable walking tuning: vA = 0.3m/s, vA = 0.6m/s
        _rd.vx_tgt = plan::clamp(_rd.vx_cmd, _rs.vx_preStep - 0.1, _rs.vx_preStep + 0.1);       // _rs.vx_preStep +- 0.1
        _rd.vx_tgt = plan::clamp(_rd.vx_tgt, - 0.5, 0.5);       // cmd_max = 0.5m/s
    }
    // Ts_nextStep, Td_nextStep
    if (_rd.vx_tgt > 0.6 + EPSILON){            // TODO: add " || vx_preStep > 0.6 + EPSILON "
        _rd.Ts_nextStep = 0.32;
    }else {
        _rd.Ts_nextStep = 0.35;
    }
    // ------------------------------ end of vx_tgt, deltaH, Ts_nextStep, Td_nextStep -------------------------------

    return true;
}

bool motionPlan::hlipStepPlan(const robotStateMachine & _rsm, const robotState & _rs, robotDesired & _rd){
    // ----------------------------- prediction of x_G_step -----------------------------------

    // predict States at 'TouchDown'
//    xE_SSP_predict = _rs.p_Center(0);
//    VxE_SSP_predict = _rs.vx_est;
    plan::planLipForward(_rsm.t, _rsm.Ts, _rs.p_Center(0), _rs.vx_est, _rs.p_Center(1), xE_SSP_predict, VxE_SSP_predict);

    // for deltaH
    if (_rd.deltaH_signal == 1){
        VxA_d_predict = _rd.vx_tgt + 1.0*_rd.deltaH;
    }else if (_rd.deltaH_signal == 2){
        VxA_d_predict = _rd.vx_tgt;
    }else{
        VxA_d_predict = _rd.vx_tgt;
    }

    // params
    H_predict = _rd.H_tgt;          // no matter 'vel_flag'
    Ts_predict = _rd.Ts_nextStep;
    Td_predict = _rd.Td_nextStep;
    if (_rsm.rom_flag == 1){
        plan::get_HIPM_P1_Params_Rom1(GRAVITY, H_predict, Ts_predict, Td_predict, lambda_predict, sigma_predict, Kp_star_predict, xi_predict);
    }else if (_rsm.rom_flag == 0){
        plan::get_HIPM_P1_Params_Rom0(GRAVITY, H_predict, Ts_predict, Td_predict, lambda_predict, sigma_predict, Kp_star_predict, xi_predict);
    }else {
        plan::get_HIPM_P1_Params_Rom0(GRAVITY, H_predict, Ts_predict, Td_predict, lambda_predict, sigma_predict, Kp_star_predict, xi_predict);
    }
    VxE_SSP_d_predict = xi_predict * VxA_d_predict;

    // velocity control by stepping
    if (_rsm.rom_flag == 1){
        plan::get_HIPM_P1_VelCtrl_Rom1(xE_SSP_predict, VxE_SSP_predict, VxE_SSP_d_predict, Kp_star_predict, lambda_predict, sigma_predict, Ts_predict, Td_predict, H_predict,
                                    x_S_step_, x_G_step_, x_step_DSP_, x_step_SSP_, delta_vx_DSP_);
        // 20200417, trick for 'rom_flag == 1'; But, I argue the reasonable trick should be modifying the feedforward item: kv_v = eta*(1/sigma).
        x_G_step_amended = x_G_step_ + rho/lambda_predict*(VxE_SSP_predict - VxE_SSP_d_predict);

    }else if (_rsm.rom_flag == 0){
        plan::get_HIPM_P1_VelCtrl_Rom0(xE_SSP_predict, VxE_SSP_predict, VxE_SSP_d_predict, Kp_star_predict, lambda_predict, sigma_predict, Ts_predict, Td_predict, H_predict,
                                       x_S_step_, x_G_step_, x_step_DSP_, x_step_SSP_, delta_vx_DSP_);
        kp_v = zeta_v * Kp_star_predict;
        x_G_step_amended = x_G_step_ + (kp_v - Kp_star_predict)*(VxE_SSP_predict - VxE_SSP_d_predict);

    }else if (_rsm.rom_flag == 2){
        // TODO: function ----> plan::get_HIPM_P1_VelCtrl_Rom0_modified(), maybe not.
        plan::get_HIPM_P1_VelCtrl_Rom0(xE_SSP_predict, VxE_SSP_predict, VxE_SSP_d_predict, Kp_star_predict, lambda_predict, sigma_predict, Ts_predict, Td_predict, H_predict,
                                       x_S_step_, x_G_step_, x_step_DSP_, x_step_SSP_, delta_vx_DSP_);
        if (_rsm.velTrk_flag == 0){     // SGLC, no use
            kv_v = Eta_v*(1/sigma_predict);
            kp_v = 0.0;
        }else{                          // normal

            if (_rd.H_tgt >= 0.4){
//                plan::cal_eta_order1(0.0, 1.3, 0.6, 1.27, _rs.vx_est, _rs.vx_preStep, _rd.vx_tgt, _rsm.velTrk_flag, eta_v);   // ROM-OSC
//                if(_rsm.velHgtCmd_flag == 2){   // push recovery
//                    plan::cal_eta_order1(0.0, 1.14, 0.6, 1.08, _rs.vx_est, _rs.vx_preStep, _rd.vx_tgt, _rsm.velTrk_flag, eta_v);    // eta =1.11 for v=0.3; eta=1.08 for v=0.6
//                }else {
//                    plan::cal_eta_order1(0.0, 1.12, 0.6, 1.10, _rs.vx_est, _rs.vx_preStep, _rd.vx_tgt, _rsm.velTrk_flag, eta_v);    // eta =1.11
//                }
                plan::cal_eta_order1(0.0, 1.14, 0.6, 1.08, _rs.vx_est, _rs.vx_preStep, _rd.vx_tgt, _rsm.velTrk_flag, eta_v);    // eta =1.11 for v=0.3; eta=1.08 for v=0.6
            }else{
                plan::cal_eta_order1(0.0, 1.47, 0.6, 1.47, _rs.vx_est, _rs.vx_preStep, _rd.vx_tgt, _rsm.velTrk_flag, eta_v);    // ROM-OSC
            }

            // for webots simulation
            if (_rsm.debug_flag == 1){
                if (_rsm.vel_flag == 0){
                    if (_rsm.massRatio_flag == 1){
                        // fake
//                        plan::cal_eta_order1(0.0, 1.4, 0.6, 1.37, _rs.vx_est, _rs.vx_preStep, _rd.vx_tgt, _rsm.velTrk_flag, eta_v);
                        // real
                        plan::cal_eta_order1(0.0, 1.295, 0.6, 1.39, _rs.vx_est, _rs.vx_preStep, _rd.vx_tgt, _rsm.velTrk_flag, eta_v);
                    } else if (_rsm.massRatio_flag == 2){
                        plan::cal_eta_order1(0.0, 1.36, 0.6, 1.36, _rs.vx_est, _rs.vx_preStep, _rd.vx_tgt, _rsm.velTrk_flag, eta_v); // not tune well
                    } else if (_rsm.massRatio_flag == 3){
                        plan::cal_eta_order1(0.0, 1.36, 0.6, 1.36, _rs.vx_est, _rs.vx_preStep, _rd.vx_tgt, _rsm.velTrk_flag, eta_v);
                    } else{     // massRatio_flag == 0, by default.
                        plan::cal_eta_order1(0.0, 1.35, 0.6, 1.3, _rs.vx_est, _rs.vx_preStep, _rd.vx_tgt, _rsm.velTrk_flag, eta_v);
                    }
                }else{
                    if (_rsm.massRatio_flag == 1){
                        // fake
//                        plan::cal_eta_order1(0.0, 1.26, 0.6, 1.29, _rs.vx_est, _rs.vx_preStep, _rd.vx_tgt, _rsm.velTrk_flag, eta_v);
                        // real
                        plan::cal_eta_order1(0.0, 1.1, 0.6, 1.1, _rs.vx_est, _rs.vx_preStep, _rd.vx_tgt, _rsm.velTrk_flag, eta_v);
                    } else if (_rsm.massRatio_flag == 2){
                        plan::cal_eta_order1(0.0, 1.2, 0.6, 0.78, _rs.vx_est, _rs.vx_preStep, _rd.vx_tgt, _rsm.velTrk_flag, eta_v); // not tune well
                    } else if (_rsm.massRatio_flag == 3){
                        plan::cal_eta_order1(0.0, 1.3, 0.6, 0.82, _rs.vx_est, _rs.vx_preStep, _rd.vx_tgt, _rsm.velTrk_flag, eta_v);
                    } else{     // massRatio_flag == 0, by default.
                        plan::cal_eta_order1(0.0, 1.09, 0.6, 1.06, _rs.vx_est, _rs.vx_preStep, _rd.vx_tgt, _rsm.velTrk_flag, eta_v);
                    }
                }
            }
            // end for webots simulation

            // for Ts=0.35s, H=0.44m, 1/sigma = 0.1437, Kp_star = 0.0843
            kv_v = eta_v*(1/sigma_predict);
            kp_v = zeta_v * Kp_star_predict;
        }
        x_G_step_amended = x_step_DSP_
                           + kv_v*VxE_SSP_predict
                           + kp_v*((VxE_SSP_predict - VxE_SSP_d_predict > 0) - (VxE_SSP_predict - VxE_SSP_d_predict < 0))
                           * std::min(std::fabs(VxE_SSP_predict - VxE_SSP_d_predict), 0.3);     // for push recovery, 0.1
    }

    //
    x_G_step_d = x_G_step_amended;
    x_S_step_d = x_G_step_d + xE_SSP_predict;

    // -------------------------end of prediction of x_G_step -------------------------------

    return true;
}

bool motionPlan::walkTrajectoryPlan(const robotStateMachine &_rsm, const robotState &_rs, robotDesired &_rd){

    if (_rsm.state == 1){            // SSP
        // prediction of x_G_step
        hlipStepPlan(_rsm, _rs, _rd);

        // ----------------------------- Center of Mass plan ----------------------------------
        /*
         *  Motive : for 'hgDotRef'
         *      update  'c_Com_S_d', 'cDot_Com_S_d', 'cDDot_Com_S_d'
         */

        if (_rsm.tick != 0 && _rsm.s >= 0 - EPSILON && _rsm.s <= 0 + EPSILON){
            // the initial desired-H of new SSP
            H_tgt_0 = z_Com_Snext_d_TD + deltaZ_Com_Snext_DSP_d;
        }
        // TODO: x follow HLIP planning output
        // xDot_Com_S_d
        _rd.c_Com_S_d(0) = _rs.c_Com_S(0);
        _rd.cDot_Com_S_d(0) = _rs.cDot_Com_S(0);
        _rd.cDDot_Com_S_d(0) = _rs.cDDot_Com_S(0);
        // zDot_Com_S_d
        if (_rd.deltaH_signal == 1){
            plan::quintic_s(0.0, 0.5, H_tgt_0, _rd.H_tgt, 0.0, 0.0, 0.0, 0.0, _rsm.s, 1/_rsm.Ts, _rd.c_Com_S_d(1), _rd.cDot_Com_S_d(1), _rd.cDDot_Com_S_d(1));
        }else if(_rd.deltaH_signal == 2){
            plan::quintic_s(0.0, 0.9, H_tgt_0, _rd.H_tgt, 0.0, 0.0, 0.0, 0.0, _rsm.s, 1/_rsm.Ts, _rd.c_Com_S_d(1), _rd.cDot_Com_S_d(1), _rd.cDDot_Com_S_d(1));
        }else{
            plan::quintic_s(0.0, 0.5, H_tgt_0, _rd.H_tgt, 0.0, 0.0, 0.0, 0.0, _rsm.s, 1/_rsm.Ts, _rd.c_Com_S_d(1), _rd.cDot_Com_S_d(1), _rd.cDDot_Com_S_d(1));
        }
        // -------------------------- end of Center of Mass plan ------------------------------

        // -------------------------------- foot plan ---------------------------------------
        /*
         *  Motive : for 'cPfDDotRef'
         *      update  'c_R_S_d', 'cDot_R_S_d', 'cDDot_R_S_d'
         *              'c_L_S_d', 'cDot_L_S_d', 'cDDot_L_S_d'
         */

        double xDDot_sw_G_d;    // swing-foot desired acceleration w.r.t. CoM "G"

        if (_rsm.stanceLeg == 1){
            // swing foot plan
            plan::cubic_s(0.0, 1.0, _rs.c_R_G_0(0), x_G_step_d, _rs.cDot_R_G_0(0), -_rs.vx_est, _rsm.s, 1/_rsm.Ts, _rd.c_R_G_d(0), _rd.cDot_R_G_d(0), xDDot_sw_G_d);
//            _rd.c_R_S_d(0) = _rd.c_R_G_d(0) + _rd.c_Com_S_d(0);
//            _rd.cDot_R_S_d(0) = _rd.cDot_R_G_d(0) + _rd.cDot_Com_S_d(0);
//            _rd.cDDot_R_S_d(0) = xDDot_sw_G_d + _rd.cDDot_Com_S_d(0);
            _rd.c_R_S_d(0) = _rd.c_R_G_d(0) + _rs.c_Com_S(0);
            _rd.cDot_R_S_d(0) = _rd.cDot_R_G_d(0) + _rs.cDot_Com_S(0);
//            _rd.cDDot_R_S_d(0) = xDDot_sw_G_d + _rs.cDDot_Com_S(0);   // real acceleration is noisy
            _rd.cDDot_R_S_d(0) = xDDot_sw_G_d + _rd.cDDot_Com_S_d(0);
            if (_rsm.s >= 0.0-EPSILON && _rsm.s <= 0.5){
                plan::cubic_s(0.0, 0.5, _rs.c_R_S_0(1), _rd.H_ret, _rs.cDot_R_S_0(1), 0.0, _rsm.s, 1/_rsm.Ts, _rd.c_R_S_d(1), _rd.cDot_R_S_d(1), _rd.cDDot_R_S_d(1));
            }else{
                plan::cubic_s(0.5, 1.0, _rd.H_ret, z_compensating, 0.0, vz_S_sw_TD_d, _rsm.s, 1/_rsm.Ts, _rd.c_R_S_d(1), _rd.cDot_R_S_d(1), _rd.cDDot_R_S_d(1));
            }
            _rd.c_R_G_d(1) = _rd.c_R_S_d(1) - _rs.c_Com_S(1);
            _rd.cDot_R_G_d(1) = _rd.cDot_R_S_d(1) - _rs.cDot_Com_S(1);
            // stance foot plan
            _rd.c_L_S_d = _rs.c_L_S;            // Zero
            _rd.cDot_L_S_d = _rs.cDot_L_S;      // Zero
            _rd.cDDot_L_S_d = Eigen::Vector2d::Zero();  // Zero
            _rd.c_L_G_d = _rd.c_L_S_d - _rd.c_Com_S_d;
            _rd.cDot_L_G_d = _rd.cDot_L_S_d - _rd.cDot_Com_S_d;
        } else{
            // swing foot plan
            plan::cubic_s(0.0, 1.0, _rs.c_L_G_0(0), x_G_step_d, _rs.cDot_L_G_0(0), -_rs.vx_est, _rsm.s, 1/_rsm.Ts, _rd.c_L_G_d(0), _rd.cDot_L_G_d(0), xDDot_sw_G_d);
//            _rd.c_L_S_d(0) = _rd.c_L_G_d(0) + _rd.c_Com_S_d(0);
//            _rd.cDot_L_S_d(0) = _rd.cDot_L_G_d(0) + _rd.cDot_Com_S_d(0);
//            _rd.cDDot_L_S_d(0) = xDDot_sw_G_d + _rd.cDDot_Com_S_d(0);
            _rd.c_L_S_d(0) = _rd.c_L_G_d(0) + _rs.c_Com_S(0);
            _rd.cDot_L_S_d(0) = _rd.cDot_L_G_d(0) + _rs.cDot_Com_S(0);
//            _rd.cDDot_L_S_d(0) = xDDot_sw_G_d + _rs.cDDot_Com_S(0);   // real acceleration is noisy
            _rd.cDDot_L_S_d(0) = xDDot_sw_G_d + _rd.cDDot_Com_S_d(0);
            if (_rsm.s >= 0.0-EPSILON && _rsm.s <= 0.5){
                plan::cubic_s(0.0, 0.5, _rs.c_L_S_0(1), _rd.H_ret, _rs.cDot_L_S_0(1), 0.0, _rsm.s, 1/_rsm.Ts, _rd.c_L_S_d(1), _rd.cDot_L_S_d(1), _rd.cDDot_L_S_d(1));
            }else{
                plan::cubic_s(0.5, 1.0, _rd.H_ret, z_compensating, 0.0, vz_S_sw_TD_d, _rsm.s, 1/_rsm.Ts, _rd.c_L_S_d(1), _rd.cDot_L_S_d(1), _rd.cDDot_L_S_d(1));
            }
            _rd.c_L_G_d(1) = _rd.c_L_S_d(1) - _rs.c_Com_S(1);
            _rd.cDot_L_G_d(1) = _rd.cDot_L_S_d(1) - _rs.cDot_Com_S(1);
            // stance foot plan
            _rd.c_R_S_d = _rs.c_R_S;            // Zero
            _rd.cDot_R_S_d = _rs.cDot_R_S;      // Zero
            _rd.cDDot_R_S_d = Eigen::Vector2d::Zero();  // Zero
            _rd.c_R_G_d = _rd.c_R_S_d - _rd.c_Com_S_d;
            _rd.cDot_R_G_d = _rd.cDot_R_S_d - _rd.cDot_Com_S_d;
        }
        // ----------------------------- end of foot plan ----------------------------------

        // ----------------------------- GRF distribution ----------------------------------
        /*
         *  Motive : for 'forcePfRef'
         *      update  'fc_grf_R_S_ff', 'fc_grf_L_S_ff'
         */
        if (_rsm.stanceLeg == 1){
            _rd.fc_grf_R_S_ff = Eigen::Vector2d::Zero();
            _rd.fc_grf_L_S_ff(1) = _rs.massTotal*(GRAVITY + _rd.cDDot_Com_S_d(1));
            _rd.fc_grf_L_S_ff(0) = _rd.fc_grf_L_S_ff(1) * _rs.c_Com_S(0) / _rs.c_Com_S(1);
        }else{
            _rd.fc_grf_L_S_ff = Eigen::Vector2d::Zero();
            _rd.fc_grf_R_S_ff(1) = _rs.massTotal*(GRAVITY + _rd.cDDot_Com_S_d(1));
            _rd.fc_grf_R_S_ff(0) = _rd.fc_grf_R_S_ff(1) * _rs.c_Com_S(0) / _rs.c_Com_S(1);
        }
        // -------------------------- end of GRF distribution ------------------------------

        // PID of PV
        if (_rsm.stanceLeg == 1){
            _rd.p_pid_frrl.assign(_rd.p_pid_Lst.begin(), _rd.p_pid_Lst.end());
            _rd.i_pid_frrl.assign(_rd.i_pid_Lst.begin(), _rd.i_pid_Lst.end());
            _rd.d_pid_frrl.assign(_rd.d_pid_Lst.begin(), _rd.d_pid_Lst.end());
        }else{
            _rd.p_pid_frrl.assign(_rd.p_pid_Rst.begin(), _rd.p_pid_Rst.end());
            _rd.i_pid_frrl.assign(_rd.i_pid_Rst.begin(), _rd.i_pid_Rst.end());
            _rd.d_pid_frrl.assign(_rd.d_pid_Rst.begin(), _rd.d_pid_Rst.end());
        }

    }else{             // state == 2, DSP

        // ----------------------------- Center of Mass plan ----------------------------------
        /*
         *  Motive : for 'hgDotRef'
         *      update  'c_Com_S_d', 'cDot_Com_S_d', 'cDDot_Com_S_d'
         */

        // TODO: x follow HLIP planning output
        // xDot_Com_S_d
        _rd.c_Com_S_d(0) = _rs.c_Com_S(0);
        _rd.cDot_Com_S_d(0) = _rs.cDot_Com_S(0);
        _rd.cDDot_Com_S_d(0) = _rs.cDDot_Com_S(0);
        // TODO:
        // zDot_Com_S_d
        double z_Com_Snext_d, zDot_Com_Snext_d, zDDot_Com_Snext_d;
        plan::quintic_s(0.0, 1.0, z_Com_Snext_d_TD, z_Com_Snext_d_TD + deltaZ_Com_Snext_DSP_d, 0.0, 0.0, 0.0, 0.0, _rsm.ss, 1/_rsm.Td,
                        z_Com_Snext_d, zDot_Com_Snext_d, zDDot_Com_Snext_d);
        _rd.c_Com_S_d(1) = _rs.deltaC_foot2_minus_foot1(1) + z_Com_Snext_d;
        _rd.cDot_Com_S_d(1) = 0.0 + zDot_Com_Snext_d;
        _rd.cDDot_Com_S_d(1) = 0.0 + zDDot_Com_Snext_d;
        if (_rsm.debug_flag == 1){
            _rd.c_Com_S_d(1) = _rd.H_tgt;
            _rd.cDot_Com_S_d(1) = 0.0;
            _rd.cDDot_Com_S_d(1) = 0.0;
        }
        // -------------------------- end of Center of Mass plan ------------------------------

        // -------------------------------- foot plan ---------------------------------------
        /*
         *  Motive : for 'cPfDDotRef'
         *      update  'c_R_S_d', 'cDot_R_S_d', 'cDDot_R_S_d'
         *              'c_L_S_d', 'cDot_L_S_d', 'cDDot_L_S_d'
         */

        if (_rsm.stanceLeg == 1){
            // former swing foot
            _rd.c_R_S_d(0) = _rs.c_R_S(0);                      // deltaC_foot2_minus_foot1 ? TODO: test it
            _rd.c_R_S_d(1) = 0.;
//            _rd.cDot_R_S_d = _rs.cDot_R_S;                    // Zero ?
            _rd.cDot_R_S_d = Eigen::Vector2d::Zero();
            _rd.cDDot_R_S_d = Eigen::Vector2d::Zero();          // Zero
            _rd.c_R_G_d = _rd.c_R_S_d - _rd.c_Com_S_d;
            _rd.cDot_R_G_d = _rd.cDot_R_S_d - _rd.cDot_Com_S_d;
            // nominal stance foot
            _rd.c_L_S_d = _rs.c_L_S;                            // Zero
            _rd.cDot_L_S_d = _rs.cDot_L_S;                      // Zero
            _rd.cDDot_L_S_d = Eigen::Vector2d::Zero();          // Zero
            _rd.c_L_G_d = _rd.c_L_S_d - _rd.c_Com_S_d;
            _rd.cDot_L_G_d = _rd.cDot_L_S_d - _rd.cDot_Com_S_d;
        }else{
            // former swing foot
            _rd.c_L_S_d(0) = _rs.c_L_S(0);                      // deltaC_foot2_minus_foot1 ? TODO: test it
            _rd.c_L_S_d(1) = 0.;
//            _rd.cDot_L_S_d = _rs.cDot_L_S;                    // Zero ?
            _rd.cDot_L_S_d = Eigen::Vector2d::Zero();
            _rd.cDDot_L_S_d = Eigen::Vector2d::Zero();          // Zero
            _rd.c_L_G_d = _rd.c_L_S_d - _rd.c_Com_S_d;
            _rd.cDot_L_G_d = _rd.cDot_L_S_d - _rd.cDot_Com_S_d;
            // nominal stance foot
            _rd.c_R_S_d = _rs.c_R_S;                            // Zero
            _rd.cDot_R_S_d = _rs.cDot_R_S;                      // Zero
            _rd.cDDot_R_S_d = Eigen::Vector2d::Zero();          // Zero
            _rd.c_R_G_d = _rd.c_R_S_d - _rd.c_Com_S_d;
            _rd.cDot_R_G_d = _rd.cDot_R_S_d - _rd.cDot_Com_S_d;
        }
        // ------------------------------ end of foot plan ----------------------------------

        // ----------------------------- GRF distribution ----------------------------------
        /*
         *  Motive : for 'forcePfRef'
         *      update  'fc_grf_R_S_ff', 'fc_grf_L_S_ff'
         */
//        double fz_grf_S_ff = massTotal*(GRAVITY + cDDot_Com_S_d(1));
        double fz_grf_S_ff = _rs.massTotal*GRAVITY;
        if (_rsm.stanceLeg == 1){
            _rd.fc_grf_R_S_ff(1) = fz_grf_S_ff * _rsm.ss;            // ss = tt/Td, clamped within [0,1]
            _rd.fc_grf_R_S_ff(0) = _rd.fc_grf_R_S_ff(1) * (_rs.c_Com_S(0) - _rs.deltaC_foot2_minus_foot1(0)) / _rs.c_Com_S(1);
            _rd.fc_grf_L_S_ff(1) = fz_grf_S_ff * (1 - _rsm.ss);
            _rd.fc_grf_L_S_ff(0) = _rd.fc_grf_L_S_ff(1) * _rs.c_Com_S(0) / _rs.c_Com_S(1);
        }else{
            _rd.fc_grf_L_S_ff(1) = fz_grf_S_ff * _rsm.ss;            // ss = tt/Td, clamped within [0,1]
            _rd.fc_grf_L_S_ff(0) = _rd.fc_grf_L_S_ff(1) * (_rs.c_Com_S(0) - _rs.deltaC_foot2_minus_foot1(0)) / _rs.c_Com_S(1);
            _rd.fc_grf_R_S_ff(1) = fz_grf_S_ff * (1 - _rsm.ss);
            _rd.fc_grf_R_S_ff(0) = _rd.fc_grf_R_S_ff(1) * _rs.c_Com_S(0) / _rs.c_Com_S(1);
        }
        // -------------------------- end of GRF distribution ------------------------------

        // PID of PV
        if (_rsm.stanceLeg == 1){
            _rd.p_pid_frrl.assign(_rd.p_pid_Lst.begin(), _rd.p_pid_Lst.end());
            _rd.i_pid_frrl.assign(_rd.i_pid_Lst.begin(), _rd.i_pid_Lst.end());
            _rd.d_pid_frrl.assign(_rd.d_pid_Lst.begin(), _rd.d_pid_Lst.end());
        }else{
            _rd.p_pid_frrl.assign(_rd.p_pid_Rst.begin(), _rd.p_pid_Rst.end());
            _rd.i_pid_frrl.assign(_rd.i_pid_Rst.begin(), _rd.i_pid_Rst.end());
            _rd.d_pid_frrl.assign(_rd.d_pid_Rst.begin(), _rd.d_pid_Rst.end());
        }

    }

    // ------------------------- desired pitch ---------------------------------
    _rd.pitch_d = 0.0;
    _rd.pitchDot_d = 0.0;
    _rd.pitchDDot_d = 0.0;
    // update Ry_d
    _rd.Ry_d << std::cos(_rd.pitch_d), std::sin(_rd.pitch_d),
                -std::sin(_rd.pitch_d), std::cos(_rd.pitch_d);
    // update RyDot_d
    _rd.RyDot_d << -std::sin(_rd.pitch_d)*_rd.pitchDot_d, std::cos(_rd.pitch_d)*_rd.pitchDot_d,
                   -std::cos(_rd.pitch_d)*_rd.pitchDot_d, -std::sin(_rd.pitch_d)*_rd.pitchDot_d;
    // ---------------------- end of desired pitch ------------------------------

    return true;
}

bool motionPlan::walkTrajectoryPlan_Ub(const robotStateMachine &_rsm, const robotState &_rs, robotDesired &_rd){

    if (_rsm.state == 1){            // SSP
        // prediction of x_G_step, actually x_W_step here
        hlipStepPlan(_rsm, _rs, _rd);

        // ----------------------------- Upper body plan ----------------------------------
        /*
         *  Motive : for 'qfDDotRef'
         *      update  'c_U_S_d', 'cDot_U_S_d', 'cDDot_U_S_d'
         */

        if (_rsm.tick != 0 && _rsm.s >= 0 - EPSILON && _rsm.s <= 0 + EPSILON){
            // the initial desired-H of new SSP
            H_tgt_0 =  _rd.c_U_S_d(1);
            Hdot_tgt_0 = _rd.cDot_U_S_d(1);
        }

        // xDot_U_S_d
        if (_rsm.horizontalCtrl_flag == 1) {
//            plan::planLipForward(0., _rsm.Ts, _rs.x_init_SSP, _rs.vx_init_SSP, _rd.H_tgt, _rd.c_U_S_d(0), _rd.cDot_U_S_d(0), _rd.cDDot_U_S_d(0));
            plan::planLipForward(0., _rsm.Ts, _rs.x_init_SSP, _rs.vxE_DSP, _rd.H_tgt, _rd.c_U_S_d(0), _rd.cDot_U_S_d(0), _rd.cDDot_U_S_d(0));           // better ~
        }else {
            _rd.c_U_S_d(0) = _rs.c_U_S(0);
            _rd.cDot_U_S_d(0) = _rs.cDot_U_S(0);
            _rd.cDDot_U_S_d(0) = _rs.cDDot_U_S(0);
        }

        // zDot_U_S_d
        if (_rd.deltaH_signal == 1){
            plan::quintic_s(0.0, 0.5, H_tgt_0, _rd.H_tgt, Hdot_tgt_0, 0.0, 0.0, 0.0, _rsm.s, 1/_rsm.Ts, _rd.c_U_S_d(1), _rd.cDot_U_S_d(1), _rd.cDDot_U_S_d(1));
        }else if(_rd.deltaH_signal == 2){
            plan::quintic_s(0.0, 0.9, H_tgt_0, _rd.H_tgt, Hdot_tgt_0, 0.0, 0.0, 0.0, _rsm.s, 1/_rsm.Ts, _rd.c_U_S_d(1), _rd.cDot_U_S_d(1), _rd.cDDot_U_S_d(1));
        }else{
            plan::quintic_s(0.0, 0.5, H_tgt_0, _rd.H_tgt, Hdot_tgt_0, 0.0, 0.0, 0.0, _rsm.s, 1/_rsm.Ts, _rd.c_U_S_d(1), _rd.cDot_U_S_d(1), _rd.cDDot_U_S_d(1));
        }
        // -------------------------- end of Upper body plan ------------------------------

        // -------------------------------- foot plan ---------------------------------------
        /*
         *  Motive : for 'cPfDDotRef'
         *      update  'c_R_S_d', 'cDot_R_S_d', 'cDDot_R_S_d'
         *              'c_L_S_d', 'cDot_L_S_d', 'cDDot_L_S_d'
         */

        // trick for circular walking, correction factors for [inner, outer] legs support.
        if (_rsm.debug_flag == 1){
            rho_circular.at(0) = 0.;
            rho_circular.at(1) = 0.;
        }else{
            // for Ts=0.35s, H=0.44m, 1/sigma = 0.1437, Kp_star = 0.0843
            rho_circular.at(0) = 0.1*(1/sigma_predict);             // 0.4172
            rho_circular.at(1) = -0.075*(1/sigma_predict);            // -0.3232
        }

        double xDDot_sw_W_d;    // swing-foot desired acceleration w.r.t. "W"

        if (_rsm.stanceLeg == 1){
            // swing foot plan
            // z direction
            if (_rsm.s >= 0.0-EPSILON && _rsm.s <= 0.5){
                plan::cubic_s(0.0, 0.5, _rs.c_R_S_0(1), _rd.H_ret, _rs.cDot_R_S_0(1), 0.0, _rsm.s, 1/_rsm.Ts, _rd.c_R_S_d(1), _rd.cDot_R_S_d(1), _rd.cDDot_R_S_d(1));
            }else{
                plan::cubic_s(0.5, 1.0, _rd.H_ret, z_compensating, 0.0, vz_S_sw_TD_d, _rsm.s, 1/_rsm.Ts, _rd.c_R_S_d(1), _rd.cDot_R_S_d(1), _rd.cDDot_R_S_d(1));
            }
            _rd.c_R_W_d(1) = _rd.c_R_S_d(1) - _rs.c_U_S(1);
            _rd.cDot_R_W_d(1) = _rd.cDot_R_S_d(1) - _rs.cDot_U_S(1);
            // trick for circular walking, inner support
            x_G_step_d += rho_circular.at(0)*VxE_SSP_predict;
            // Security Guarantee
            x_sw_W_limit = std::sqrt( (0.16 + 0.32 - safety_margin)*(0.16 + 0.32 - safety_margin) - _rd.c_R_W_d(1)*_rd.c_R_W_d(1) );
            x_G_step_d = plan::clamp(x_G_step_d, -x_sw_W_limit + 0.5*0.097, x_sw_W_limit - 0.5*0.097);
            // x direction
            plan::cubic_s(0.0, 1.0, _rs.c_R_W_0(0), x_G_step_d, _rs.cDot_R_W_0(0), -_rs.vx_est, _rsm.s, 1/_rsm.Ts, _rd.c_R_W_d(0), _rd.cDot_R_W_d(0), xDDot_sw_W_d);
//            _rd.c_R_S_d(0) = _rd.c_R_W_d(0) + _rd.c_U_S_d(0);
//            _rd.cDot_R_S_d(0) = _rd.cDot_R_W_d(0) + _rd.cDot_U_S_d(0);
//            _rd.cDDot_R_S_d(0) = xDDot_sw_W_d + _rd.cDDot_U_S_d(0);
            _rd.c_R_S_d(0) = _rd.c_R_W_d(0) + _rs.c_U_S(0);
            _rd.cDot_R_S_d(0) = _rd.cDot_R_W_d(0) + _rs.cDot_U_S(0);
//            _rd.cDDot_R_S_d(0) = xDDot_sw_W_d + _rs.cDDot_U_S(0);   // real acceleration is noisy
            _rd.cDDot_R_S_d(0) = xDDot_sw_W_d + _rd.cDDot_U_S_d(0);
            // stance foot plan
            _rd.c_L_S_d = _rs.c_L_S;            // Zero
            _rd.cDot_L_S_d = _rs.cDot_L_S;      // Zero
            _rd.cDDot_L_S_d = Eigen::Vector2d::Zero();  // Zero
            _rd.c_L_W_d = _rd.c_L_S_d - _rd.c_U_S_d;
            _rd.cDot_L_W_d = _rd.cDot_L_S_d - _rd.cDot_U_S_d;
        } else{
            // swing foot plan
            // z direction
            if (_rsm.s >= 0.0-EPSILON && _rsm.s <= 0.5){
                plan::cubic_s(0.0, 0.5, _rs.c_L_S_0(1), _rd.H_ret, _rs.cDot_L_S_0(1), 0.0, _rsm.s, 1/_rsm.Ts, _rd.c_L_S_d(1), _rd.cDot_L_S_d(1), _rd.cDDot_L_S_d(1));
            }else{
                plan::cubic_s(0.5, 1.0, _rd.H_ret, z_compensating, 0.0, vz_S_sw_TD_d, _rsm.s, 1/_rsm.Ts, _rd.c_L_S_d(1), _rd.cDot_L_S_d(1), _rd.cDDot_L_S_d(1));
            }
            _rd.c_L_W_d(1) = _rd.c_L_S_d(1) - _rs.c_U_S(1);
            _rd.cDot_L_W_d(1) = _rd.cDot_L_S_d(1) - _rs.cDot_U_S(1);
            // trick for circular walking, outer support
            x_G_step_d += rho_circular.at(1)*VxE_SSP_predict;
            // Security Guarantee
            x_sw_W_limit = std::sqrt( (0.16 + 0.32 - safety_margin)*(0.16 + 0.32 - safety_margin) - _rd.c_R_W_d(1)*_rd.c_R_W_d(1) );
            x_G_step_d = plan::clamp(x_G_step_d, -x_sw_W_limit + 0.5*0.097, x_sw_W_limit - 0.5*0.097);
            // x direction
            plan::cubic_s(0.0, 1.0, _rs.c_L_W_0(0), x_G_step_d, _rs.cDot_L_W_0(0), -_rs.vx_est, _rsm.s, 1/_rsm.Ts, _rd.c_L_W_d(0), _rd.cDot_L_W_d(0), xDDot_sw_W_d);
//            _rd.c_L_S_d(0) = _rd.c_L_W_d(0) + _rd.c_U_S_d(0);
//            _rd.cDot_L_S_d(0) = _rd.cDot_L_W_d(0) + _rd.cDot_U_S_d(0);
//            _rd.cDDot_L_S_d(0) = xDDot_sw_W_d + _rd.cDDot_U_S_d(0);
            _rd.c_L_S_d(0) = _rd.c_L_W_d(0) + _rs.c_U_S(0);
            _rd.cDot_L_S_d(0) = _rd.cDot_L_W_d(0) + _rs.cDot_U_S(0);
//            _rd.cDDot_L_S_d(0) = xDDot_sw_W_d + _rs.cDDot_U_S(0);   // real acceleration is noisy
            _rd.cDDot_L_S_d(0) = xDDot_sw_W_d + _rd.cDDot_U_S_d(0);
            // stance foot plan
            _rd.c_R_S_d = _rs.c_R_S;            // Zero
            _rd.cDot_R_S_d = _rs.cDot_R_S;      // Zero
            _rd.cDDot_R_S_d = Eigen::Vector2d::Zero();  // Zero
            _rd.c_R_W_d = _rd.c_R_S_d - _rd.c_U_S_d;
            _rd.cDot_R_W_d = _rd.cDot_R_S_d - _rd.cDot_U_S_d;
        }
        // ----------------------------- end of foot plan ----------------------------------

        // ----------------------------- GRF distribution ----------------------------------
        /*
         *  Motive : for 'forcePfRef'
         *      update  'fc_grf_R_S_ff', 'fc_grf_L_S_ff'
         */
        if (_rsm.stanceLeg == 1){
            _rd.fc_grf_R_S_ff = Eigen::Vector2d::Zero();
            _rd.fc_grf_L_S_ff(1) = _rs.massTotal*(GRAVITY + _rd.cDDot_U_S_d(1));
            _rd.fc_grf_L_S_ff(0) = _rd.fc_grf_L_S_ff(1) * _rs.c_U_S(0) / _rs.c_U_S(1);
        }else{
            _rd.fc_grf_L_S_ff = Eigen::Vector2d::Zero();
            _rd.fc_grf_R_S_ff(1) = _rs.massTotal*(GRAVITY + _rd.cDDot_U_S_d(1));
            _rd.fc_grf_R_S_ff(0) = _rd.fc_grf_R_S_ff(1) * _rs.c_U_S(0) / _rs.c_U_S(1);
        }
        // -------------------------- end of GRF distribution ------------------------------

        // PID of PV
        if (_rsm.stanceLeg == 1){
            _rd.p_pid_frrl.assign(_rd.p_pid_Lst.begin(), _rd.p_pid_Lst.end());
            _rd.i_pid_frrl.assign(_rd.i_pid_Lst.begin(), _rd.i_pid_Lst.end());
            _rd.d_pid_frrl.assign(_rd.d_pid_Lst.begin(), _rd.d_pid_Lst.end());
        }else{
            _rd.p_pid_frrl.assign(_rd.p_pid_Rst.begin(), _rd.p_pid_Rst.end());
            _rd.i_pid_frrl.assign(_rd.i_pid_Rst.begin(), _rd.i_pid_Rst.end());
            _rd.d_pid_frrl.assign(_rd.d_pid_Rst.begin(), _rd.d_pid_Rst.end());
        }

    }else{             // state == 2, DSP

        // ----------------------------- Center of Mass plan ----------------------------------
        /*
         *  Motive : for 'hgDotRef'
         *      update  'c_U_S_d', 'cDot_U_S_d', 'cDDot_U_S_d'
         */

        // TODO: x follow HLIP planning output
        // xDot_U_S_d
        _rd.c_U_S_d(0) = _rs.c_U_S(0);
        _rd.cDot_U_S_d(0) = _rs.cDot_U_S(0);            // TODO: test v+ of DSP
        _rd.cDDot_U_S_d(0) = _rs.cDDot_U_S(0);

        // zDot_U_S_d
        _rd.c_U_S_d(1) = _rd.H_tgt;
        _rd.cDot_U_S_d(1) = 0.0;
        _rd.cDDot_U_S_d(1) = 0.0;

        // ---------------------------------- transition from TD (the start of DSP), bad ! ----------------------- //
//        if (_rsm.remark_flag == 1){
//            if (_rsm.stanceLeg == 1){
//                z_sw_W_d_TD = _rd.c_L_W_d(1);
//                zDot_sw_W_d_TD = _rd.cDot_L_W_d(1);
//            }else{
//                z_sw_W_d_TD = _rd.c_R_W_d(1);
//                zDot_sw_W_d_TD = _rd.cDot_R_W_d(1);
//            }
//        }
//        plan::quintic_s(0.0, _rsm.Td + 0.5*_rsm.Ts, -z_sw_W_d_TD, _rd.H_tgt, 0.0, 0.0, 0.0, 0.0, _rsm.tt, 1,
//                        _rd.c_U_S_d(1), _rd.cDot_U_S_d(1), _rd.cDDot_U_S_d(1));
//        plan::quintic_s(0.0, _rsm.Td, -z_sw_W_d_TD, _rd.H_tgt, 0.0, 0.0, 0.0, 0.0, _rsm.tt, 1,
//                        _rd.c_U_S_d(1), _rd.cDot_U_S_d(1), _rd.cDDot_U_S_d(1));
        // ---------------------------------- transition from TD (the start of DSP), bad ! ----------------------- //

        // -------------------------- end of Center of Mass plan ------------------------------

        // -------------------------------- foot plan ---------------------------------------
        /*
         *  Motive : for 'cPfDDotRef'
         *      update  'c_R_S_d', 'cDot_R_S_d', 'cDDot_R_S_d'
         *              'c_L_S_d', 'cDot_L_S_d', 'cDDot_L_S_d'
         */

        if (_rsm.stanceLeg == 1){
            // former stance foot
            _rd.c_R_S_d(0) = _rs.c_R_S(0);
//            _rd.c_R_S_d(1) = _rs.c_R_S(1);                      // Zero ?
            _rd.c_R_S_d(1) = 0.;
//            _rd.cDot_R_S_d = _rs.cDot_R_S;                      // Zero ?
            _rd.cDot_R_S_d = Eigen::Vector2d::Zero();
            _rd.cDDot_R_S_d = Eigen::Vector2d::Zero();          // Zero
            // here, to be tested!
            _rd.c_R_W_d = _rd.c_R_S_d - _rd.c_U_S_d;
            _rd.cDot_R_W_d = _rd.cDot_R_S_d - _rd.cDot_U_S_d;
            // new nominal stance foot
            _rd.c_L_S_d = _rs.c_L_S;                            // Zero
            _rd.cDot_L_S_d = _rs.cDot_L_S;                      // Zero
            _rd.cDDot_L_S_d = Eigen::Vector2d::Zero();          // Zero
            _rd.c_L_W_d = _rd.c_L_S_d - _rd.c_U_S_d;
            _rd.cDot_L_W_d = _rd.cDot_L_S_d - _rd.cDot_U_S_d;
        }else{
            // former stance foot
            _rd.c_L_S_d(0) = _rs.c_L_S(0);
//            _rd.c_L_S_d(1) = _rs.c_L_S(1);                      // Zero ?
            _rd.c_L_S_d(1) = 0.;
//            _rd.cDot_L_S_d = _rs.cDot_L_S;                      // Zero ?
            _rd.cDot_L_S_d = Eigen::Vector2d::Zero();
            _rd.cDDot_L_S_d = Eigen::Vector2d::Zero();          // Zero
            _rd.c_L_W_d = _rd.c_L_S_d - _rd.c_U_S_d;
            _rd.cDot_L_W_d = _rd.cDot_L_S_d - _rd.cDot_U_S_d;
            // new nominal stance foot
            _rd.c_R_S_d = _rs.c_R_S;                            // Zero
            _rd.cDot_R_S_d = _rs.cDot_R_S;                      // Zero
            _rd.cDDot_R_S_d = Eigen::Vector2d::Zero();          // Zero
            _rd.c_R_W_d = _rd.c_R_S_d - _rd.c_U_S_d;
            _rd.cDot_R_W_d = _rd.cDot_R_S_d - _rd.cDot_U_S_d;
        }
        // ------------------------------ end of foot plan ----------------------------------

        // ----------------------------- GRF distribution ----------------------------------
        /*
         *  Motive : for 'forcePfRef'
         *      update  'fc_grf_R_S_ff', 'fc_grf_L_S_ff'
         */
//        double fz_grf_S_ff = massTotal*(GRAVITY + cDDot_U_S_d(1));
        double fz_grf_S_ff = _rs.massTotal*GRAVITY;
        if (_rsm.stanceLeg == 1){
            _rd.fc_grf_L_S_ff(1) = fz_grf_S_ff * _rsm.ss;            // ss = tt/Td, clamped within [0,1]
            _rd.fc_grf_L_S_ff(0) = _rd.fc_grf_L_S_ff(1) * _rs.c_U_S(0) / _rs.c_U_S(1);
            _rd.fc_grf_R_S_ff(1) = fz_grf_S_ff * (1 - _rsm.ss);
            _rd.fc_grf_R_S_ff(0) = _rd.fc_grf_R_S_ff(1) * (_rs.c_U_S(0) - (_rs.c_R_S(0) - _rs.c_L_S(0))) / _rs.c_U_S(1);
        }else{
            _rd.fc_grf_R_S_ff(1) = fz_grf_S_ff * _rsm.ss;            // ss = tt/Td, clamped within [0,1]
            _rd.fc_grf_R_S_ff(0) = _rd.fc_grf_R_S_ff(1) * _rs.c_U_S(0) / _rs.c_U_S(1);
            _rd.fc_grf_L_S_ff(1) = fz_grf_S_ff * (1 - _rsm.ss);
            _rd.fc_grf_L_S_ff(0) = _rd.fc_grf_L_S_ff(1) * (_rs.c_U_S(0) - (_rs.c_L_S(0) - _rs.c_R_S(0))) / _rs.c_U_S(1);
        }
        // -------------------------- end of GRF distribution ------------------------------


        // PID of PV
//        if (_rsm.stanceLeg == 1){
//            _rd.p_pid_frrl.assign(_rd.p_pid_Lst.begin(), _rd.p_pid_Lst.end());
//            _rd.i_pid_frrl.assign(_rd.i_pid_Lst.begin(), _rd.i_pid_Lst.end());
//            _rd.d_pid_frrl.assign(_rd.d_pid_Lst.begin(), _rd.d_pid_Lst.end());
//        }else{
//            _rd.p_pid_frrl.assign(_rd.p_pid_Rst.begin(), _rd.p_pid_Rst.end());
//            _rd.i_pid_frrl.assign(_rd.i_pid_Rst.begin(), _rd.i_pid_Rst.end());
//            _rd.d_pid_frrl.assign(_rd.d_pid_Rst.begin(), _rd.d_pid_Rst.end());
//        }
        double tmp;
        if (_rsm.stanceLeg == 1){
            for (int i = 0; i != 4; i++){
                plan::cubic_s(0.0, 1.0, _rd.p_pid_Rst.at(i), _rd.p_pid_Lst.at(i), 0.0, 0.0, _rsm.ss, 1/_rsm.Td, _rd.p_pid_frrl.at(i), tmp);
                plan::cubic_s(0.0, 1.0, _rd.i_pid_Rst.at(i), _rd.i_pid_Lst.at(i), 0.0, 0.0, _rsm.ss, 1/_rsm.Td, _rd.i_pid_frrl.at(i), tmp);
                plan::cubic_s(0.0, 1.0, _rd.d_pid_Rst.at(i), _rd.d_pid_Lst.at(i), 0.0, 0.0, _rsm.ss, 1/_rsm.Td, _rd.d_pid_frrl.at(i), tmp);
            }
        }else{
            for (int i = 0; i != 4; i++){
                plan::cubic_s(0.0, 1.0, _rd.p_pid_Lst.at(i), _rd.p_pid_Rst.at(i), 0.0, 0.0, _rsm.ss, 1/_rsm.Td, _rd.p_pid_frrl.at(i), tmp);
                plan::cubic_s(0.0, 1.0, _rd.i_pid_Lst.at(i), _rd.i_pid_Rst.at(i), 0.0, 0.0, _rsm.ss, 1/_rsm.Td, _rd.i_pid_frrl.at(i), tmp);
                plan::cubic_s(0.0, 1.0, _rd.d_pid_Lst.at(i), _rd.d_pid_Rst.at(i), 0.0, 0.0, _rsm.ss, 1/_rsm.Td, _rd.d_pid_frrl.at(i), tmp);
            }
        }

    }

    // ------------------------- desired pitch ---------------------------------
    _rd.pitch_d = 0.0;
    _rd.pitchDot_d = 0.0;
    _rd.pitchDDot_d = 0.0;
    // update Ry_d
    _rd.Ry_d << std::cos(_rd.pitch_d), std::sin(_rd.pitch_d),
                -std::sin(_rd.pitch_d), std::cos(_rd.pitch_d);
    // update RyDot_d
    _rd.RyDot_d << -std::sin(_rd.pitch_d)*_rd.pitchDot_d, std::cos(_rd.pitch_d)*_rd.pitchDot_d,
                   -std::cos(_rd.pitch_d)*_rd.pitchDot_d, -std::sin(_rd.pitch_d)*_rd.pitchDot_d;
    // ---------------------- end of desired pitch ------------------------------

    // ------------------------- in frame 'B' -----------------------------------
    if (_rsm.stanceLeg == 1){
        _rd.c_R_B_d = (_rs.Ry.transpose()) * _rd.c_R_W_d;
        _rd.cDot_R_B_d = (_rs.Ry.transpose()) * (_rd.cDot_R_W_d - _rs.RyDot * _rd.c_R_B_d);
        _rd.c_L_B_d = (_rd.Ry_d.transpose()) * _rd.c_L_W_d;
        _rd.cDot_L_B_d = (_rd.Ry_d.transpose()) * (_rd.cDot_L_W_d - _rd.RyDot_d * _rd.c_L_B_d);
    }else{
        _rd.c_R_B_d = (_rd.Ry_d.transpose()) * _rd.c_R_W_d;
        _rd.cDot_R_B_d = (_rd.Ry_d.transpose()) * (_rd.cDot_R_W_d - _rd.RyDot_d* _rd.c_R_B_d);
        _rd.c_L_B_d = (_rs.Ry.transpose()) * _rd.c_L_W_d;
        _rd.cDot_L_B_d = (_rs.Ry.transpose()) * (_rd.cDot_L_W_d - _rs.RyDot * _rd.c_L_B_d);
    }
    // Security Guarantee
    x_R_B_d_limit = std::sqrt( (0.16 + 0.32 - safety_margin)*(0.16 + 0.32 - safety_margin) - _rd.c_R_B_d(1)*_rd.c_R_B_d(1) );
    _rd.c_R_B_d(0) = plan::clamp(_rd.c_R_B_d(0), -x_R_B_d_limit + 0.5*0.097, x_R_B_d_limit - 0.5*0.097);
    x_L_B_d_limit = std::sqrt( (0.16 + 0.32 - safety_margin)*(0.16 + 0.32 - safety_margin) - _rd.c_L_B_d(1)*_rd.c_L_B_d(1) );
    _rd.c_L_B_d(0) = plan::clamp(_rd.c_L_B_d(0), -x_L_B_d_limit + 0.5*0.097, x_L_B_d_limit - 0.5*0.097);
    // ------------------------- in frame 'B' -----------------------------------

    return true;
}

bool motionPlan::holdInAirPlan(const robotStateMachine &_rsm, const robotState &_rs, robotDesired &_rd){

    // ------------------------- desired pitch ---------------------------------
    _rd.pitch_d = 0.0;
    _rd.pitchDot_d = 0.0;
    _rd.pitchDDot_d = 0.0;
    // update Ry_d
    _rd.Ry_d << std::cos(_rd.pitch_d), std::sin(_rd.pitch_d),
                -std::sin(_rd.pitch_d), std::cos(_rd.pitch_d);
    // update RyDot_d
    _rd.RyDot_d << -std::sin(_rd.pitch_d)*_rd.pitchDot_d, std::cos(_rd.pitch_d)*_rd.pitchDot_d,
                   -std::cos(_rd.pitch_d)*_rd.pitchDot_d, -std::sin(_rd.pitch_d)*_rd.pitchDot_d;
    // ---------------------- end of desired pitch ------------------------------

    // Update desired trajectory of Stance Leg --> Left Leg
    _rd.c_R_W_d = _rs.c_R_W_0;
    _rd.c_L_W_d = _rs.c_L_W_0;

    _rd.cDot_R_W_d << 0. , 0.;
    _rd.cDot_L_W_d << 0. , 0.;

    if (_rsm.behavior_flag == 0){
        _rd.c_R_B_d = (_rs.Ry.transpose()) * _rd.c_R_W_d;
        _rd.cDot_R_B_d = (_rs.Ry.transpose()) * (_rd.cDot_R_W_d - _rs.RyDot * _rd.c_R_B_d);
        _rd.c_L_B_d = (_rs.Ry.transpose()) * _rd.c_L_W_d;
        _rd.cDot_L_B_d = (_rs.Ry.transpose()) * (_rd.cDot_L_W_d - _rs.RyDot * _rd.c_L_B_d);
    }

    // ------------------------------ control -----------------------------------------
    // PID of PV
    if (_rsm.stanceLeg == 1){
        _rd.p_pid_frrl.assign(_rd.p_pid_Lst.begin(), _rd.p_pid_Lst.end());
        _rd.i_pid_frrl.assign(_rd.i_pid_Lst.begin(), _rd.i_pid_Lst.end());
        _rd.d_pid_frrl.assign(_rd.d_pid_Lst.begin(), _rd.d_pid_Lst.end());
    }else{
        _rd.p_pid_frrl.assign(_rd.p_pid_Rst.begin(), _rd.p_pid_Rst.end());
        _rd.i_pid_frrl.assign(_rd.i_pid_Rst.begin(), _rd.i_pid_Rst.end());
        _rd.d_pid_frrl.assign(_rd.d_pid_Rst.begin(), _rd.d_pid_Rst.end());
    }
    return true;
}

bool motionPlan::standUpPlan(const robotStateMachine &_rsm, const robotState &_rs, robotDesired &_rd){

    // ------------------------------ planing ------------------------------------------
    // Interpolation form starting-state to desired-state (standing)
    if (_rsm.standUp_flag == 0){
        // ------------------------------ right leg --------------------------------
        plan::cubic_s(0.0, _rsm.t_interpolating, _rs.c_R_W_0(0), 0.0, 0.0, 0.0, _rsm.t, 1, x_R_W_d, xDot_R_W_d);
        _rd.c_R_W_d(0) = x_R_W_d;
        _rd.cDot_R_W_d(0) = xDot_R_W_d;
        plan::cubic_s(0.0, _rsm.t_interpolating, _rs.c_R_W_0(1), -(_rsm.Hn_tryStand), 0, 0, _rsm.t, 1, z_R_W_d, zDot_R_W_d);
        _rd.c_R_W_d(1) = z_R_W_d;
        _rd.cDot_R_W_d(1) = zDot_R_W_d;
        // ------------------------------ left leg --------------------------------
        plan::cubic_s(0.0, _rsm.t_interpolating, _rs.c_L_W_0(0), 0.0, 0.0, 0.0, _rsm.t, 1, x_L_W_d, xDot_L_W_d);
        _rd.c_L_W_d(0) = x_L_W_d;
        _rd.cDot_L_W_d(0) = xDot_L_W_d;
        plan::cubic_s(0.0, _rsm.t_interpolating, _rs.c_L_W_0(1), -(_rsm.Hn_tryStand), 0, 0, _rsm.t, 1, z_L_W_d, zDot_L_W_d);
        _rd.c_L_W_d(1) = z_L_W_d;
        _rd.cDot_L_W_d(1) = zDot_L_W_d;
    }else if(_rsm.standUp_flag == 1){
        if (_rsm.firstStance_flag == 1){
            // ------------------------------ left leg (stance) --------------------------------
            plan::cubic_s(0.0, _rsm.t_interpolating, _rs.c_L_W_0(0), 0.0, 0.0, 0.0, _rsm.t, 1, x_L_W_d, xDot_L_W_d);
            _rd.c_L_W_d(0) = x_L_W_d;
            _rd.cDot_L_W_d(0) = xDot_L_W_d;
            plan::cubic_s(0.0, _rsm.t_interpolating, _rs.c_L_W_0(1), -(_rsm.Hn_tryStand), 0, 0, _rsm.t, 1, z_L_W_d, zDot_L_W_d);
            _rd.c_L_W_d(1) = z_L_W_d;
            _rd.cDot_L_W_d(1) = zDot_L_W_d;
            // ------------------------------ right leg (swing) --------------------------------
            plan::cubic_s(0.0, _rsm.t_interpolating, _rs.c_R_W_0(0), 0.0, 0.0, 0.0, _rsm.t, 1, x_R_W_d, xDot_R_W_d);
            _rd.c_R_W_d(0) = x_R_W_d;
            _rd.cDot_R_W_d(0) = xDot_R_W_d;
            plan::cubic_s(0.0, _rsm.t_interpolating, _rs.c_R_S_0(1), _rd.H_ret, 0, 0, _rsm.t, 1, z_sw_S_d, dz_sw_S_d);
            _rd.c_R_W_d(1) = z_sw_S_d - _rs.c_U_S(1);
            _rd.cDot_R_W_d(1) = dz_sw_S_d - _rs.cDot_U_S(1);
        }else{
            // ------------------------------ right leg (stance) --------------------------------
            plan::cubic_s(0.0, _rsm.t_interpolating, _rs.c_R_W_0(0), 0.0, 0.0, 0.0, _rsm.t, 1, x_R_W_d, xDot_R_W_d);
            _rd.c_R_W_d(0) = x_R_W_d;
            _rd.cDot_R_W_d(0) = xDot_R_W_d;
            plan::cubic_s(0.0, _rsm.t_interpolating, _rs.c_R_W_0(1), -(_rsm.Hn_tryStand), 0, 0, _rsm.t, 1, z_R_W_d, zDot_R_W_d);
            _rd.c_R_W_d(1) = z_R_W_d;
            _rd.cDot_R_W_d(1) = zDot_R_W_d;
            // ------------------------------ left leg (swing) --------------------------------
            plan::cubic_s(0.0, _rsm.t_interpolating, _rs.c_L_W_0(0), 0.0, 0.0, 0.0, _rsm.t, 1, x_L_W_d, xDot_L_W_d);
            _rd.c_L_W_d(0) = x_L_W_d;
            _rd.cDot_L_W_d(0) = xDot_L_W_d;
            plan::cubic_s(0.0, _rsm.t_interpolating, _rs.c_L_S_0(1), _rd.H_ret, 0, 0, _rsm.t, 1, z_sw_S_d, dz_sw_S_d);
            _rd.c_L_W_d(1) = z_sw_S_d - _rs.c_U_S(1);
            _rd.cDot_L_W_d(1) = dz_sw_S_d - _rs.cDot_U_S(1);
        }
    }else{  // _rsm.standUp_flag == 2, for 'stand-balance-control'
        // ------------------------------ right leg --------------------------------
        plan::cubic_s(0.0, 0.5*_rsm.t_interpolating, _rs.c_R_W_0(0), _rd.X_offset, 0.0, 0.0, _rsm.t, 1, x_R_W_d, xDot_R_W_d);
        _rd.c_R_W_d(0) = x_R_W_d;
        _rd.cDot_R_W_d(0) = xDot_R_W_d;
        plan::cubic_s(0.0, _rsm.t_interpolating, _rs.c_R_W_0(1), -(_rsm.Hn_tryStand), 0, 0, _rsm.t, 1, z_R_W_d, zDot_R_W_d);
        _rd.c_R_W_d(1) = z_R_W_d;
        _rd.cDot_R_W_d(1) = zDot_R_W_d;
        // ------------------------------ left leg --------------------------------
        plan::cubic_s(0.0, 0.5*_rsm.t_interpolating, _rs.c_L_W_0(0), -_rd.X_offset, 0.0, 0.0, _rsm.t, 1, x_L_W_d, xDot_L_W_d);
        _rd.c_L_W_d(0) = x_L_W_d;
        _rd.cDot_L_W_d(0) = xDot_L_W_d;
        plan::cubic_s(0.0, _rsm.t_interpolating, _rs.c_L_W_0(1), -(_rsm.Hn_tryStand), 0, 0, _rsm.t, 1, z_L_W_d, zDot_L_W_d);
        _rd.c_L_W_d(1) = z_L_W_d;
        _rd.cDot_L_W_d(1) = zDot_L_W_d;
    }

    // ------------------------- desired pitch ---------------------------------
    _rd.pitch_d = 0.0;
    _rd.pitchDot_d = 0.0;
    _rd.pitchDDot_d = 0.0;
    // update Ry_d
    _rd.Ry_d << std::cos(_rd.pitch_d), std::sin(_rd.pitch_d),
                -std::sin(_rd.pitch_d), std::cos(_rd.pitch_d);
    // update RyDot_d
    _rd.RyDot_d << -std::sin(_rd.pitch_d)*_rd.pitchDot_d, std::cos(_rd.pitch_d)*_rd.pitchDot_d,
                   -std::cos(_rd.pitch_d)*_rd.pitchDot_d, -std::sin(_rd.pitch_d)*_rd.pitchDot_d;
    // ---------------------- end of desired pitch ------------------------------

    if (_rsm.behavior_flag == 1){
        _rd.c_R_B_d = (_rd.Ry_d.transpose()) * _rd.c_R_W_d;
        _rd.cDot_R_B_d = (_rd.Ry_d.transpose()) * (_rd.cDot_R_W_d - _rd.RyDot_d * _rd.c_R_B_d);
        _rd.c_L_B_d = (_rd.Ry_d.transpose()) * _rd.c_L_W_d;
        _rd.cDot_L_B_d = (_rd.Ry_d.transpose()) * (_rd.cDot_L_W_d - _rd.RyDot_d * _rd.c_L_B_d);
    }

    // ------------------------------ control -----------------------------------------
    // PID of PV
    if (_rsm.stanceLeg == 1){
        _rd.p_pid_frrl.assign(_rd.p_pid_Lst.begin(), _rd.p_pid_Lst.end());
        _rd.i_pid_frrl.assign(_rd.i_pid_Lst.begin(), _rd.i_pid_Lst.end());
        _rd.d_pid_frrl.assign(_rd.d_pid_Lst.begin(), _rd.d_pid_Lst.end());
    }else{
        _rd.p_pid_frrl.assign(_rd.p_pid_Rst.begin(), _rd.p_pid_Rst.end());
        _rd.i_pid_frrl.assign(_rd.i_pid_Rst.begin(), _rd.i_pid_Rst.end());
        _rd.d_pid_frrl.assign(_rd.d_pid_Rst.begin(), _rd.d_pid_Rst.end());
    }
    return true;
}

bool motionPlan::standBalancePlan(const robotStateMachine &_rsm, const robotState &_rs, robotDesired &_rd){

    // ----------------------------- Center of Mass plan ----------------------------------
    /*
     *  Motive : for 'hgDotRef'
     *      update  'c_Com_S_d', 'cDot_Com_S_d', 'cDDot_Com_S_d'
     */

    // TODO: x follow HLIP planning output
    // xDot_Com_S_d
    _rd.c_Com_S_d(0) = _rd.X_offset;
    _rd.cDot_Com_S_d(0) = 0.;
    _rd.cDDot_Com_S_d(0) = 0.;
    // zDot_Com_S_d
    if(_rsm.tick == 0){
        _rd.H_tgt_pre = H_tgt_0;
    }
    plan::quintic_s(0.0, 1.0, _rd.H_tgt_pre, _rd.H_tgt, 0.0, 0.0, 0.0, 0.0, _rsm.tt, 1,
                    _rd.c_Com_S_d(1), _rd.cDot_Com_S_d(1), _rd.cDDot_Com_S_d(1));
    // -------------------------- end of Center of Mass plan ------------------------------

    // -------------------------------- foot plan ---------------------------------------
    /*
     *  Motive : for 'cPfDDotRef'
     *      update  'c_R_S_d', 'cDot_R_S_d', 'cDDot_R_S_d'
     *              'c_L_S_d', 'cDot_L_S_d', 'cDDot_L_S_d'
     */

    if (_rsm.stanceLeg == 1){
        // former swing foot
        _rd.c_R_S_d << 2.0*_rd.X_offset, 0.0;
        _rd.cDot_R_S_d = Eigen::Vector2d::Zero();
        _rd.cDDot_R_S_d = Eigen::Vector2d::Zero();
        _rd.c_R_G_d = _rd.c_R_S_d - _rd.c_Com_S_d;
        _rd.cDot_R_G_d = _rd.cDot_R_S_d - _rd.cDot_Com_S_d;
        // nominal stance foot
        _rd.c_L_S_d = Eigen::Vector2d::Zero();
        _rd.cDot_L_S_d = Eigen::Vector2d::Zero();
        _rd.cDDot_L_S_d = Eigen::Vector2d::Zero();
        _rd.c_L_G_d = _rd.c_L_S_d - _rd.c_Com_S_d;
        _rd.cDot_L_G_d = _rd.cDot_L_S_d - _rd.cDot_Com_S_d;
    }else{
        // former swing foot
        _rd.c_L_S_d << 2.0*_rd.X_offset, 0.0;
        _rd.cDot_L_S_d = Eigen::Vector2d::Zero();
        _rd.cDDot_L_S_d = Eigen::Vector2d::Zero();
        _rd.c_L_G_d = _rd.c_L_S_d - _rd.c_Com_S_d;
        _rd.cDot_L_G_d = _rd.cDot_L_S_d - _rd.cDot_Com_S_d;
        // nominal stance foot
        _rd.c_R_S_d = Eigen::Vector2d::Zero();
        _rd.cDot_R_S_d = Eigen::Vector2d::Zero();
        _rd.cDDot_R_S_d = Eigen::Vector2d::Zero();
        _rd.c_R_G_d = _rd.c_R_S_d - _rd.c_Com_S_d;
        _rd.cDot_R_G_d = _rd.cDot_R_S_d - _rd.cDot_Com_S_d;
    }
    // ------------------------------ end of foot plan ----------------------------------

    // ----------------------------- GRF distribution ----------------------------------
    /*
     *  Motive : for 'forcePfRef'
     *      update  'fc_grf_R_S_ff', 'fc_grf_L_S_ff'
     */
//        double fz_grf_S_ff = massTotal*(GRAVITY + cDDot_Com_S_d(1));
    double fz_grf_S_ff = _rs.massTotal*GRAVITY;
    if (_rsm.stanceLeg == 1){
        _rd.fc_grf_R_S_ff(1) = fz_grf_S_ff * 0.5;
        _rd.fc_grf_R_S_ff(0) = _rd.fc_grf_R_S_ff(1) * (_rs.c_Com_S(0) - 2.0*_rd.X_offset) / _rs.c_Com_S(1);
        _rd.fc_grf_L_S_ff(1) = fz_grf_S_ff * 0.5;
        _rd.fc_grf_L_S_ff(0) = _rd.fc_grf_L_S_ff(1) * _rs.c_Com_S(0) / _rs.c_Com_S(1);
    }else{
        _rd.fc_grf_L_S_ff(1) = fz_grf_S_ff * 0.5;
        _rd.fc_grf_L_S_ff(0) = _rd.fc_grf_L_S_ff(1) * (_rs.c_Com_S(0) - 2.0*_rd.X_offset) / _rs.c_Com_S(1);
        _rd.fc_grf_R_S_ff(1) = fz_grf_S_ff * 0.5;
        _rd.fc_grf_R_S_ff(0) = _rd.fc_grf_R_S_ff(1) * _rs.c_Com_S(0) / _rs.c_Com_S(1);
    }
    // -------------------------- end of GRF distribution ------------------------------

    // ------------------------- desired pitch ---------------------------------
    _rd.pitch_d = 0.0;
    _rd.pitchDot_d = 0.0;
    _rd.pitchDDot_d = 0.0;
    // update Ry_d
    _rd.Ry_d << std::cos(_rd.pitch_d), std::sin(_rd.pitch_d),
                -std::sin(_rd.pitch_d), std::cos(_rd.pitch_d);
    // update RyDot_d
    _rd.RyDot_d << -std::sin(_rd.pitch_d)*_rd.pitchDot_d, std::cos(_rd.pitch_d)*_rd.pitchDot_d,
                   -std::cos(_rd.pitch_d)*_rd.pitchDot_d, -std::sin(_rd.pitch_d)*_rd.pitchDot_d;
    // ---------------------- end of desired pitch ------------------------------

    // PID of PV
    if (_rsm.stanceLeg == 1){
        _rd.p_pid_frrl.assign(_rd.p_pid_Lst.begin(), _rd.p_pid_Lst.end());
        _rd.i_pid_frrl.assign(_rd.i_pid_Lst.begin(), _rd.i_pid_Lst.end());
        _rd.d_pid_frrl.assign(_rd.d_pid_Lst.begin(), _rd.d_pid_Lst.end());
    }else{
        _rd.p_pid_frrl.assign(_rd.p_pid_Rst.begin(), _rd.p_pid_Rst.end());
        _rd.i_pid_frrl.assign(_rd.i_pid_Rst.begin(), _rd.i_pid_Rst.end());
        _rd.d_pid_frrl.assign(_rd.d_pid_Rst.begin(), _rd.d_pid_Rst.end());
    }

    return true;
}

bool motionPlan::standBalancePlan_Ub(const robotStateMachine &_rsm, const robotState &_rs, robotDesired &_rd){

    if(ctrl_flag == 2){
        _rd.X_offset = 0.0;
    }else{
        _rd.X_offset = 0.1;
    }

    // ----------------------------- Upper body plan ----------------------------------
    /*
     *  Motive : for 'qfDDotRef'
     *      update  'c_U_S_d', 'cDot_U_S_d', 'cDDot_U_S_d'
     */

    if (_rsm.t >= t_start){

        if (ctrl_flag == 1){
            _rd.pitch_d = pitch_bias + pitch_amplitude*std::cos(2*PI/T_cycle*(_rsm.t - t_start) + phase_init);
            _rd.pitchDot_d = -2*PI/T_cycle*pitch_amplitude*std::sin(2*PI/T_cycle*(_rsm.t - t_start) + phase_init);
            _rd.pitchDDot_d = -4*PI*PI/(T_cycle*T_cycle)*pitch_amplitude*std::cos(2*PI/T_cycle*(_rsm.t - t_start) + phase_init);

            _rd.c_U_S_d(0) = _rd.X_offset;
            _rd.cDot_U_S_d(0) = 0.;
            _rd.cDDot_U_S_d(0) = 0.;

//            _rd.c_U_S_d(1) = z_bias + z_amplitude*std::sin(2*PI/T_cycle*(_rsm.t - t_start) + phase_init);
//            _rd.cDot_U_S_d(1) = 2*PI/T_cycle*z_amplitude*std::cos(2*PI/T_cycle*(_rsm.t - t_start) + phase_init);
//            _rd.cDDot_U_S_d(1) = -4*PI*PI/(T_cycle*T_cycle)*z_amplitude*std::sin(2*PI/T_cycle*(_rsm.t - t_start) + phase_init);

            if(t_ >= T_cycle - EPSILON && t_ <= T_cycle + EPSILON){
                t_last_ = _rsm.t - t_start;
            }
            t_ = _rsm.t - t_start - t_last_;
            if(t_ <= 0.5*T_cycle){
                plan::quintic_s(0.0, 0.5*T_cycle, z_bias + z_amplitude, z_bias - z_amplitude, 0.0, 0.0, 0.0, 0.0, t_, 1,
                                _rd.c_U_S_d(1), _rd.cDot_U_S_d(1), _rd.cDDot_U_S_d(1));
            }else{
                plan::quintic_s(0.0, 0.5*T_cycle, z_bias - z_amplitude, z_bias + z_amplitude, 0.0, 0.0, 0.0, 0.0, t_ - 0.5*T_cycle, 1,
                                _rd.c_U_S_d(1), _rd.cDot_U_S_d(1), _rd.cDDot_U_S_d(1));
            }

        }else{
            _rd.pitch_d = 0.0;
            _rd.pitchDot_d = 0.0;
            _rd.pitchDDot_d = 0.0;

            _rd.c_U_S_d(0) = _rd.X_offset;
            _rd.cDot_U_S_d(0) = 0.;
            _rd.cDDot_U_S_d(0) = 0.;

            _rd.c_U_S_d(1) = _rd.H_tgt;
            _rd.cDot_U_S_d(1) = 0.;
            _rd.cDDot_U_S_d(1) = 0.;
        }
    }else{
        plan::quintic_s(0.0, t_start, _rs.pitch_0, 0.0, 0.0, 0.0, 0.0, 0.0, _rsm.t, 1,
                        _rd.pitch_d, _rd.pitchDot_d, _rd.pitchDDot_d);

        plan::quintic_s(0.0, t_start, _rs.c_U_S_0(0), _rd.X_offset, _rs.cDot_U_S_0(0), 0.0, 0.0, 0.0, _rsm.t, 1,
                        _rd.c_U_S_d(0), _rd.cDot_U_S_d(0), _rd.cDDot_U_S_d(0));

        if(_rsm.tick == 0){
            _rd.H_tgt_pre = H_tgt_0;
        }
        plan::quintic_s(0.0, t_start, _rd.H_tgt_pre, _rd.H_tgt, _rs.cDot_U_S_0(1), 0.0, 0.0, 0.0, _rsm.t, 1,
                        _rd.c_U_S_d(1), _rd.cDot_U_S_d(1), _rd.cDDot_U_S_d(1));
    }

    // -------------------------- end of Upper body plan ------------------------------

    // -------------------------------- foot plan ---------------------------------------
    /*
     *  Motive : for 'cPfDDotRef'
     *      update  'c_R_S_d', 'cDot_R_S_d', 'cDDot_R_S_d'
     *              'c_L_S_d', 'cDot_L_S_d', 'cDDot_L_S_d'
     */

    if (_rsm.stanceLeg == 1){
        // former swing foot
        _rd.c_R_S_d << 2.0*_rd.X_offset, 0.0;
        _rd.cDot_R_S_d = Eigen::Vector2d::Zero();
        _rd.cDDot_R_S_d = Eigen::Vector2d::Zero();
        _rd.c_R_W_d = _rd.c_R_S_d - _rd.c_U_S_d;
        _rd.cDot_R_W_d = _rd.cDot_R_S_d - _rd.cDot_U_S_d;
        // nominal stance foot
        _rd.c_L_S_d = Eigen::Vector2d::Zero();
        _rd.cDot_L_S_d = Eigen::Vector2d::Zero();
        _rd.cDDot_L_S_d = Eigen::Vector2d::Zero();
        _rd.c_L_W_d = _rd.c_L_S_d - _rd.c_U_S_d;
        _rd.cDot_L_W_d = _rd.cDot_L_S_d - _rd.cDot_U_S_d;
    }else{
        // former swing foot
        _rd.c_L_S_d << 2.0*_rd.X_offset, 0.0;
        _rd.cDot_L_S_d = Eigen::Vector2d::Zero();
        _rd.cDDot_L_S_d = Eigen::Vector2d::Zero();
        _rd.c_L_W_d = _rd.c_L_S_d - _rd.c_U_S_d;
        _rd.cDot_L_W_d = _rd.cDot_L_S_d - _rd.cDot_U_S_d;
        // nominal stance foot
        _rd.c_R_S_d = Eigen::Vector2d::Zero();
        _rd.cDot_R_S_d = Eigen::Vector2d::Zero();
        _rd.cDDot_R_S_d = Eigen::Vector2d::Zero();
        _rd.c_R_W_d = _rd.c_R_S_d - _rd.c_U_S_d;
        _rd.cDot_R_W_d = _rd.cDot_R_S_d - _rd.cDot_U_S_d;
    }
    // ------------------------------ end of foot plan ----------------------------------

    // ----------------------------- GRF distribution ----------------------------------
    /*
     *  Motive : for 'forcePfRef'
     *      update  'fc_grf_R_S_ff', 'fc_grf_L_S_ff'
     */
//        double fz_grf_S_ff = massTotal*(GRAVITY + cDDot_Com_S_d(1));
    double fz_grf_S_ff = _rs.massTotal*GRAVITY;
    if (_rsm.stanceLeg == 1){
        _rd.fc_grf_R_S_ff(1) = fz_grf_S_ff * 0.5;
        _rd.fc_grf_R_S_ff(0) = _rd.fc_grf_R_S_ff(1) * (_rs.c_U_S(0) - 2.0*_rd.X_offset) / _rs.c_U_S(1);
        _rd.fc_grf_L_S_ff(1) = fz_grf_S_ff * 0.5;
        _rd.fc_grf_L_S_ff(0) = _rd.fc_grf_L_S_ff(1) * _rs.c_U_S(0) / _rs.c_U_S(1);
    }else{
        _rd.fc_grf_L_S_ff(1) = fz_grf_S_ff * 0.5;
        _rd.fc_grf_L_S_ff(0) = _rd.fc_grf_L_S_ff(1) * (_rs.c_U_S(0) - 2.0*_rd.X_offset) / _rs.c_U_S(1);
        _rd.fc_grf_R_S_ff(1) = fz_grf_S_ff * 0.5;
        _rd.fc_grf_R_S_ff(0) = _rd.fc_grf_R_S_ff(1) * _rs.c_U_S(0) / _rs.c_U_S(1);
    }
    // -------------------------- end of GRF distribution ------------------------------

    // ------------------------- desired pitch ---------------------------------
    // update Ry_d
    _rd.Ry_d << std::cos(_rd.pitch_d), std::sin(_rd.pitch_d),
                -std::sin(_rd.pitch_d), std::cos(_rd.pitch_d);
    // update RyDot_d
    _rd.RyDot_d << -std::sin(_rd.pitch_d)*_rd.pitchDot_d, std::cos(_rd.pitch_d)*_rd.pitchDot_d,
                   -std::cos(_rd.pitch_d)*_rd.pitchDot_d, -std::sin(_rd.pitch_d)*_rd.pitchDot_d;
    // ---------------------- end of desired pitch ------------------------------

    // PID of PV
    if (_rsm.stanceLeg == 1){
        _rd.p_pid_frrl.assign(_rd.p_pid_Lst.begin(), _rd.p_pid_Lst.end());
        _rd.i_pid_frrl.assign(_rd.i_pid_Lst.begin(), _rd.i_pid_Lst.end());
        _rd.d_pid_frrl.assign(_rd.d_pid_Lst.begin(), _rd.d_pid_Lst.end());
    }else{
        _rd.p_pid_frrl.assign(_rd.p_pid_Rst.begin(), _rd.p_pid_Rst.end());
        _rd.i_pid_frrl.assign(_rd.i_pid_Rst.begin(), _rd.i_pid_Rst.end());
        _rd.d_pid_frrl.assign(_rd.d_pid_Rst.begin(), _rd.d_pid_Rst.end());
    }

    return true;
}
