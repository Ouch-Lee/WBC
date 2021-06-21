//
// Created by jun on 2020-8-19.
//

#include "stateEstimation.h"

GrfDector::GrfDector(){
    forceDotBuffer.resize(bufferVolume_);
    forceDotBuffer.assign(bufferVolume_, 0.0);
    force_pre_ = 0.0;
    riseFlag_ = -1;
}

GrfDector::GrfDector(int buffer_volume, double critical_slope){
    if (buffer_volume >=3){
        bufferVolume_ = buffer_volume;
    }else{
        bufferVolume_ = 3;
    }
    forceDotBuffer.resize(bufferVolume_);
    forceDotBuffer.assign(bufferVolume_, 0.0);
    force_pre_ = 0.0;
    riseFlag_ = -1;
    criticalSlope_ = critical_slope;
}

bool GrfDector::update(double s, double force){
    if (s >= 0 - EPSILON && s <= 0 + EPSILON){
        forceDotBuffer.resize(bufferVolume_);
        forceDotBuffer.assign(bufferVolume_, 0.0);
        force_pre_ = 0.0;
        riseFlag_ = -1;
    }else{
        forceDot_ = (force - force_pre_)/DT;
        forceDotBuffer.push_front(forceDot_);
        forceDotBuffer.pop_back();
        force_pre_ = force;
    }

    if (s > 0.8){
        if (forceDotBuffer.at(0) > criticalSlope_ && forceDotBuffer.at(1) > criticalSlope_ && forceDotBuffer.at(2) > criticalSlope_){
            riseFlag_ = 1;
        }else{
            riseFlag_ = -1;
        }
    }else {
        riseFlag_ = -1;
    }

    return true;
}

double GrfDector::getRiseFlag(){
    return riseFlag_;
}

stateEstimation::stateEstimation(){
    dl2D = new DiaKine::DiamondLeg2D();
    vGrfDector = new GrfDector(2, 0.8/DT);  // 0.8/DT ~ 1.0/DT
    com = Eigen::VectorXd::Zero(2);
    comDot = Eigen::VectorXd::Zero(2);
}

stateEstimation::~stateEstimation(){
    delete dl2D;
    delete vGrfDector;
    dl2D = nullptr;
    vGrfDector = nullptr;
}

bool stateEstimation::readSensor(const double * imu_data, const double * j_pos, const double * j_vel , const double * j_tor,
                                 robotState & _rs) {

    // ------------------------------- Read feedback information ----------------------------
    // Read actual pitch & pitchDot
    _rs.pitch = -imu_data[1];
    _rs.pitchDot = -imu_data[7];

    // Read actual joint position, velocity, torque, output in "frrl"
    _rs.q_R_A << -j_pos[1] + 2 * PI - _rs.qZero_frrl[0],
                 -j_pos[0] + 2 * PI - _rs.qZero_frrl[1];
    _rs.q_L_A << j_pos[2] + _rs.qZero_frrl[2],
                 j_pos[3] + _rs.qZero_frrl[3];

    _rs.qDot_R_A << -j_vel[1],
                    -j_vel[0];
    _rs.qDot_L_A << j_vel[2],
                    j_vel[3];

    _rs.u_R_A << -j_tor[1],
                 -j_tor[0];
    _rs.u_L_A << j_tor[2],
                 j_tor[3];
    // -------------------------- end of Read feedback information --------------------------

    // ---------------------------- Convert to RobotDynamics_Mario2D coordinate ---------------------------
    _rs.q_(2) = _rs.pitch;
    _rs.q_(3) = _rs.q_R_A(0);
    _rs.q_(5) = _rs.q_R_A(1);
    _rs.q_(7) = _rs.q_L_A(0);
    _rs.q_(9) = _rs.q_L_A(1);

    _rs.qDot_(2) = _rs.pitchDot;
    _rs.qDot_(3) = _rs.qDot_R_A(0);
    _rs.qDot_(5) = _rs.qDot_R_A(1);
    _rs.qDot_(7) = _rs.qDot_L_A(0);
    _rs.qDot_(9) = _rs.qDot_L_A(1);

//    _rs.u_(3) = _rs.u_R_A(0);
//    _rs.u_(5) = _rs.u_R_A(1);
//    _rs.u_(7) = _rs.u_L_A(0);
//    _rs.u_(9) = _rs.u_L_A(1);
    // ------------------------ end of Convert to RobotDynamics_Mario2D coordinate -------------------------

    return true;
}

bool stateEstimation::readSensor(const double * imu_data, const double * j_pos, const double * j_vel , const double * j_tor,
                                       const double *grf, robotState & _rs) {

    // ------------------------------- Read feedback information ----------------------------
    // direction is the same as simulation in Debug
    // Read actual pitch & pitchDot
    _rs.pitch = imu_data[1];
    _rs.pitchDot = imu_data[7];

    // Read actual joint position, velocity, torque, output in "frrl"
    _rs.q_R_A << j_pos[0],
                j_pos[1];
    _rs.q_L_A << j_pos[2],
                j_pos[3];

    _rs.qDot_R_A << j_vel[0],
                    j_vel[1];
    _rs.qDot_L_A << j_vel[2],
                    j_vel[3];

    _rs.u_R_A << j_tor[0],
                j_tor[1];
    _rs.u_L_A << j_tor[2],
                j_tor[3];

    _rs.fc_grf_R_S << grf[0],
                    grf[2];
    _rs.fc_grf_L_S << grf[3],
                    grf[5];
    // -------------------------- end of Read feedback information --------------------------

    // ---------------------------- Convert to RobotDynamics_Mario2D coordinate ---------------------------
    _rs.q_(2) = _rs.pitch;
    _rs.q_(3) = _rs.q_R_A(0);
    _rs.q_(5) = _rs.q_R_A(1);
    _rs.q_(7) = _rs.q_L_A(0);
    _rs.q_(9) = _rs.q_L_A(1);

    _rs.qDot_(2) = _rs.pitchDot;
    _rs.qDot_(3) = _rs.qDot_R_A(0);
    _rs.qDot_(5) = _rs.qDot_R_A(1);
    _rs.qDot_(7) = _rs.qDot_L_A(0);
    _rs.qDot_(9) = _rs.qDot_L_A(1);

//    _rs.u_(3) = _rs.u_R_A(0);
//    _rs.u_(5) = _rs.u_R_A(1);
//    _rs.u_(7) = _rs.u_L_A(0);
//    _rs.u_(9) = _rs.u_L_A(1);
    // ------------------------ end of Convert to RobotDynamics_Mario2D coordinate -------------------------

    return true;
}


bool stateEstimation::stateDetecting(robotStateMachine & _rsm, robotState &_rs){

    /**
     *  Estimation of actual generalized forces
     *  Static Force Jacobian relationship:
     *      tau = Jc' * Fc_B;
     *      tau is the actual joint torque applying to the loads, contributed by joint actuator. i.e. tor[*]
     *  Static Force Jacobian relationship:
     *      Fc_W = R_WB * Fc_B
     *  Compute the grf_S applied in each leg:
     *      grf_S = - Fc_W
     */
    if (_rsm.debug_flag != 1){
        // jacobian
        _rs.Jc_R = dl2D->jacoCartes2D(_rs.q_R_A);
        _rs.Jc_L = dl2D->jacoCartes2D(_rs.q_L_A);
        // update Ry
        _rs.Ry << std::cos(_rs.pitch), std::sin(_rs.pitch),
                  -std::sin(_rs.pitch), std::cos(_rs.pitch);
        // calculate grf
        _rs.fc_grf_R_S = - _rs.Ry * ((_rs.Jc_R.transpose()).inverse()) * _rs.u_R_A;
        _rs.fc_grf_L_S = - _rs.Ry * ((_rs.Jc_L.transpose()).inverse()) * _rs.u_L_A;
    }
    // Scaling factors representing the magnitude of force in each leg
    _rsm.s_R = plan::scaleFactor(_rs.fc_grf_R_S(1), thres_lo, thres_hi);
    _rsm.s_L = plan::scaleFactor(_rs.fc_grf_L_S(1), thres_lo, thres_hi);
    if (_rsm.stanceLeg == 1){
        _rsm.s_st = _rsm.s_L;
        _rsm.s_sw = _rsm.s_R;
        _rs.fc_grf_st_S = _rs.fc_grf_L_S;
        _rs.fc_grf_sw_S = _rs.fc_grf_R_S;
    }else{
        _rsm.s_st = _rsm.s_R;
        _rsm.s_sw = _rsm.s_L;
        _rs.fc_grf_st_S = _rs.fc_grf_R_S;
        _rs.fc_grf_sw_S = _rs.fc_grf_L_S;
    }

    // update 's' for SSP
    _rsm.s = plan::clamp(_rsm.t/_rsm.Ts, 0, 1.1);
    if (_rsm.init_flag == 1){
        _rsm.s = _rsm.s + 0.5;
    }
    // Detect Touch-Down
    vGrfDector->update(_rsm.s, _rs.fc_grf_sw_S(1));
    if (_rsm.TD_flag == 0 &&
            (((_rsm.s_sw_pre <= 0 + EPSILON) && (_rsm.s_sw > 0 + EPSILON) && (_rsm.s > 0.8)  && (_rsm.s < 1 - EPSILON))
             || vGrfDector->getRiseFlag() > 0
             || _rsm.s >= 1 - EPSILON)){
        _rsm.TD_flag = 1;
        _rsm.s_td = _rsm.s;
        _rsm.t_td = _rsm.t;
        _rs.vx_est_pre = _rs.vx_est;
        _rsm.state = 2;                      // go to DSP

        // Marker Conversion
        _rsm.remark_flag = 1;                // remark the Marker in this control frame
        _rsm.stanceLeg = - _rsm.stanceLeg;        // Switch stance legs
    }

    // update 'ss' for DSP
    _rsm.tt = _rsm.t - _rsm.t_td;
    _rsm.ss = plan::clamp(_rsm.tt/_rsm.Td, 0, 1);
    // Detect Lift-Off : 'TD_flag == 1' and 'ss >= 1'
    if (_rsm.TD_flag == 1 && _rsm.ss >= 1 - EPSILON){
        _rsm.TD_flag = 0;
        _rsm.s_td = 0.0;
        _rsm.t_td = _rsm.Ts + _rsm.Td;

        // step update
        _rsm.steps_total = _rsm.steps_total +1;   // update steps_total
        _rsm.t = 0.0;                        // Reset time for new SSP
        _rsm.init_flag = 0;                  // after the first step
        _rsm.state = 1;                      // go to SSP
    }

    _rsm.s_sw_pre = _rsm.s_sw;                    // update 's_sw_pre' value for the next Detection

    return true;
}

bool stateEstimation::kinematicsCalculating(robotStateMachine &_rsm, robotState &_rs, RobotDynamics_Mario2D *mario){

    // update Ry
    _rs.Ry << std::cos(_rs.pitch), std::sin(_rs.pitch),
          -std::sin(_rs.pitch), std::cos(_rs.pitch);
    // update RyDot
    _rs.RyDot << -std::sin(_rs.pitch)*_rs.pitchDot, std::cos(_rs.pitch)*_rs.pitchDot,
                 -std::cos(_rs.pitch)*_rs.pitchDot, -std::sin(_rs.pitch)*_rs.pitchDot;

    /**
     *  Forward Kinematic in Cartesian frame, c = [x; z]
     *  c_W = R_WB * c_B ; dc_W = R_WB * dc_B + dR_WB * c_B ; dc_B = Jc * dq
     *
     *  c_W: Cartesian position of footpoints relative to Body origin in world frame (W)
     *  c_B: Cartesian position of footpoints relative to Body origin in body frame (B)
     *  R_WB: here, Rotation Transformation of pitch, in 2D
     */
    // Right leg
    _rs.c_R_B = dl2D->fkPosCartes2D(_rs.q_R_A);
    _rs.c_R_W = _rs.Ry * _rs.c_R_B;
    _rs.Jc_R = dl2D->jacoCartes2D(_rs.q_R_A);
    _rs.cDot_R_B = _rs.Jc_R * _rs.qDot_R_A;
    _rs.cDot_R_W = _rs.Ry * _rs.cDot_R_B + _rs.RyDot * _rs.c_R_B;
    // Left leg
    _rs.c_L_B = dl2D->fkPosCartes2D(_rs.q_L_A);
    _rs.c_L_W = _rs.Ry * _rs.c_L_B;
    _rs.Jc_L = dl2D->jacoCartes2D(_rs.q_L_A);
    _rs.cDot_L_B = _rs.Jc_L * _rs.qDot_L_A;
    _rs.cDot_L_W = _rs.Ry * _rs.cDot_L_B + _rs.RyDot * _rs.c_L_B;

    /**
     * c_S: Cartesian position of footpoints relative to Footpoint of Supporting in inertial frame (S)
     */
    if (_rsm.stanceLeg == 1){
        _rs.c_L_S = Eigen::Vector2d::Zero();
        _rs.c_R_S = _rs.c_R_W - _rs.c_L_W;
        _rs.c_U_S = - _rs.c_L_W;
//        _rs.cDot_L_S = Eigen::Vector2d::Zero();
//        _rs.cDot_R_S = _rs.cDot_R_W - _rs.cDot_L_W;
//        _rs.cDot_U_S = - _rs.cDot_L_W;
        if (_rsm.state == 1){   // SSP
            _rs.cDot_L_S = Eigen::Vector2d::Zero();
            _rs.cDot_R_S = _rs.cDot_R_W - _rs.cDot_L_W;
            _rs.cDot_U_S = - _rs.cDot_L_W;
        }else {                 // DSP
            plan::line_s(0., _rsm.Td, -_rs.cDot_R_W, -_rs.cDot_L_W, _rsm.tt, 1., _rs.cDot_U_S);
//            _rs.cDot_U_S = 0.5*(- _rs.cDot_L_W - _rs.cDot_R_W);
            _rs.cDot_L_S = _rs.cDot_L_W + _rs.cDot_U_S;
            _rs.cDot_R_S = _rs.cDot_R_W + _rs.cDot_U_S;
        }
    }else {
        _rs.c_R_S = Eigen::Vector2d::Zero();
        _rs.c_L_S = _rs.c_L_W - _rs.c_R_W;
        _rs.c_U_S = - _rs.c_R_W;
//        _rs.cDot_R_S = Eigen::Vector2d::Zero();
//        _rs.cDot_L_S = _rs.cDot_L_W - _rs.cDot_R_W;
//        _rs.cDot_U_S = - _rs.cDot_R_W;
        if (_rsm.state == 1){   // SSP
            _rs.cDot_R_S = Eigen::Vector2d::Zero();
            _rs.cDot_L_S = _rs.cDot_L_W - _rs.cDot_R_W;
            _rs.cDot_U_S = - _rs.cDot_R_W;
        }else {                 // DSP
            plan::line_s(0., _rsm.Td, -_rs.cDot_L_W, -_rs.cDot_R_W, _rsm.tt, 1., _rs.cDot_U_S);
//            _rs.cDot_U_S = 0.5*(- _rs.cDot_L_W - _rs.cDot_R_W);
            _rs.cDot_L_S = _rs.cDot_L_W + _rs.cDot_U_S;
            _rs.cDot_R_S = _rs.cDot_R_W + _rs.cDot_U_S;
        }
    }

    // calculate underactuated-joints : Knees. for RobotDynamics_Mario2D coordinate
    double q_Knee_fore_R, q_Knee_rear_R, qdot_Knee_fore_R, qdot_Knee_rear_R;
    dl2D->calHipKnee2D(_rs.q_R_A, _rs.qDot_R_A, _rs.c_R_B, _rs.cDot_R_B, q_Knee_fore_R, q_Knee_rear_R, qdot_Knee_fore_R, qdot_Knee_rear_R);
    double q_Knee_fore_L, q_Knee_rear_L, qdot_Knee_fore_L, qdot_Knee_rear_L;
    dl2D->calHipKnee2D(_rs.q_L_A, _rs.qDot_L_A, _rs.c_L_B, _rs.cDot_L_B, q_Knee_fore_L, q_Knee_rear_L, qdot_Knee_fore_L, qdot_Knee_rear_L);

    // ---------------------------- Convert to RobotDynamics_Mario2D coordinate ---------------------------
    _rs.q_.head(2) = _rs.c_U_S;
    _rs.q_(4) = q_Knee_fore_R;
    _rs.q_(6) = q_Knee_rear_R;
    _rs.q_(8) = q_Knee_fore_L;
    _rs.q_(10) = q_Knee_rear_L;
    _rs.qDot_.head(2) = _rs.cDot_U_S;
    _rs.qDot_(4) = qdot_Knee_fore_R;
    _rs.qDot_(6) = qdot_Knee_rear_R;
    _rs.qDot_(8) = qdot_Knee_fore_L;
    _rs.qDot_(10) = qdot_Knee_rear_L;
    // ------------------------ end of Convert to RobotDynamics_Mario2D coordinate -------------------------

    // Emergency / Mechanical joint limits
    if(q_Knee_fore_R <= _rsm.q_Knee_Margin || q_Knee_rear_R >= -_rsm.q_Knee_Margin
            || q_Knee_fore_L <= _rsm.q_Knee_Margin || q_Knee_rear_L >= -_rsm.q_Knee_Margin){
        _rsm.emergency_flag = 1;
    }
//    else{
//        _rsm.emergency_flag = 0;
//    }
    // end of Emergency / Mechanical joint limits

    // calculate CoM positon & velocity in "S"
    mario->setQ_Qdot(_rs.q_, _rs.qDot_);  
    mario->centerOfMassPosVel(com, comDot);
    _rs.c_Com_S = com;
    _rs.cDot_Com_S = comDot;
//    _rs.cDDot_Com_S = 1./DT*(_rs.cDot_Com_S - _rs.cDot_Com_S_pre);
//    _rs.cDot_Com_S_pre = _rs.cDot_Com_S;
    // Com in "W"
    _rs.c_Com_W = _rs.c_Com_S - _rs.c_U_S;
    _rs.cDot_Com_W = _rs.cDot_Com_S - _rs.cDot_U_S;
    // frame "G" : origin is center of mass, orientation is the same as "I","S","W"
    _rs.c_R_G = _rs.c_R_S - _rs.c_Com_S;
    _rs.c_L_G = _rs.c_L_S - _rs.c_Com_S;
    _rs.cDot_R_G = _rs.cDot_R_S - _rs.cDot_Com_S;
    _rs.cDot_L_G = _rs.cDot_L_S - _rs.cDot_Com_S;

    return true;
}

bool stateEstimation::stateEstimating(robotStateMachine &_rsm, robotState &_rs, RobotDynamics_Mario2D * mario){

    // update Kinematics firstly
    kinematicsCalculating(_rsm, _rs, mario);

    // update 's' for SSP, 'ss' for DSP
    if (_rsm.behavior_flag == 3){                   // walking
        _rsm.s = plan::clamp(_rsm.t/_rsm.Ts, 0, 1.1);
        _rsm.tt = _rsm.t - _rsm.t_td;
        _rsm.ss = plan::clamp(_rsm.tt/_rsm.Td, 0, 1);
    }else if (_rsm.behavior_flag == 2){             // test in air
        _rsm.s = plan::clamp(_rsm.t/_rsm.Ts_testInAir, 0, 1);
        if (_rsm.init_flagTestInAir == 1){
            _rsm.s = 0.5 + _rsm.s;
        }
        _rsm.tt = 0.0;
        _rsm.ss = 0.0;
    }else if (_rsm.behavior_flag == 5){             // test squat
        _rsm.s = plan::clamp(_rsm.t/_rsm.Ts_testSquat, 0, 1);
        _rsm.tt = 0.0;
        _rsm.ss = 0.0;
    }else if (_rsm.behavior_flag == 6){             // stand balance
        _rsm.s = -1.0;
        _rsm.tt = _rsm.t;
        _rsm.ss = 0.0;
    }else{                                          // other cases
        _rsm.s = -1.0;
        _rsm.tt = 0.0;
        _rsm.ss = 0.0;
    }

    if (_rsm.remark_flag == 1){     // at the moment "TD"
        // ---------------------------- Remark ST & SW coordinate ---------------------------
        if (_rsm.stanceLeg == 1){
            _rsm.s_st = _rsm.s_L;
            _rsm.s_sw = _rsm.s_R;
            _rs.fc_grf_st_S = _rs.fc_grf_L_S;
            _rs.fc_grf_sw_S = _rs.fc_grf_R_S;
        }else{
            _rsm.s_st = _rsm.s_R;
            _rsm.s_sw = _rsm.s_L;
            _rs.fc_grf_st_S = _rs.fc_grf_R_S;
            _rs.fc_grf_sw_S = _rs.fc_grf_L_S;
        }
        // ------------------------ end of Remark ST & SW coordinate -------------------------
    }

    // grf_S estimation for non-walking case (behavior_flag ! = 3)
    if (_rsm.behavior_flag != 3){
        if (_rsm.debug_flag != 1){
            _rs.fc_grf_R_S = - _rs.Ry * ((_rs.Jc_R.transpose()).inverse()) * _rs.u_R_A;
            _rs.fc_grf_L_S = - _rs.Ry * ((_rs.Jc_L.transpose()).inverse()) * _rs.u_L_A;
        }
        // Scaling factors representing the magnitude of force in each leg
        _rsm.s_R = plan::scaleFactor(_rs.fc_grf_R_S(1), thres_lo, thres_hi);
        _rsm.s_L = plan::scaleFactor(_rs.fc_grf_L_S(1), thres_lo, thres_hi);
        if (_rsm.stanceLeg == 1){
            _rsm.s_st = _rsm.s_L;
            _rsm.s_sw = _rsm.s_R;
            _rs.fc_grf_st_S = _rs.fc_grf_L_S;
            _rs.fc_grf_sw_S = _rs.fc_grf_R_S;
        }else{
            _rsm.s_st = _rsm.s_R;
            _rsm.s_sw = _rsm.s_L;
            _rs.fc_grf_st_S = _rs.fc_grf_R_S;
            _rs.fc_grf_sw_S = _rs.fc_grf_L_S;
        }
    }
    // end of grf_S estimation

    // ----------------------------- Gait Velocity Estimation -------------------------------

    if (_rsm.vel_flag == 0){
        _rs.p_Center = _rs.c_Com_S;
        _rs.v_Center = _rs.cDot_Com_S;
    }else{
        if (_rsm.stanceLeg == _rsm.innerLeg_flag){
            x_inner = _rs.c_U_S(0);
            _rs.vx_inner = _rs.cDot_U_S(0);
        } else{
            x_outer = _rs.c_U_S(0);
            _rs.vx_outer = _rs.cDot_U_S(0);
        }
        // circular-motion process
        if (_rsm.circular_flag == 0 || _rsm.debug_flag == 1){
            _rs.p_Center(0) = _rs.c_U_S(0);
            _rs.v_Center(0) = _rs.cDot_U_S(0);
        } else{
            _rs.p_Center(0) = _rs.c_U_S(0);
            if (_rsm.stanceLeg == _rsm.innerLeg_flag){
                _rs.v_Center(0) = ratio_CenByInner * _rs.vx_inner;
            } else{
                _rs.v_Center(0) = ratio_CenByOuter * _rs.vx_outer;
            }
        }
        _rs.p_Center(1) = _rs.c_U_S(1);
        _rs.v_Center(1) = _rs.cDot_U_S(1);
    }

    // CoM-velocity : s_confidence = s_st, not the bigger one of s_st and s_sw
    // when " vGRF >= (0.7*(thres_hi - thres_lo) + thres_lo) ~= 0.62mg", the 'dx' & 'dz' is credible
    if (_rsm.behavior_flag == 3){
        thres_confidence = 0.7;
    }else{
        thres_confidence = 0.3;
    }
    if (_rsm.behavior_flag == 3 ){
        // cause x_step_G oscillation
//        if (_rsm.t <= 0.5*_rsm.Ts){
//            sf_vx = 0.03;
//        }else {
//            sf_vx = 0.3;
//        }
        sf_vx = 0.03;   // 0.03
        sf_vz = 0.3;    // 0.3
    }else{
        sf_vx = 0.3;
        sf_vz = 0.3;
    }
//    if (_rsm.s_st >= thres_confidence){
//        _rs.vx_est = _rs.vx_est + sf_vx*(_rs.v_Center(0) - _rs.vx_est)*(std::fabs(_rs.v_Center(0)) < 2);
//        _rs.vx_est_nf = _rs.vx_est_nf + (_rs.v_Center(0) - _rs.vx_est_nf)*(std::fabs(_rs.v_Center(0)) < 2);
//    }
//    if (_rsm.s_st >= thres_confidence){
//        _rs.vz_est = _rs.vz_est + sf_vz*(_rs.v_Center(1) - _rs.vz_est)*(std::fabs(_rs.v_Center(1)) < 2);
//        _rs.vz_est_nf = _rs.vz_est_nf + (_rs.v_Center(1) - _rs.vz_est_nf)*(std::fabs(_rs.v_Center(1)) < 2);
//    }
    _rs.vx_est = _rs.vx_est + sf_vx*(_rs.v_Center(0) - _rs.vx_est)*(std::fabs(_rs.v_Center(0)) < 2);
    _rs.vx_est_nf = _rs.vx_est_nf + (_rs.v_Center(0) - _rs.vx_est_nf)*(std::fabs(_rs.v_Center(0)) < 2);
    _rs.vz_est = _rs.vz_est + sf_vz*(_rs.v_Center(1) - _rs.vz_est)*(std::fabs(_rs.v_Center(1)) < 2);
    _rs.vz_est_nf = _rs.vz_est_nf + (_rs.v_Center(1) - _rs.vz_est_nf)*(std::fabs(_rs.v_Center(1)) < 2);

    // filter : Prevent sudden changes in velocity of CoM or Ub, considering the PD control
    if (_rsm.vel_flag == 0){
        if(_rsm.behavior_flag == 3 || _rsm.behavior_flag == 6){
            _rs.cDot_Com_S << _rs.vx_est, _rs.vz_est;
        }
        _rs.cDDot_Com_S = 1./DT*(_rs.cDot_Com_S - _rs.cDot_Com_S_pre);
        _rs.cDot_Com_S_pre = _rs.cDot_Com_S;
    }else{
        if(_rsm.behavior_flag == 3 || _rsm.behavior_flag == 6){
            _rs.cDot_U_S << _rs.vx_est, _rs.vz_est;
        }
        _rs.cDDot_U_S = 1./DT*(_rs.cDot_U_S - _rs.cDot_U_S_pre);
        _rs.cDot_U_S_pre = _rs.cDot_U_S;
    }
    // ---------------------------- Convert to RobotDynamics_Mario2D coordinate ---------------------------
//    _rs.qDot_.head(2) = _rs.cDot_U_S;
//    // update RBDM
//    mario->resetIsPosVelUpdated();
    // ---------------------------- Convert to RobotDynamics_Mario2D coordinate ---------------------------

    // ---------------------------- Odometry Update ---------------------------
    _rs.pos_odo(0) += _rs.pos_odo_0(0) + _rs.vx_est * DT;
    _rs.pos_odo(1) += _rs.pos_odo_0(1) + _rs.vz_est * DT;
    // ---------------------------- Odometry Update ---------------------------

    // mark initial value
    if (_rsm.tick == 0){
        _rs.pitch_0 = _rs.pitch;
        _rs.pitchDot_0 = _rs.pitchDot;

        _rs.c_R_W_0 = _rs.c_R_W;
        _rs.c_L_W_0 = _rs.c_L_W;
        _rs.c_R_S_0 = _rs.c_R_S;
        _rs.c_L_S_0 = _rs.c_L_S;

        _rs.c_Com_S_0 = _rs.c_Com_S;
        _rs.cDot_Com_S_0 = _rs.cDot_Com_S;
        _rs.c_U_S_0 = _rs.c_U_S;
        _rs.cDot_U_S_0 = _rs.cDot_U_S;

        if (_rsm.vel_flag == 0){
            _rs.p_Center_0(1) = _rs.c_Com_S(1);
        }else{
            _rs.p_Center_0(1) = _rs.c_U_S(1);
        }
        // odo
        _rs.pos_odo_0(0) = 0.;
        _rs.pos_odo_0(1) = _rs.c_U_S_0(1);
        _rs.pos_odo_lastLO = _rs.pos_odo_0;
        _rs.vel_odo_preStep = Eigen::Vector2d::Zero();
    }
    if (_rsm.init_flag == 1){    // Case: "s_init = 0.5"
        _rsm.s = _rsm.s + 0.5;
        if (_rsm.tick == 0){             // mark initial value
            _rs.vx_est = 0.;
            _rs.vx_est_pre = _rs.vx_est;
            _rs.vx_est_nf = 0.;
            _rs.vx_est_nf_pre = _rs.vx_est_nf;
            _rs.vz_est = 0.;
            _rs.vz_est_nf = 0.;
            _rs.vx_full_pre = 0.;
            _rs.vx_full_pre_pre = _rs.vx_full_pre;
            _rs.vx_double = _rs.vx_full_pre;
            // for vx_full_pre & vx_double
            _rs.p_Center_0 = _rs.p_Center;
            _rs.v_Center_0 = _rs.v_Center;
            // odo
            _rs.vel_odo_preStep = Eigen::Vector2d::Zero();
        }
    }

    // set the initial value of new SSP
    if (_rsm.s >= 0 - EPSILON && _rsm.s <= 0 + EPSILON){         // s == 0, a new SSP beginning

        _rs.c_R_W_0 = _rs.c_R_W;
        _rs.c_L_W_0 = _rs.c_L_W;
        _rs.cDot_R_W_0 = _rs.cDot_R_W;
        _rs.cDot_L_W_0 = _rs.cDot_L_W;

        _rs.c_R_G_0 = _rs.c_R_G;
        _rs.c_L_G_0 = _rs.c_L_G;
        _rs.cDot_R_G_0 = _rs.cDot_R_G;
        _rs.cDot_L_G_0 = _rs.cDot_L_G;

        _rs.c_R_S_0 = _rs.c_R_S;
        _rs.c_L_S_0 = _rs.c_L_S;
        _rs.cDot_R_S_0 = _rs.cDot_R_S;
        _rs.cDot_L_S_0 = _rs.cDot_L_S;
        _rs.c_Com_S_0 = _rs.c_Com_S;
        _rs.cDot_Com_S_0 = _rs.cDot_Com_S;
        _rs.c_U_S_0 = _rs.c_U_S;
        _rs.cDot_U_S_0 = _rs.cDot_U_S;

        Eigen::Vector2d c_VectorFromSwingfootToCenter_S = Eigen::Vector2d::Zero();
        if (_rsm.stanceLeg == 1){
            c_VectorFromSwingfootToCenter_S = _rs.p_Center - _rs.c_R_S;
        }else{
            c_VectorFromSwingfootToCenter_S = _rs.p_Center - _rs.c_L_S;
        }

        _rs.xE_DSP = c_VectorFromSwingfootToCenter_S(0);
        _rs.vxE_DSP = _rs.vx_est_nf_pre;

        _rs.x_init_SSP = _rs.c_U_S_0(0);
//        _rs.vx_init_SSP = _rs.cDot_U_S_0(0);      // for walking, _rs.cDot_U_S_0(0) == _rs.vx_est
        _rs.vx_init_SSP = _rs.vx_est;

        // Average Velocity of previous step: from "former LO(or new SSP)" to "current LO(or new SSP)"
        if (_rsm.tick != 0){      // Time > 0, except the first control frame "Time == 0"
            _rs.vx_full_pre_pre = _rs.vx_full_pre;
            _rs.x_est_full_pre = c_VectorFromSwingfootToCenter_S(0) - _rs.p_Center_0(0);
            _rs.vx_full_pre = _rs.x_est_full_pre / (_rsm.Time - _rs.T_full_pre);
            _rs.z_est_full_pre = c_VectorFromSwingfootToCenter_S(1) - _rs.p_Center_0(1);
            _rs.vz_full_pre = _rs.z_est_full_pre / (_rsm.Time - _rs.T_full_pre);
            _rs.vx_double = 0.5*(_rs.vx_full_pre_pre + _rs.vx_full_pre);
            // for odometry
            _rs.vel_odo_preStep(0) = (_rs.pos_odo(0) - _rs.pos_odo_lastLO(0)) / (_rsm.Time - _rs.T_full_pre);
            _rs.vel_odo_preStep(1) = (_rs.pos_odo(1) - _rs.pos_odo_lastLO(1)) / (_rsm.Time - _rs.T_full_pre);
            _rs.pos_odo_lastLO = _rs.pos_odo;
            // reset time
            _rs.T_full_pre = _rsm.Time;

            // vx_preStep
            _rs.vx_preStep = _rs.vel_odo_preStep(0);            // vel_odo_preStep(0) OR vx_full_pre
        }

        if (_rsm.tick == 0){
            _rs.x_est_total_pre = _rs.p_Center(0);
            _rs.z_est_total_pre = _rs.p_Center(1);
        }else{
            _rs.x_est_total_pre = _rs.x_est_total_pre + _rs.x_est_full_pre;
            _rs.z_est_total_pre = _rs.z_est_total_pre + _rs.z_est_full_pre;
        }
        _rs.p_Center_0 = _rs.p_Center;
        _rs.v_Center_0 = _rs.v_Center;
    }

    // Update upperBody or CoM position estimates
    _rs.x_est_total = _rs.x_est_total_pre + (_rs.p_Center(0) - _rs.p_Center_0(0));
    _rs.z_est_total = _rs.z_est_total_pre + (_rs.p_Center(1) - _rs.p_Center_0(1));

    // --------------------------- end of Gait Velocity Estimation -------------------------------

    // set the initial value of new DSP
    if (_rsm.TD_flag == 1 && _rsm.s >= _rsm.s_td - EPSILON && _rsm.s <= _rsm.s_td + EPSILON){        // s == s_td, a new DSP beginning

        _rs.c_R_W_td = _rs.c_R_W;
        _rs.c_L_W_td = _rs.c_L_W;
        _rs.cDot_R_W_td = _rs.cDot_R_W;
        _rs.cDot_L_W_td = _rs.cDot_L_W;

        _rs.c_R_G_td = _rs.c_R_G;
        _rs.c_L_G_td = _rs.c_L_G;
        _rs.cDot_R_G_td = _rs.cDot_R_G;
        _rs.cDot_L_G_td = _rs.cDot_L_G;

        _rs.c_R_S_td = _rs.c_R_S;
        _rs.c_L_S_td = _rs.c_L_S;
        _rs.cDot_R_S_td = _rs.cDot_R_S;
        _rs.cDot_L_S_td = _rs.cDot_L_S;
        _rs.c_Com_S_td = _rs.c_Com_S;
        _rs.cDot_Com_S_td = _rs.cDot_Com_S;
        _rs.c_U_S_td = _rs.c_U_S;
        _rs.cDot_U_S_td = _rs.cDot_U_S;

        _rs.xE_SSP = _rs.p_Center(0);
        _rs.vxE_SSP = _rs.vx_est_nf_pre;

        if (_rsm.stanceLeg == 1){
            _rs.deltaC_foot2_minus_foot1 = _rs.c_R_S_td - _rs.c_L_S_td;
        }else{
            _rs.deltaC_foot2_minus_foot1 = _rs.c_L_S_td - _rs.c_R_S_td;
        }

    }

    return true;
}
