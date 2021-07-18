//
// Created by jun on 2020-8-19.
//

#include "taskControl.h"

taskControl::taskControl(RobotDynamics_Mario2D * robotRBDM, int qpFlag){

    robot_ = robotRBDM;
    qp_flag_ = qpFlag;

    dl2D = new DiaKine::DiamondLeg2D();

    // for wbc
    fcLowerBounds = Eigen::Vector4d::Zero();
    fcUpperBounds = Eigen::Vector4d::Zero();

    /// weight
    weightVec_CentroidalMoment = Eigen::VectorXd::Zero(nJf);                    // 3*1
    weightVec_FloatingBasePose = Eigen::VectorXd::Zero(nJf);                    // 3*1
    weightVec_PointFeetCartPosition = Eigen::VectorXd::Zero(nFc);               // 4*1
    weightVec_PointFeetCartForce = Eigen::VectorXd::Zero(nFc);                  // 4*1
    weightVec_InternalClosedLoop = Eigen::VectorXd::Zero(nFc);                  // 4*1
    weightVec_TorqueChange = Eigen::VectorXd::Zero(nJa);                        // 4*1
    weightVec_ForceChange = Eigen::VectorXd::Zero(nFc);                        // 4*1

    /// bounds
    boundsVec_lb = Eigen::VectorXd::Zero(nV);                                  // nV*1
    boundsVec_ub = Eigen::VectorXd::Zero(nV);                                  // nV*1


    // Task & Constraint

    Task * Mario2dCentroidalMoment_ptr = new Mario2dCentroidalMoment("Mario2dCentroidalMoment", 3, nV);
    Task * Mario2dFloatingBasePose_ptr = new Mario2dFloatingBasePose("Mario2dFloatingBasePose", 3, nV);
    Task * Mario2dPointFeetCartPosition_ptr = new Mario2dPointFeetCartPosition("Mario2dPointFeetCartPosition", 4, nV);
    Task * Mario2dPointFeetCartForce_ptr = new Mario2dPointFeetCartForce("Mario2dPointFeetCartForce", 4, nV);
    Task * Mario2dInternalClosedLoop_ptr = new Mario2dInternalClosedLoop("Mario2dInternalClosedLoop", 4, nV);
    Task * Mario2dJointTorqueChange_ptr = new Mario2dJointTorqueChange("Mario2dJointTorqueChange", 4, nV);
    Task * Mario2dFeetForceChange_ptr = new Mario2dFeetForceChange("Mario2dFeetForceChange", 4, nV);

    Mario2dPointFeetCartPosition_ptr->setParameter(std::vector<double>{-1.});  // JcTruncation == false

    Constraint * Mario2dDynamicConsistency_ptr = new Mario2dDynamicConsistency("Mario2dDynamicConsistency", 3, nV);
    Constraint * Mario2dFrictionCone_ptr = new Mario2dFrictionCone("Mario2dFrictionCone", 4, nV);
    Constraint * Mario2dJointTorqueSaturation_ptr = new Mario2dJointTorqueSaturation("Mario2dJointTorqueSaturation", 4, nV);
    Constraint * Mario2dKneeSingularity_ptr = new Mario2dKneeSingularity("Mario2dKneeSingularity", 4, nV);
    Constraint * Mario2dCenterOfPressure_ptr = new Mario2dCenterOfPressure("Mario2dCenterOfPressure", 2, nV);

    Mario2dFrictionCone_ptr->setParameter(std::vector<double>{mu_static_, myInfinity_});
    Mario2dJointTorqueSaturation_ptr->setParameter(std::vector<double>{jointTau_limit_});
    Mario2dKneeSingularity_ptr->setParameter(std::vector<double>{delta_t_, knee_absMin_, knee_absMax_});
    Mario2dCenterOfPressure_ptr->setParameter(std::vector<double>{rear_flag_, distance_feet_, x_cop_lb_, x_cop_ub_});


    // =============================== create wqp_WBC ======================================
    wbc_ = new WqpWbc(nV, robot_);

    wbc_->addTask(Mario2dCentroidalMoment_ptr, 0);
    wbc_->addTask(Mario2dFloatingBasePose_ptr, 0);
    wbc_->addTask(Mario2dPointFeetCartPosition_ptr, 0);
    wbc_->addTask(Mario2dPointFeetCartForce_ptr, 0);
    wbc_->addTask(Mario2dInternalClosedLoop_ptr, 0);
    wbc_->addTask(Mario2dJointTorqueChange_ptr, 0);
    wbc_->addTask(Mario2dFeetForceChange_ptr, 0);

    wbc_->addConstraint(Mario2dDynamicConsistency_ptr, 0);
    wbc_->addConstraint(Mario2dFrictionCone_ptr, 0);
    wbc_->addConstraint(Mario2dJointTorqueSaturation_ptr, 0);
    wbc_->addConstraint(Mario2dKneeSingularity_ptr, 0);
//    wbc_->addConstraint(Mario2dCenterOfPressure_ptr, 0);

    wbc_->wbcInit();

    wbc_->displayWbcInformation();
    wbc_->setParametersInt(nWSR_w_);
    wbc_->setParametersDouble(cpuTime_w_);
    // =============================== create wqp_WBC ======================================

    // =============================== create hqp_WBC ======================================
    dataOptInt.resize(8);
    dataOptDouble.resize(8);
    iterOpt.resize(4);
    successOpt.resize(4);
    costOpt.resize(4);
    timeOpt.resize(4);

    wbch_ = new WqpWbc(nV, robot_);

    wbch_->addTask(Mario2dCentroidalMoment_ptr, 2);
    wbch_->addTask(Mario2dFloatingBasePose_ptr, 3);
    wbch_->addTask(Mario2dPointFeetCartPosition_ptr, 1);
    wbch_->addTask(Mario2dPointFeetCartForce_ptr, 3);
    wbch_->addTask(Mario2dInternalClosedLoop_ptr, 0);
    wbch_->addTask(Mario2dFeetForceChange_ptr, 3);

    wbch_->addConstraint(Mario2dDynamicConsistency_ptr, 0);
    wbch_->addConstraint(Mario2dFrictionCone_ptr, 1);
    wbch_->addConstraint(Mario2dJointTorqueSaturation_ptr, 0);
    wbch_->addConstraint(Mario2dKneeSingularity_ptr, 2);
//    wbch_->addConstraint(Mario2dCenterOfPressure_ptr, 4);

    wbch_->wbcInit();

    wbch_->displayWbcInformation();
    wbch_->setParametersInt(nWSR_h_);
    wbch_->setParametersDouble(cpuTime_h_);
    // =============================== create hqp_WBC ======================================

   // =============================== create nsp_WBC ======================================

     wbcn_ =new WqpWbc(nV, robot_);

    wbcn_->addTask(Mario2dCentroidalMoment_ptr, 2);
    wbcn_->addTask(Mario2dFloatingBasePose_ptr, 3);
    wbcn_->addTask(Mario2dPointFeetCartPosition_ptr, 1);
    wbcn_->addTask(Mario2dPointFeetCartForce_ptr, 3);
    wbcn_->addTask(Mario2dInternalClosedLoop_ptr, 0);
    // wbcn_->addTask(Mario2dJointTorqueChange_ptr, 0);
    wbcn_->addTask(Mario2dFeetForceChange_ptr, 3);

    wbcn_->addConstraint(Mario2dDynamicConsistency_ptr, 0);
    wbcn_->addConstraint(Mario2dFrictionCone_ptr, 1);
    wbcn_->addConstraint(Mario2dJointTorqueSaturation_ptr, 0);
    wbcn_->addConstraint(Mario2dKneeSingularity_ptr, 2);

    wbcn_->wbcInit();

    wbcn_->displayWbcInformation();
    wbcn_->setParametersInt(nWSR_n_);
    wbcn_->setParametersDouble(cpuTime_n_);
     // =============================== create nsp_WBC ======================================
}

taskControl::~taskControl(){
    delete wbc_;
    delete wbch_;
    delete wbcn_;
}

bool taskControl::setParameters(const robotStateMachine &_rsm){

    Iy_ub = robot_->getIyUpperBody();
    m_total = robot_->getTotalMass();
    m_leg = robot_->getMassLeg();

    if (_rsm.vel_flag == 0){        // control COM
        // the PD of pitch
        kp_pitch = 2*624/Iy_ub;
        kd_pitch = 2*18.2/Iy_ub;
        // the PD of centroidal linear Moment
        stdVec_kp_com = {1/m_total, 50000/m_total}; // x , z
        stdVec_kd_com = {1/m_total, 500/m_total};   // x , z
        // the PD of c_point-feet
        stdVec_kp_cPfDDot_R = {2400/m_leg, 2400/m_leg};
        stdVec_kd_cPfDDot_R = {80/m_leg, 80/m_leg};
        stdVec_kp_cPfDDot_L = {2400/m_leg, 2400/m_leg};
        stdVec_kd_cPfDDot_L = {80/m_leg, 80/m_leg};
    }else{                          // control UpperBody
        // the PD of pitch
        kp_pitch = 1*624/Iy_ub;
        kd_pitch = 1*18.2/Iy_ub;
        // the PD of Cartesian Upper Body
        stdVec_kp_U = {0./m_total, 4500/m_total};
        stdVec_kd_U = {0./m_total, 200/m_total};
        // the PD of c_point-feet
        stdVec_kp_cPfDDot_R = {557, 557};
        stdVec_kd_cPfDDot_R = {6.2, 6.2};
        stdVec_ki_cPfDDot_R = {0., 0.};
        stdVec_kp_cPfDDot_L = {557, 557};
        stdVec_kd_cPfDDot_L = {6.2, 6.2};
        stdVec_ki_cPfDDot_L = {0., 0.};

        // the PD of Cartesian Upper Body
//        stdVec_kp_U = {1/m_total, 300/m_total};
//        stdVec_kd_U = {1/m_total, 120/m_total};
//        // the PD of c_point-feet
//        stdVec_kp_cPfDDot_R = {2400/m_leg, 2400/m_leg};
//        stdVec_kd_cPfDDot_R = {80/m_leg, 80/m_leg};
//        stdVec_kp_cPfDDot_L = {2400/m_leg, 2400/m_leg};
//        stdVec_kd_cPfDDot_L = {80/m_leg, 80/m_leg};
    }

    return true;
}

bool taskControl::walkCtrl(const robotStateMachine & _rsm, const robotState & _rs, const robotDesired & _rd, robotTaskRef & _rtr){
    if (qp_flag_ == 1){
        walkCtrl_hqp(_rsm, _rs, _rd, _rtr);
//        std::cout << "hqp" << std::endl;300
    }
    if (qp_flag_ ==0)
    {
        walkCtrl_wqp(_rsm, _rs, _rd, _rtr);
//        std::cout << "wqp" << std::endl;
    }
    if (qp_flag_ ==2)
    {
        walkCtrl_nsp(_rsm, _rs, _rd, _rtr);
          std::cout << "nsp" << std::endl;
    }
    return true;
}

bool taskControl::walkCtrl_hqp(const robotStateMachine & _rsm, const robotState & _rs, const robotDesired & _rd, robotTaskRef & _rtr){

    // WBC timekeeping calculate
    auto start = std::chrono::system_clock::now();

    // ------------------------------ update Model -------------------------------------
    // RoDy timekeeping calculate
    auto start_rody = std::chrono::system_clock::now();
    wbch_->updateRobotDynamics(_rs.q_, _rs.qDot_);
    // RoDy timekeeping calculate
    auto end_rody = std::chrono::system_clock::now();
    auto duration_rody = std::chrono::duration_cast<std::chrono::nanoseconds>(end_rody - start_rody);
    _rtr.time_RoDy = double(duration_rody.count()) * 1e-6;     // ms


    // TcUpdate timekeeping calculate
    auto start_TcUpdate = std::chrono::system_clock::now();

    // ------------------------------ Calculate Reference ---------------------------------

    if (_rsm.vel_flag == 0){
        // hgDotRef
        _rtr.hgDotRef(0) = 0.0;
        _rtr.hgDotRef.tail(2) = _rs.massTotal*(_rd.cDDot_Com_S_d
                                               + plan::diag(stdVec_kp_com)*(_rd.c_Com_S_d - _rs.c_Com_S)
                                               + plan::diag(stdVec_kd_com)*(_rd.cDot_Com_S_d - _rs.cDot_Com_S));

        // qfDDotRef
        _rtr.qfDDotRef.head(2) = _rd.cDDot_U_S_d
                                + plan::diag(stdVec_kp_U)*(_rd.c_U_S_d - _rs.c_U_S)
                                + plan::diag(stdVec_kd_U)*(_rd.cDot_U_S_d - _rs.cDot_U_S);
        _rtr.qfDDotRef(2) = _rd.pitchDDot_d + kp_pitch*(_rd.pitch_d - _rs.pitch) + kd_pitch*(_rd.pitchDot_d - _rs.pitchDot);
    }else{
        // hgDotRef
        _rtr.hgDotRef(0) = 0.0;
        _rtr.hgDotRef.tail(2) = _rs.massTotal*(_rd.cDDot_Com_S_d
                                               + plan::diag(stdVec_kp_com)*(_rd.c_Com_S_d - _rs.c_Com_S)
                                               + plan::diag(stdVec_kd_com)*(_rd.cDot_Com_S_d - _rs.cDot_Com_S));

        // qfDDotRef
        _rtr.qfDDotRef.head(2) = _rd.cDDot_U_S_d
                                + plan::diag(stdVec_kp_U)*(_rd.c_U_S_d - _rs.c_U_S)
                                + plan::diag(stdVec_kd_U)*(_rd.cDot_U_S_d - _rs.cDot_U_S);
        _rtr.qfDDotRef(2) = _rd.pitchDDot_d + kp_pitch*(_rd.pitch_d - _rs.pitch) + kd_pitch*(_rd.pitchDot_d - _rs.pitchDot);
    }

    // cPfDDotRef
    if ((_rsm.s >= 0 - EPSILON && _rsm.s <= 0 + EPSILON) || (_rsm.remark_flag == 1)){
        integral_cPfError_R = Eigen::Vector2d::Zero();
        integral_cPfError_L = Eigen::Vector2d::Zero();
    }
    integral_cPfError_R += plan::diag(stdVec_ki_cPfDDot_R)*(_rd.c_R_S_d - _rs.c_R_S);
    integral_cPfError_L += plan::diag(stdVec_ki_cPfDDot_L)*(_rd.c_L_S_d - _rs.c_L_S);

    _rtr.cPfDDotRef.head(2) = _rd.cDDot_R_S_d
                                + plan::diag(stdVec_kp_cPfDDot_R)*(_rd.c_R_S_d - _rs.c_R_S)
                                + plan::diag(stdVec_kd_cPfDDot_R)*(_rd.cDot_R_S_d - _rs.cDot_R_S)
                                + integral_cPfError_R;
    _rtr.cPfDDotRef.tail(2) = _rd.cDDot_L_S_d
                                + plan::diag(stdVec_kp_cPfDDot_L)*(_rd.c_L_S_d - _rs.c_L_S)
                                + plan::diag(stdVec_kd_cPfDDot_L)*(_rd.cDot_L_S_d - _rs.cDot_L_S)
                                + integral_cPfError_L;

    // forcePfRef
    if (_rsm.forcePfRef_flag == 0){
        _rtr.forcePfRef = Eigen::Vector4d::Zero();
    }else{
        _rtr.forcePfRef << _rd.fc_grf_R_S_ff,
                            _rd.fc_grf_L_S_ff;
    }

    // torOptPreRef
    _rtr.torOptPreRef = _rtr.tauA_opt;

    // fcOptPreRef
    _rtr.fcOptPreRef = _rtr.fcPf_opt;

    // Weights
    if (_rsm.vel_flag == 0){
        weightVec_CentroidalMoment << 0., 0., 1.;   // omega_y, x, z
        weightVec_FloatingBasePose << 0., 0., 1.;   // x, z, theta
    }else{
        weightVec_CentroidalMoment << 0., 0., 0.;   // omega_y, x, z
        weightVec_FloatingBasePose << 0., 1., 1.;   // x, z, theta
    }

    weightVec_PointFeetCartPosition << 1., 1., 1., 1.;

    weightVec_InternalClosedLoop << 1e5, 1e5, 1e5, 1e5;

    weightVec_ForceChange << 2e-4, 2e-4, 2e-4, 2e-4; // 2e-4


    if (_rsm.state == 1){
        if(_rsm.stanceLeg == 1){
            weightVec_PointFeetCartForce << weight_fx_sw, weight_fz_sw, weight_fx_st, weight_fz_st;
        }else{
            weightVec_PointFeetCartForce << weight_fx_st, weight_fz_st, weight_fx_sw, weight_fz_sw;
        }
    }else{
        if(_rsm.stanceLeg == 1){
            // swing
//            weightVec_PointFeetCartForce(0) = weight_fx_st + (weight_fx_sw - weight_fx_st)*_rsm.ss;
//            weightVec_PointFeetCartForce(1) = weight_fz_st + (weight_fz_sw - weight_fz_st)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, weight_fx_st, weight_fx_sw, _rsm.tt, 1., weightVec_PointFeetCartForce(0));
            plan::line_s(0., scale_ss*_rsm.Td, weight_fz_st, weight_fz_sw, _rsm.tt, 1., weightVec_PointFeetCartForce(1));
            // stance
//            weightVec_PointFeetCartForce(2) = weight_fx_sw + (weight_fx_st - weight_fx_sw)*_rsm.ss;
//            weightVec_PointFeetCartForce(3) = weight_fz_sw + (weight_fz_st - weight_fz_sw)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, weight_fx_sw, weight_fx_st, _rsm.tt, 1., weightVec_PointFeetCartForce(2));
            plan::line_s(0., scale_ss*_rsm.Td, weight_fz_sw, weight_fz_st, _rsm.tt, 1., weightVec_PointFeetCartForce(3));
        }else{
            // swing
//            weightVec_PointFeetCartForce(2) = weight_fx_st + (weight_fx_sw - weight_fx_st)*_rsm.ss;
//            weightVec_PointFeetCartForce(3) = weight_fz_st + (weight_fz_sw - weight_fz_st)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, weight_fx_st, weight_fx_sw, _rsm.tt, 1., weightVec_PointFeetCartForce(2));
            plan::line_s(0., scale_ss*_rsm.Td, weight_fz_st, weight_fz_sw, _rsm.tt, 1., weightVec_PointFeetCartForce(3));
            // stance
//            weightVec_PointFeetCartForce(0) = weight_fx_sw + (weight_fx_st - weight_fx_sw)*_rsm.ss;
//            weightVec_PointFeetCartForce(1) = weight_fz_sw + (weight_fz_st - weight_fz_sw)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, weight_fx_sw, weight_fx_st, _rsm.tt, 1., weightVec_PointFeetCartForce(0));
            plan::line_s(0., scale_ss*_rsm.Td, weight_fz_sw, weight_fz_st, _rsm.tt, 1., weightVec_PointFeetCartForce(1));
        }
    }


    // ------------------------------ Costs/Objects ------------------------------------
    wbch_->updateTask("Mario2dCentroidalMoment", _rtr.hgDotRef, weightVec_CentroidalMoment);
    wbch_->updateTask("Mario2dFloatingBasePose", _rtr.qfDDotRef, weightVec_FloatingBasePose);
    wbch_->updateTask("Mario2dPointFeetCartPosition", _rtr.cPfDDotRef, weightVec_PointFeetCartPosition);
    wbch_->updateTask("Mario2dPointFeetCartForce", _rtr.forcePfRef, weightVec_PointFeetCartForce);
    wbch_->updateTask("Mario2dInternalClosedLoop", _rtr.loopRef, weightVec_InternalClosedLoop);
    wbch_->updateTask("Mario2dFeetForceChange", _rtr.fcOptPreRef, weightVec_ForceChange);

    // --------------------------------- Constraints -------------------------------------
    wbch_->updateConstraint("Mario2dDynamicConsistency");
    wbch_->updateConstraint("Mario2dFrictionCone");
    wbch_->updateConstraint("Mario2dJointTorqueSaturation");
    wbch_->updateConstraint("Mario2dKneeSingularity");

    // -------------------------------- Bounds -----------------------------------------
    // Bounds foot external forces
    if (_rsm.state == 1){
        if(_rsm.stanceLeg == 1){
            fcLowerBounds << 0., 0., -2*24*GRAVITY, 0.;
            fcUpperBounds << 0., 0., 2*24*GRAVITY, 2*24*GRAVITY;
        }else{
            fcLowerBounds << -2*24*GRAVITY, 0., 0., 0.;
            fcUpperBounds << 2*24*GRAVITY, 2*24*GRAVITY, 0., 0.;
        }
    }else{
        if(_rsm.stanceLeg == 1){
            // swing
            fcLowerBounds(1) = 0.;
//            fcUpperBounds(1) = 1.5*24*GRAVITY + (0. - 1.5*24*GRAVITY)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, 1.5*24*GRAVITY, 0., _rsm.tt, 1., fcUpperBounds(1));
            fcLowerBounds(0) = -fcUpperBounds(1);
            fcUpperBounds(0) = fcUpperBounds(1);
            // stance
            fcLowerBounds(3) = 0.;
//            fcUpperBounds(3) = 0. + (1.5*24*GRAVITY - 0.)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, 0., 1.5*24*GRAVITY, _rsm.tt, 1., fcUpperBounds(3));
            fcLowerBounds(2) = -fcUpperBounds(3);
            fcUpperBounds(2) = fcUpperBounds(3);
        }else{
            // swing
            fcLowerBounds(3) = 0.;
//            fcUpperBounds(3) = 1.5*24*GRAVITY + (0. - 1.5*24*GRAVITY)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, 1.5*24*GRAVITY, 0., _rsm.tt, 1., fcUpperBounds(3));
            fcLowerBounds(2) = -fcUpperBounds(3);
            fcUpperBounds(2) = fcUpperBounds(3);
            // stance
            fcLowerBounds(1) = 0.;
//            fcUpperBounds(1) = 0. + (1.5*24*GRAVITY - 0.)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, 0., 1.5*24*GRAVITY, _rsm.tt, 1., fcUpperBounds(1));
            fcLowerBounds(0) = -fcUpperBounds(1);
            fcUpperBounds(0) = fcUpperBounds(1);
        }
    }

    boundsVec_lb.head(nJg) = -jointQddot_limit_ * Eigen::VectorXd::Ones(nJg);
    boundsVec_ub.head(nJg) = jointQddot_limit_ * Eigen::VectorXd::Ones(nJg);
    boundsVec_lb.tail(nFc) = fcLowerBounds;
    boundsVec_ub.tail(nFc) = fcUpperBounds;

    wbch_->updateBound(boundsVec_lb, boundsVec_ub);

    // TcUpdate timekeeping calculate
    auto end_TcUpdate = std::chrono::system_clock::now();
    auto duration_TcUpdate = std::chrono::duration_cast<std::chrono::nanoseconds>(end_TcUpdate - start_TcUpdate);
    _rtr.time_TcUpdate = double(duration_TcUpdate.count()) * 1e-6;     // ms

    // ================================= WBC solve =========================================

    // QP timekeeping calculate
    auto start_QP = std::chrono::system_clock::now();
    wbch_->wbcSolve();
    // QP timekeeping calculate
    auto end_QP = std::chrono::system_clock::now();
    auto duration_QP = std::chrono::duration_cast<std::chrono::nanoseconds>(end_QP - start_QP);
    _rtr.time_QP = double(duration_QP.count()) * 1e-6;     // ms

    std::copy(dataOptInt.begin(), dataOptInt.begin() + 4, iterOpt.begin());
    std::copy(dataOptInt.begin() + 4 , dataOptInt.begin() + 8, successOpt.begin());

    std::copy(dataOptDouble.begin(), dataOptDouble.begin() + 4, costOpt.begin());
    std::copy(dataOptDouble.begin() + 4 , dataOptDouble.begin() + 8, timeOpt.begin());

//    for (int i = 0; i != 4; i++) {
//        H_cost_opt(i) = costOpt.at(i);
//        H_iter_opt(i) = iterOpt.at(i);
//        H_time_opt(i) = timeOpt.at(i);
//        if (successOpt.at(i) == 0) {
//            H_success_opt[i] = true;
//        } else {
//            H_success_opt[i] = false;
//        }
//    }

    if (_rtr.simpleStatus == 0){
        getOptResult(_rtr.qDDot_opt, _rtr.fcPf_opt, _rtr.tauA_opt);
    }else{
        std::cout << "QP failed at " << _rsm.timeCs << " (s)." << std::endl;
    }
    _rtr.tauA_cmd = _rtr.tauA_opt;

    // optimization result analysis
    _rtr.hgDot_opt = robot_->AG * _rtr.qDDot_opt + robot_->AGdotQdot;
    _rtr.qfDDot_opt = robot_->Sf * _rtr.qDDot_opt;
    _rtr.cPfDDot_opt = robot_->J_c * _rtr.qDDot_opt + robot_->JdotQdot_c;
    testClosedLoopConsistent(_rtr.qDDot_opt, _rtr.xRzRxLzL_ddot_ClosedLoop);

    // WBC timekeeping calculate
    auto end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    _rtr.time_WBC = double(duration.count()) * 1e-6;     // ms

    // integral of qDDot_opt
    if (_rsm.actuatorMode_flag == 0){
        stdVec_k_sw_qDot_integral = {0., 0.};
    }else{
        stdVec_k_sw_qDot_integral = {0., 0.};
    }
    if (_rsm.state == 1){
        if(_rsm.stanceLeg == 1){
            if(_rsm.s >= 0 - EPSILON && _rsm.s <= 0 + EPSILON){   // 'LO'
                _rtr.qDot_A_sw_integral = _rs.qDot_R_A;
            }
            _rtr.qDot_A_sw_integral(0) += _rtr.qDDot_opt(3)*DT;
            _rtr.qDot_A_sw_integral(1) += _rtr.qDDot_opt(5)*DT;
            _rtr.tauA_cmd.head(2) = _rtr.tauA_opt.head(2) + plan::diag(stdVec_k_sw_qDot_integral)*(_rtr.qDot_A_sw_integral - _rs.qDot_R_A);
        }else{
            if(_rsm.s >= 0 - EPSILON && _rsm.s <= 0 + EPSILON){   // 'LO'
                _rtr.qDot_A_sw_integral = _rs.qDot_L_A;
            }
            _rtr.qDot_A_sw_integral(0) += _rtr.qDDot_opt(7)*DT;
            _rtr.qDot_A_sw_integral(1) += _rtr.qDDot_opt(9)*DT;
            _rtr.tauA_cmd.tail(2) = _rtr.tauA_opt.tail(2) + plan::diag(stdVec_k_sw_qDot_integral)*(_rtr.qDot_A_sw_integral - _rs.qDot_L_A);
        }
    }else {
        _rtr.tauA_cmd = _rtr.tauA_opt;
    }

    // calculate output
    calcJointDes(_rsm, _rs, _rd, _rtr);
    calcPVTPID(_rs, _rd, _rtr);

    // reset
    robot_->resetIsPosVelUpdated();

    return true;
}


bool taskControl::walkCtrl_wqp(const robotStateMachine & _rsm, const robotState & _rs, const robotDesired & _rd, robotTaskRef & _rtr){

    // WBC timekeeping calculate
    auto start = std::chrono::system_clock::now();

    // ------------------------------ update Model -------------------------------------
    // RoDy timekeeping calculate
    auto start_rody = std::chrono::system_clock::now();
    wbc_->updateRobotDynamics(_rs.q_, _rs.qDot_);
    // RoDy timekeeping calculate
    auto end_rody = std::chrono::system_clock::now();
    auto duration_rody = std::chrono::duration_cast<std::chrono::nanoseconds>(end_rody - start_rody);
    _rtr.time_RoDy = double(duration_rody.count()) * 1e-6;     // ms


    // TcUpdate timekeeping calculate
    auto start_TcUpdate = std::chrono::system_clock::now();

    // ------------------------------ Calculate Reference ---------------------------------

    if (_rsm.vel_flag == 0){
        // hgDotRef
        _rtr.hgDotRef(0) = 0.0;
        _rtr.hgDotRef.tail(2) = _rs.massTotal*(_rd.cDDot_Com_S_d
                                               + plan::diag(stdVec_kp_com)*(_rd.c_Com_S_d - _rs.c_Com_S)
                                               + plan::diag(stdVec_kd_com)*(_rd.cDot_Com_S_d - _rs.cDot_Com_S));

        // qfDDotRef
        _rtr.qfDDotRef.head(2) = Eigen::Vector2d::Zero();
        _rtr.qfDDotRef(2) = _rd.pitchDDot_d + kp_pitch*(_rd.pitch_d - _rs.pitch) + kd_pitch*(_rd.pitchDot_d - _rs.pitchDot);
    }else{
        // hgDotRef
        _rtr.hgDotRef = Eigen::Vector3d::Zero();

        // qfDDotRef
        _rtr.qfDDotRef.head(2) = _rd.cDDot_U_S_d
                                + plan::diag(stdVec_kp_U)*(_rd.c_U_S_d - _rs.c_U_S)
                                + plan::diag(stdVec_kd_U)*(_rd.cDot_U_S_d - _rs.cDot_U_S);
        _rtr.qfDDotRef(2) = _rd.pitchDDot_d + kp_pitch*(_rd.pitch_d - _rs.pitch) + kd_pitch*(_rd.pitchDot_d - _rs.pitchDot);
    }

    // cPfDDotRef
    if ((_rsm.s >= 0 - EPSILON && _rsm.s <= 0 + EPSILON) || (_rsm.remark_flag == 1)){
        integral_cPfError_R = Eigen::Vector2d::Zero();
        integral_cPfError_L = Eigen::Vector2d::Zero();
    }
    integral_cPfError_R += plan::diag(stdVec_ki_cPfDDot_R)*(_rd.c_R_S_d - _rs.c_R_S);
    integral_cPfError_L += plan::diag(stdVec_ki_cPfDDot_L)*(_rd.c_L_S_d - _rs.c_L_S);

    _rtr.cPfDDotRef.head(2) = _rd.cDDot_R_S_d
                                + plan::diag(stdVec_kp_cPfDDot_R)*(_rd.c_R_S_d - _rs.c_R_S)
                                + plan::diag(stdVec_kd_cPfDDot_R)*(_rd.cDot_R_S_d - _rs.cDot_R_S)
                                + integral_cPfError_R;
    _rtr.cPfDDotRef.tail(2) = _rd.cDDot_L_S_d
                                + plan::diag(stdVec_kp_cPfDDot_L)*(_rd.c_L_S_d - _rs.c_L_S)
                                + plan::diag(stdVec_kd_cPfDDot_L)*(_rd.cDot_L_S_d - _rs.cDot_L_S)
                                + integral_cPfError_L;

    // forcePfRef
    if (_rsm.forcePfRef_flag == 0){
        _rtr.forcePfRef = Eigen::Vector4d::Zero();
    }else{
        _rtr.forcePfRef << _rd.fc_grf_R_S_ff,
                            _rd.fc_grf_L_S_ff;
    }

    // torOptPreRef
    _rtr.torOptPreRef = _rtr.tauA_opt;

    // fcOptPreRef
    _rtr.fcOptPreRef = _rtr.fcPf_opt;

    // Weights
    if (_rsm.vel_flag == 0){
        weightVec_CentroidalMoment << 0., 0., 1.;
        weightVec_FloatingBasePose << 0., 0., 1.;
    }else{
        weightVec_CentroidalMoment << 0., 0., 0.;
        weightVec_FloatingBasePose << 0., 1., 1.;
    }

    weightVec_PointFeetCartPosition << 10., 10., 10., 10.;

    if (_rsm.state == 1){
        if(_rsm.stanceLeg == 1){
            weightVec_PointFeetCartForce << weight_fx_sw, weight_fz_sw, weight_fx_st, weight_fz_st;
        }else{
            weightVec_PointFeetCartForce << weight_fx_st, weight_fz_st, weight_fx_sw, weight_fz_sw;
        }
    }else{
        if(_rsm.stanceLeg == 1){
            // swing
//            weightVec_PointFeetCartForce(0) = weight_fx_st + (weight_fx_sw - weight_fx_st)*_rsm.ss;
//            weightVec_PointFeetCartForce(1) = weight_fz_st + (weight_fz_sw - weight_fz_st)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, weight_fx_st, weight_fx_sw, _rsm.tt, 1., weightVec_PointFeetCartForce(0));
            plan::line_s(0., scale_ss*_rsm.Td, weight_fz_st, weight_fz_sw, _rsm.tt, 1., weightVec_PointFeetCartForce(1));
            // stance
//            weightVec_PointFeetCartForce(2) = weight_fx_sw + (weight_fx_st - weight_fx_sw)*_rsm.ss;
//            weightVec_PointFeetCartForce(3) = weight_fz_sw + (weight_fz_st - weight_fz_sw)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, weight_fx_sw, weight_fx_st, _rsm.tt, 1., weightVec_PointFeetCartForce(2));
            plan::line_s(0., scale_ss*_rsm.Td, weight_fz_sw, weight_fz_st, _rsm.tt, 1., weightVec_PointFeetCartForce(3));
        }else{
            // swing
//            weightVec_PointFeetCartForce(2) = weight_fx_st + (weight_fx_sw - weight_fx_st)*_rsm.ss;
//            weightVec_PointFeetCartForce(3) = weight_fz_st + (weight_fz_sw - weight_fz_st)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, weight_fx_st, weight_fx_sw, _rsm.tt, 1., weightVec_PointFeetCartForce(2));
            plan::line_s(0., scale_ss*_rsm.Td, weight_fz_st, weight_fz_sw, _rsm.tt, 1., weightVec_PointFeetCartForce(3));
            // stance
//            weightVec_PointFeetCartForce(0) = weight_fx_sw + (weight_fx_st - weight_fx_sw)*_rsm.ss;
//            weightVec_PointFeetCartForce(1) = weight_fz_sw + (weight_fz_st - weight_fz_sw)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, weight_fx_sw, weight_fx_st, _rsm.tt, 1., weightVec_PointFeetCartForce(0));
            plan::line_s(0., scale_ss*_rsm.Td, weight_fz_sw, weight_fz_st, _rsm.tt, 1., weightVec_PointFeetCartForce(1));
        }
    }

    weightVec_InternalClosedLoop << 1e5, 1e5, 1e5, 1e5;

    weightVec_TorqueChange << 0., 0., 0., 0.;

    weightVec_ForceChange << 2e-4, 2e-4, 2e-4, 2e-4; // 2e-4

    if (_rsm.tick == 0){
        weightVec_TorqueChange = Eigen::Vector4d::Zero();
        weightVec_ForceChange = Eigen::Vector4d::Zero();
    }

    // ------------------------------ Costs/Objects ------------------------------------
    wbc_->updateTask("Mario2dCentroidalMoment", _rtr.hgDotRef, weightVec_CentroidalMoment);
    wbc_->updateTask("Mario2dFloatingBasePose", _rtr.qfDDotRef, weightVec_FloatingBasePose);
    wbc_->updateTask("Mario2dPointFeetCartPosition", _rtr.cPfDDotRef, weightVec_PointFeetCartPosition);
    wbc_->updateTask("Mario2dPointFeetCartForce", _rtr.forcePfRef, weightVec_PointFeetCartForce);
    wbc_->updateTask("Mario2dInternalClosedLoop", _rtr.loopRef, weightVec_InternalClosedLoop);
    wbc_->updateTask("Mario2dJointTorqueChange", _rtr.torOptPreRef, weightVec_TorqueChange);
    wbc_->updateTask("Mario2dFeetForceChange", _rtr.fcOptPreRef, weightVec_ForceChange);

    // --------------------------------- Constraints -------------------------------------
    wbc_->updateConstraint("Mario2dDynamicConsistency");
    wbc_->updateConstraint("Mario2dFrictionCone");
    wbc_->updateConstraint("Mario2dJointTorqueSaturation");
    wbc_->updateConstraint("Mario2dKneeSingularity");

    // -------------------------------- Bounds -----------------------------------------
    // Bounds foot external forces
    if (_rsm.state == 1){
        if(_rsm.stanceLeg == 1){
            fcLowerBounds << 0., 0., -2*24*GRAVITY, 0.;
            fcUpperBounds << 0., 0., 2*24*GRAVITY, 2*24*GRAVITY;
        }else{
            fcLowerBounds << -2*24*GRAVITY, 0., 0., 0.;
            fcUpperBounds << 2*24*GRAVITY, 2*24*GRAVITY, 0., 0.;
        }
    }else{
        if(_rsm.stanceLeg == 1){
            // swing
            fcLowerBounds(1) = 0.;
//            fcUpperBounds(1) = 1.5*24*GRAVITY + (0. - 1.5*24*GRAVITY)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, 1.5*24*GRAVITY, 0., _rsm.tt, 1., fcUpperBounds(1));
            fcLowerBounds(0) = -fcUpperBounds(1);
            fcUpperBounds(0) = fcUpperBounds(1);
            // stance
            fcLowerBounds(3) = 0.;
//            fcUpperBounds(3) = 0. + (1.5*24*GRAVITY - 0.)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, 0., 1.5*24*GRAVITY, _rsm.tt, 1., fcUpperBounds(3));
            fcLowerBounds(2) = -fcUpperBounds(3);
            fcUpperBounds(2) = fcUpperBounds(3);
        }else{
            // swing
            fcLowerBounds(3) = 0.;
//            fcUpperBounds(3) = 1.5*24*GRAVITY + (0. - 1.5*24*GRAVITY)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, 1.5*24*GRAVITY, 0., _rsm.tt, 1., fcUpperBounds(3));
            fcLowerBounds(2) = -fcUpperBounds(3);
            fcUpperBounds(2) = fcUpperBounds(3);
            // stance
            fcLowerBounds(1) = 0.;
//            fcUpperBounds(1) = 0. + (1.5*24*GRAVITY - 0.)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, 0., 1.5*24*GRAVITY, _rsm.tt, 1., fcUpperBounds(1));
            fcLowerBounds(0) = -fcUpperBounds(1);
            fcUpperBounds(0) = fcUpperBounds(1);
        }
    }

    boundsVec_lb.head(nJg) = -jointQddot_limit_ * Eigen::VectorXd::Ones(nJg);
    boundsVec_ub.head(nJg) = jointQddot_limit_ * Eigen::VectorXd::Ones(nJg);
    boundsVec_lb.tail(nFc) = fcLowerBounds;
    boundsVec_ub.tail(nFc) = fcUpperBounds;

    wbc_->updateBound(boundsVec_lb, boundsVec_ub);

    // TcUpdate timekeeping calculate
    auto end_TcUpdate = std::chrono::system_clock::now();
    auto duration_TcUpdate = std::chrono::duration_cast<std::chrono::nanoseconds>(end_TcUpdate - start_TcUpdate);
    _rtr.time_TcUpdate = double(duration_TcUpdate.count()) * 1e-6;     // ms

    // ================================= WBC solve =========================================

    // QP timekeeping calculate
    auto start_QP = std::chrono::system_clock::now();
    wbc_->wbcSolve();
    // QP timekeeping calculate
    auto end_QP = std::chrono::system_clock::now();
    auto duration_QP = std::chrono::duration_cast<std::chrono::nanoseconds>(end_QP - start_QP);
    _rtr.time_QP = double(duration_QP.count()) * 1e-6;     // ms

    wbc_->getAuxiliaryDataInt(intData);
    _rtr.nWSR_res = intData.at(0);
    _rtr.simpleStatus = intData.at(1);
    wbc_->getAuxiliaryDataDouble(doubleData);
    _rtr.cost_opt = doubleData.at(0);
    _rtr.cputime_res = doubleData.at(1);

    if (_rtr.simpleStatus == 0){
        getOptResult(_rtr.qDDot_opt, _rtr.fcPf_opt, _rtr.tauA_opt);
    }else{
        std::cout << "QP failed at " << _rsm.timeCs << " (s)." << std::endl;
    }
    _rtr.tauA_cmd = _rtr.tauA_opt;

    // optimization result analysis
    _rtr.hgDot_opt = robot_->AG * _rtr.qDDot_opt + robot_->AGdotQdot;
    _rtr.qfDDot_opt = robot_->Sf * _rtr.qDDot_opt;
    _rtr.cPfDDot_opt = robot_->J_c * _rtr.qDDot_opt + robot_->JdotQdot_c;
    testClosedLoopConsistent(_rtr.qDDot_opt, _rtr.xRzRxLzL_ddot_ClosedLoop);

    // WBC timekeeping calculate
    auto end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    _rtr.time_WBC = double(duration.count()) * 1e-6;     // ms

    // integral of qDDot_opt
    if (_rsm.actuatorMode_flag == 0){
        stdVec_k_sw_qDot_integral = {0., 0.};
    }else{
        stdVec_k_sw_qDot_integral = {0., 0.};
    }
    if (_rsm.state == 1){
        if(_rsm.stanceLeg == 1){
            if(_rsm.s >= 0 - EPSILON && _rsm.s <= 0 + EPSILON){   // 'LO'
                _rtr.qDot_A_sw_integral = _rs.qDot_R_A;
            }
            _rtr.qDot_A_sw_integral(0) += _rtr.qDDot_opt(3)*DT;
            _rtr.qDot_A_sw_integral(1) += _rtr.qDDot_opt(5)*DT;
            _rtr.tauA_cmd.head(2) = _rtr.tauA_opt.head(2) + plan::diag(stdVec_k_sw_qDot_integral)*(_rtr.qDot_A_sw_integral - _rs.qDot_R_A);
        }else{
            if(_rsm.s >= 0 - EPSILON && _rsm.s <= 0 + EPSILON){   // 'LO'
                _rtr.qDot_A_sw_integral = _rs.qDot_L_A;
            }
            _rtr.qDot_A_sw_integral(0) += _rtr.qDDot_opt(7)*DT;
            _rtr.qDot_A_sw_integral(1) += _rtr.qDDot_opt(9)*DT;
            _rtr.tauA_cmd.tail(2) = _rtr.tauA_opt.tail(2) + plan::diag(stdVec_k_sw_qDot_integral)*(_rtr.qDot_A_sw_integral - _rs.qDot_L_A);
        }
    }else {
        _rtr.tauA_cmd = _rtr.tauA_opt;
    }

    // calculate output
    calcJointDes(_rsm, _rs, _rd, _rtr);
    calcPVTPID(_rs, _rd, _rtr);

    // reset
    robot_->resetIsPosVelUpdated();

    return true;
}
bool taskControl::walkCtrl_nsp(const robotStateMachine & _rsm, const robotState & _rs, const robotDesired & _rd, robotTaskRef & _rtr){
    // WBC timekeeping calculate
    auto start = std::chrono::system_clock::now();
    
    // ------------------------------ update Model -------------------------------------
    // RoDy timekeeping calculate
    auto start_rody = std::chrono::system_clock::now();
    wbcn_->updateRobotDynamics(_rs.q_, _rs.qDot_);
    auto end_rody = std::chrono::system_clock::now();
    auto duration_rody = std::chrono::duration_cast<std::chrono::nanoseconds>(end_rody - start_rody);
    _rtr.time_RoDy = double(duration_rody.count()) * 1e-6;     // ms

    // TcUpdate timekeeping calculate
    auto start_TcUpdate = std::chrono::system_clock::now();

     // ------------------------------ Calculate Reference --------------------------------- there are diffirences between wqp and hqp

    if (_rsm.vel_flag == 0){
        // hgDotRef
        _rtr.hgDotRef(0) = 0.0;
        _rtr.hgDotRef.tail(2) = _rs.massTotal*(_rd.cDDot_Com_S_d
                                               + plan::diag(stdVec_kp_com)*(_rd.c_Com_S_d - _rs.c_Com_S)
                                               + plan::diag(stdVec_kd_com)*(_rd.cDot_Com_S_d - _rs.cDot_Com_S));

        // qfDDotRef
        _rtr.qfDDotRef.head(2) = _rd.cDDot_U_S_d
                                + plan::diag(stdVec_kp_U)*(_rd.c_U_S_d - _rs.c_U_S)
                                + plan::diag(stdVec_kd_U)*(_rd.cDot_U_S_d - _rs.cDot_U_S);
        _rtr.qfDDotRef(2) = _rd.pitchDDot_d + kp_pitch*(_rd.pitch_d - _rs.pitch) + kd_pitch*(_rd.pitchDot_d - _rs.pitchDot);
    }else{
        // hgDotRef
        _rtr.hgDotRef(0) = 0.0;
        _rtr.hgDotRef.tail(2) = _rs.massTotal*(_rd.cDDot_Com_S_d
                                               + plan::diag(stdVec_kp_com)*(_rd.c_Com_S_d - _rs.c_Com_S)
                                               + plan::diag(stdVec_kd_com)*(_rd.cDot_Com_S_d - _rs.cDot_Com_S));

        // qfDDotRef
        _rtr.qfDDotRef.head(2) = _rd.cDDot_U_S_d
                                + plan::diag(stdVec_kp_U)*(_rd.c_U_S_d - _rs.c_U_S)
                                + plan::diag(stdVec_kd_U)*(_rd.cDot_U_S_d - _rs.cDot_U_S);
        _rtr.qfDDotRef(2) = _rd.pitchDDot_d + kp_pitch*(_rd.pitch_d - _rs.pitch) + kd_pitch*(_rd.pitchDot_d - _rs.pitchDot);
    }
    // cPfDDotRef
    if ((_rsm.s >= 0 - EPSILON && _rsm.s <= 0 + EPSILON) || (_rsm.remark_flag == 1)){
        integral_cPfError_R = Eigen::Vector2d::Zero();
        integral_cPfError_L = Eigen::Vector2d::Zero();
    }
    integral_cPfError_R += plan::diag(stdVec_ki_cPfDDot_R)*(_rd.c_R_S_d - _rs.c_R_S);
    integral_cPfError_L += plan::diag(stdVec_ki_cPfDDot_L)*(_rd.c_L_S_d - _rs.c_L_S);

    _rtr.cPfDDotRef.head(2) = _rd.cDDot_R_S_d
                                + plan::diag(stdVec_kp_cPfDDot_R)*(_rd.c_R_S_d - _rs.c_R_S)
                                + plan::diag(stdVec_kd_cPfDDot_R)*(_rd.cDot_R_S_d - _rs.cDot_R_S)
                                + integral_cPfError_R;
    _rtr.cPfDDotRef.tail(2) = _rd.cDDot_L_S_d
                                + plan::diag(stdVec_kp_cPfDDot_L)*(_rd.c_L_S_d - _rs.c_L_S)
                                + plan::diag(stdVec_kd_cPfDDot_L)*(_rd.cDot_L_S_d - _rs.cDot_L_S)
                                + integral_cPfError_L;

    // forcePfRef
    if (_rsm.forcePfRef_flag == 0){
        _rtr.forcePfRef = Eigen::Vector4d::Zero();
    }else{
        _rtr.forcePfRef << _rd.fc_grf_R_S_ff,
                            _rd.fc_grf_L_S_ff;
    }

    // torOptPreRef
    _rtr.torOptPreRef = _rtr.tauA_opt;

    // fcOptPreRef
    _rtr.fcOptPreRef = _rtr.fcPf_opt;

        // Weights
        //it's essential to set the correct weights for each task and constraint
         if (_rsm.vel_flag == 0){
        weightVec_CentroidalMoment << 0., 0., 1.;   // omega_y, x, z
        weightVec_FloatingBasePose << 0., 0., 1.;   // x, z, theta
    }else{
        weightVec_CentroidalMoment << 0., 0., 0.;   // omega_y, x, z
        weightVec_FloatingBasePose << 0., 1., 1.;   // x, z, theta
    }

    weightVec_PointFeetCartPosition << 1., 1., 1., 1.;

    weightVec_InternalClosedLoop << 1e5, 1e5, 1e5, 1e5;

    weightVec_ForceChange << 2e-4, 2e-4, 2e-4, 2e-4; // 2e-4
        // Weights

        if (_rsm.state == 1){
        if(_rsm.stanceLeg == 1){
            weightVec_PointFeetCartForce << weight_fx_sw, weight_fz_sw, weight_fx_st, weight_fz_st;
        }else{
            weightVec_PointFeetCartForce << weight_fx_st, weight_fz_st, weight_fx_sw, weight_fz_sw;
        }
    }else{
        if(_rsm.stanceLeg == 1){
            // swing
//            weightVec_PointFeetCartForce(0) = weight_fx_st + (weight_fx_sw - weight_fx_st)*_rsm.ss;
//            weightVec_PointFeetCartForce(1) = weight_fz_st + (weight_fz_sw - weight_fz_st)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, weight_fx_st, weight_fx_sw, _rsm.tt, 1., weightVec_PointFeetCartForce(0));
            plan::line_s(0., scale_ss*_rsm.Td, weight_fz_st, weight_fz_sw, _rsm.tt, 1., weightVec_PointFeetCartForce(1));
            // stance
//            weightVec_PointFeetCartForce(2) = weight_fx_sw + (weight_fx_st - weight_fx_sw)*_rsm.ss;
//            weightVec_PointFeetCartForce(3) = weight_fz_sw + (weight_fz_st - weight_fz_sw)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, weight_fx_sw, weight_fx_st, _rsm.tt, 1., weightVec_PointFeetCartForce(2));
            plan::line_s(0., scale_ss*_rsm.Td, weight_fz_sw, weight_fz_st, _rsm.tt, 1., weightVec_PointFeetCartForce(3));
        }else{
            // swing
//            weightVec_PointFeetCartForce(2) = weight_fx_st + (weight_fx_sw - weight_fx_st)*_rsm.ss;
//            weightVec_PointFeetCartForce(3) = weight_fz_st + (weight_fz_sw - weight_fz_st)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, weight_fx_st, weight_fx_sw, _rsm.tt, 1., weightVec_PointFeetCartForce(2));
            plan::line_s(0., scale_ss*_rsm.Td, weight_fz_st, weight_fz_sw, _rsm.tt, 1., weightVec_PointFeetCartForce(3));
            // stance
//            weightVec_PointFeetCartForce(0) = weight_fx_sw + (weight_fx_st - weight_fx_sw)*_rsm.ss;
//            weightVec_PointFeetCartForce(1) = weight_fz_sw + (weight_fz_st - weight_fz_sw)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, weight_fx_sw, weight_fx_st, _rsm.tt, 1., weightVec_PointFeetCartForce(0));
            plan::line_s(0., scale_ss*_rsm.Td, weight_fz_sw, weight_fz_st, _rsm.tt, 1., weightVec_PointFeetCartForce(1));
        }
    }

    // ------------------------------ Costs/Objects ------------------------------------
    wbcn_->updateTask("Mario2dCentroidalMoment", _rtr.hgDotRef, weightVec_CentroidalMoment);
    wbcn_->updateTask("Mario2dFloatingBasePose", _rtr.qfDDotRef, weightVec_FloatingBasePose);
    wbcn_->updateTask("Mario2dPointFeetCartPosition", _rtr.cPfDDotRef, weightVec_PointFeetCartPosition);
    wbcn_->updateTask("Mario2dPointFeetCartForce", _rtr.forcePfRef, weightVec_PointFeetCartForce);
    wbcn_->updateTask("Mario2dInternalClosedLoop", _rtr.loopRef, weightVec_InternalClosedLoop);
    wbcn_->updateTask("Mario2dFeetForceChange", _rtr.fcOptPreRef, weightVec_ForceChange);

    // --------------------------------- Constraints -------------------------------------
    wbcn_->updateConstraint("Mario2dDynamicConsistency");
    wbcn_->updateConstraint("Mario2dFrictionCone");
    wbcn_->updateConstraint("Mario2dJointTorqueSaturation");
    wbcn_->updateConstraint("Mario2dKneeSingularity");

    // -------------------------------- Bounds -----------------------------------------
    // Bounds foot external forces
    if (_rsm.state == 1){
        if(_rsm.stanceLeg == 1){
            fcLowerBounds << 0., 0., -2*24*GRAVITY, 0.;
            fcUpperBounds << 0., 0., 2*24*GRAVITY, 2*24*GRAVITY;
        }else{
            fcLowerBounds << -2*24*GRAVITY, 0., 0., 0.;
            fcUpperBounds << 2*24*GRAVITY, 2*24*GRAVITY, 0., 0.;
        }
    }else{
        if(_rsm.stanceLeg == 1){
            // swing
            fcLowerBounds(1) = 0.;
//            fcUpperBounds(1) = 1.5*24*GRAVITY + (0. - 1.5*24*GRAVITY)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, 1.5*24*GRAVITY, 0., _rsm.tt, 1., fcUpperBounds(1));
            fcLowerBounds(0) = -fcUpperBounds(1);
            fcUpperBounds(0) = fcUpperBounds(1);
            // stance
            fcLowerBounds(3) = 0.;
//            fcUpperBounds(3) = 0. + (1.5*24*GRAVITY - 0.)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, 0., 1.5*24*GRAVITY, _rsm.tt, 1., fcUpperBounds(3));
            fcLowerBounds(2) = -fcUpperBounds(3);
            fcUpperBounds(2) = fcUpperBounds(3);
        }else{
            // swing
            fcLowerBounds(3) = 0.;
//            fcUpperBounds(3) = 1.5*24*GRAVITY + (0. - 1.5*24*GRAVITY)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, 1.5*24*GRAVITY, 0., _rsm.tt, 1., fcUpperBounds(3));
            fcLowerBounds(2) = -fcUpperBounds(3);
            fcUpperBounds(2) = fcUpperBounds(3);
            // stance
            fcLowerBounds(1) = 0.;
//            fcUpperBounds(1) = 0. + (1.5*24*GRAVITY - 0.)*_rsm.ss;
            plan::line_s(0., scale_ss*_rsm.Td, 0., 1.5*24*GRAVITY, _rsm.tt, 1., fcUpperBounds(1));
            fcLowerBounds(0) = -fcUpperBounds(1);
            fcUpperBounds(0) = fcUpperBounds(1);
        }
    }

    boundsVec_lb.head(nJg) = -jointQddot_limit_ * Eigen::VectorXd::Ones(nJg);
    boundsVec_ub.head(nJg) = jointQddot_limit_ * Eigen::VectorXd::Ones(nJg);
    boundsVec_lb.tail(nFc) = fcLowerBounds;
    boundsVec_ub.tail(nFc) = fcUpperBounds;

    wbcn_->updateBound(boundsVec_lb, boundsVec_ub);

    // TcUpdate timekeeping calculate
    auto end_TcUpdate = std::chrono::system_clock::now();
    auto duration_TcUpdate = std::chrono::duration_cast<std::chrono::nanoseconds>(end_TcUpdate - start_TcUpdate);
    _rtr.time_TcUpdate = double(duration_TcUpdate.count()) * 1e-6;     // ms

    // ================================= WBC solve =========================================

    // QP timekeeping calculate
    auto start_QP = std::chrono::system_clock::now();
    wbcn_->wbcSolve();
    // QP timekeeping calculate
    auto end_QP = std::chrono::system_clock::now();
    auto duration_QP = std::chrono::duration_cast<std::chrono::nanoseconds>(end_QP - start_QP);
    _rtr.time_QP = double(duration_QP.count()) * 1e-6;     // ms

    std::copy(dataOptInt.begin(), dataOptInt.begin() + 4, iterOpt.begin());
    std::copy(dataOptInt.begin() + 4 , dataOptInt.begin() + 8, successOpt.begin());

    std::copy(dataOptDouble.begin(), dataOptDouble.begin() + 4, costOpt.begin());
    std::copy(dataOptDouble.begin() + 4 , dataOptDouble.begin() + 8, timeOpt.begin());

    if (_rtr.simpleStatus == 0){
        getOptResult(_rtr.qDDot_opt, _rtr.fcPf_opt, _rtr.tauA_opt);
    }else{
        std::cout << "QP failed at " << _rsm.timeCs << " (s)." << std::endl;
    }
    _rtr.tauA_cmd = _rtr.tauA_opt;

    // optimization result analysis
    _rtr.hgDot_opt = robot_->AG * _rtr.qDDot_opt + robot_->AGdotQdot;
    _rtr.qfDDot_opt = robot_->Sf * _rtr.qDDot_opt;
    _rtr.cPfDDot_opt = robot_->J_c * _rtr.qDDot_opt + robot_->JdotQdot_c;
    testClosedLoopConsistent(_rtr.qDDot_opt, _rtr.xRzRxLzL_ddot_ClosedLoop);

    // WBC timekeeping calculate
    auto end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    _rtr.time_WBC = double(duration.count()) * 1e-6;     // ms

    // integral of qDDot_opt
    if (_rsm.actuatorMode_flag == 0){
        stdVec_k_sw_qDot_integral = {0., 0.};
    }else{
        stdVec_k_sw_qDot_integral = {0., 0.};
    }
    if (_rsm.state == 1){
        if(_rsm.stanceLeg == 1){
            if(_rsm.s >= 0 - EPSILON && _rsm.s <= 0 + EPSILON){   // 'LO'
                _rtr.qDot_A_sw_integral = _rs.qDot_R_A;
            }
            _rtr.qDot_A_sw_integral(0) += _rtr.qDDot_opt(3)*DT;
            _rtr.qDot_A_sw_integral(1) += _rtr.qDDot_opt(5)*DT;
            _rtr.tauA_cmd.head(2) = _rtr.tauA_opt.head(2) + plan::diag(stdVec_k_sw_qDot_integral)*(_rtr.qDot_A_sw_integral - _rs.qDot_R_A);
        }else{
            if(_rsm.s >= 0 - EPSILON && _rsm.s <= 0 + EPSILON){   // 'LO'
                _rtr.qDot_A_sw_integral = _rs.qDot_L_A;
            }
            _rtr.qDot_A_sw_integral(0) += _rtr.qDDot_opt(7)*DT;
            _rtr.qDot_A_sw_integral(1) += _rtr.qDDot_opt(9)*DT;
            _rtr.tauA_cmd.tail(2) = _rtr.tauA_opt.tail(2) + plan::diag(stdVec_k_sw_qDot_integral)*(_rtr.qDot_A_sw_integral - _rs.qDot_L_A);
        }
    }else {
        _rtr.tauA_cmd = _rtr.tauA_opt;
    }

    // calculate output
    calcJointDes(_rsm, _rs, _rd, _rtr);
    calcPVTPID(_rs, _rd, _rtr);

    // reset
    robot_->resetIsPosVelUpdated();
    return true;
}
// ---------------------------- private functions --------------------------------------------

bool taskControl::getOptResult(Eigen::VectorXd &qDDot_opt, Eigen::VectorXd &fcPf_opt, Eigen::VectorXd &tauA_opt){
    Eigen::VectorXd x_opt(nV);
    if (qp_flag_ == 1){
        wbch_->getResultOpt(x_opt);
    }
    if (qp_flag_==0) {
        wbc_->getResultOpt(x_opt);
    }
    if (qp_flag_==2){
        wbcn_->getResultOpt(x_opt);
    }

    qDDot_opt = x_opt.head(nJg);
    fcPf_opt = x_opt.tail(nFc);

    tauA_opt = robot_->Pa * x_opt + robot_->Qa;

    return true;
}

bool taskControl::testClosedLoopConsistent(const Eigen::VectorXd &qDDot_opt, Eigen::Vector4d &xRzRxLzL_ddot_ClosedLoop){
    xRzRxLzL_ddot_ClosedLoop = robot_->CS.G * qDDot_opt - robot_->CS.gamma;
//    std::cout << "xRzRxLzL_ddot_ClosedLoop ^T = " << std::endl <<
//                 xRzRxLzL_ddot_ClosedLoop.transpose() << std::endl;
    return true;
}

bool taskControl::calcJointDes(const robotStateMachine & _rsm, const robotState & _rs, const robotDesired & _rd, robotTaskRef & _rtr){

    // joint Pos, Vel in "frrl"
    _rtr.q_R_A_d = dl2D->ikPosCartes2D(_rd.c_R_B_d);
    _rtr.q_L_A_d = dl2D->ikPosCartes2D(_rd.c_L_B_d);
    _rtr.qDot_R_A_d = _rs.Jc_R.inverse() * _rd.cDot_R_B_d;
    _rtr.qDot_L_A_d = _rs.Jc_L.inverse() * _rd.cDot_L_B_d;


    // -------------------------- NaN protection of desired joint angle --------------------------------//
    if (isnan(_rtr.q_R_A_d(0))){
        _rtr.q_R_A_d(0) = _rs.q_R_A(0);
//        std::cout << "Calculate NaN in desired joint angle of hip-Fore-Right, at " << _rsm.timeCs << " (s)." << std::endl;
    }
    if (isnan(_rtr.q_R_A_d(1))){
        _rtr.q_R_A_d(1) = _rs.q_R_A(1);
//        std::cout << "Calculate NaN in desired joint angle of hip-Rear-Right, at " << _rsm.timeCs << " (s)." << std::endl;
    }
    if (isnan(_rtr.q_L_A_d(0))){
        _rtr.q_L_A_d(0) = _rs.q_L_A(0);
//        std::cout << "Calculate NaN in desired joint angle of hip-Fore-Left, at " << _rsm.timeCs << " (s)." << std::endl;
    }
    if (isnan(_rtr.q_L_A_d(1))){
        _rtr.q_L_A_d(1) = _rs.q_L_A(1);
//        std::cout << "Calculate NaN in desired joint angle of hip-Rear-Left, at " << _rsm.timeCs << " (s)." << std::endl;
    }
    // -------------------------- NaN protection of desired joint angle --------------------------------//

    // joint torque in "frrl"
    _rtr.u_R_A_d = _rtr.tauA_cmd.head(2);
    _rtr.u_L_A_d = _rtr.tauA_cmd.tail(2);


    return true;
}

bool taskControl::calcPVTPID(const robotState & _rs, const robotDesired & _rd, robotTaskRef &_rtr){
    // -------------------------------  PVT  -----------------------------------
    // Pos, Vel, Tor in "frrl"
    _rtr.q_frrl_d.at(0) = _rtr.q_R_A_d(0);
    _rtr.q_frrl_d.at(1) = _rtr.q_R_A_d(1);
    _rtr.q_frrl_d.at(2) = _rtr.q_L_A_d(0);
    _rtr.q_frrl_d.at(3) = _rtr.q_L_A_d(1);

    _rtr.qDot_frrl_d.at(0) = _rtr.qDot_R_A_d(0);
    _rtr.qDot_frrl_d.at(1) = _rtr.qDot_R_A_d(1);
    _rtr.qDot_frrl_d.at(2) = _rtr.qDot_L_A_d(0);
    _rtr.qDot_frrl_d.at(3) = _rtr.qDot_L_A_d(1);

    _rtr.tauA_frrl_d.at(0) = _rtr.u_R_A_d(0);
    _rtr.tauA_frrl_d.at(1) = _rtr.u_R_A_d(1);
    _rtr.tauA_frrl_d.at(2) = _rtr.u_L_A_d(0);
    _rtr.tauA_frrl_d.at(3) = _rtr.u_L_A_d(1);

    // Pos, Vel, Tor in "rr"
    _rtr.q_rr_d.at(0) = -_rtr.q_R_A_d(1) + 2 * PI - _rs.qZero_frrl[1];
    _rtr.q_rr_d.at(1) = -_rtr.q_R_A_d(0) + 2 * PI - _rs.qZero_frrl[0];
    _rtr.q_rr_d.at(2) = _rtr.q_L_A_d(0) - _rs.qZero_frrl[2];
    _rtr.q_rr_d.at(3) = _rtr.q_L_A_d(1) - _rs.qZero_frrl[3];

    _rtr.qDot_rr_d.at(0) = -_rtr.qDot_R_A_d(1);
    _rtr.qDot_rr_d.at(1) = -_rtr.qDot_R_A_d(0);
    _rtr.qDot_rr_d.at(2) = _rtr.qDot_L_A_d(0);
    _rtr.qDot_rr_d.at(3) = _rtr.qDot_L_A_d(1);

    _rtr.tauA_rr_d.at(0) = -_rtr.u_R_A_d(1);
    _rtr.tauA_rr_d.at(1) = -_rtr.u_R_A_d(0);
    _rtr.tauA_rr_d.at(2) = _rtr.u_L_A_d(0);
    _rtr.tauA_rr_d.at(3) = _rtr.u_L_A_d(1);

    // limit absolute velocity commands
    for (int i = 0 ; i != 4; ++i){
        _rtr.qDot_frrl_d[i] = plan::clamp(_rtr.qDot_frrl_d[i], -jointQdot_limit_, jointQdot_limit_);
        _rtr.qDot_rr_d[i] = plan::clamp(_rtr.qDot_rr_d[i], -jointQdot_limit_, jointQdot_limit_);
    }
    // limit absolute torque commands
    for (int i = 0 ; i != 4; ++i){
        _rtr.tauA_frrl_d[i] = plan::clamp(_rtr.tauA_frrl_d[i], -jointTau_limit_, jointTau_limit_);
        _rtr.tauA_rr_d[i] = plan::clamp(_rtr.tauA_rr_d[i], -jointTau_limit_, jointTau_limit_);
    }
    // ---------------------------- end of PVT  --------------------------------

    _rtr.q_d_(2) = _rd.pitch_d;
    _rtr.q_d_(3) = _rtr.q_frrl_d.at(0);
    _rtr.q_d_(5) = _rtr.q_frrl_d.at(1);
    _rtr.q_d_(7) = _rtr.q_frrl_d.at(2);
    _rtr.q_d_(9) = _rtr.q_frrl_d.at(3);

    _rtr.qDot_d_(2) = _rd.pitchDot_d;
    _rtr.qDot_d_(3) = _rtr.qDot_frrl_d.at(0);
    _rtr.qDot_d_(5) = _rtr.qDot_frrl_d.at(1);
    _rtr.qDot_d_(7) = _rtr.qDot_frrl_d.at(2);
    _rtr.qDot_d_(9) = _rtr.qDot_frrl_d.at(3);


    // -------------------------------  PID  -----------------------------------
    _rtr.p_pid_rr.at(0) = _rd.p_pid_frrl.at(1);
    _rtr.i_pid_rr.at(0) = _rd.i_pid_frrl.at(1);
    _rtr.d_pid_rr.at(0) = _rd.d_pid_frrl.at(1);
    _rtr.p_pid_rr.at(1) = _rd.p_pid_frrl.at(0);
    _rtr.i_pid_rr.at(1) = _rd.i_pid_frrl.at(0);
    _rtr.d_pid_rr.at(1) = _rd.d_pid_frrl.at(0);
    for (int i = 2; i != 4; ++i) {
        _rtr.p_pid_rr.at(i) = _rd.p_pid_frrl.at(i);
        _rtr.i_pid_rr.at(i) = _rd.i_pid_frrl.at(i);
        _rtr.d_pid_rr.at(i) = _rd.d_pid_frrl.at(i);
    }
    // ---------------------------- end of PID  --------------------------------

    return true;
}
