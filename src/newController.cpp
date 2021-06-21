//
// Created by jun on 2020-8-19.
//

#include "newController.h"

/// ============================== the public function members ===================================================== ///

newController::newController(int massRatioFlag){
    // mass Ratio
    if (massRatioFlag < 0 || massRatioFlag >3){
        std::cout << "WRONG 'massRatioFlag' ! USE 'massRatioFlag == 0' by default NOW !" << std::endl;
        rm.rsm.massRatio_flag = 0;
    }else{
        rm.rsm.massRatio_flag = massRatioFlag;
        if (rm.rsm.vel_flag == 0){
            if (rm.rsm.massRatio_flag == 1){
                rm.rd.H_tgt = 0.44;
            }else if (rm.rsm.massRatio_flag == 2){
                rm.rd.H_tgt = 0.45;
            }else if (rm.rsm.massRatio_flag == 3){
                rm.rd.H_tgt = 0.425;
            }
        }else{
            if (rm.rsm.massRatio_flag == 1){
                rm.rd.H_tgt = 0.43; // 0.44
            }else if (rm.rsm.massRatio_flag == 2){
                rm.rd.H_tgt = 0.44;
            }else if (rm.rsm.massRatio_flag == 3){
                rm.rd.H_tgt = 0.44;
            }
        }
    }

    // new pointer
    Mario = new RobotDynamics_Mario2D(rm.rsm.massRatio_flag, 1);
    SE = new stateEstimation();
    MP = new motionPlan();
    TC = new taskControl(Mario, rm.rsm.JcTruncation_flag);
    TC->setParameters(rm.rsm);

    // mass
    rm.rs.massTotal = Mario->getTotalMass();

    // initialize the dimensions of std::vector
    userOut.resize(num_userOut);
}

newController::~newController(){
    delete Mario;
    delete SE;
    delete MP;
    delete TC;
    Mario = nullptr;
    SE = nullptr;
    MP = nullptr;
    TC = nullptr;
}

bool newController::init(){

    return true;
}

bool newController::setBehaviorFlag(int behaviorFlag){
    if (rm.rsm.behavior_flag != behaviorFlag){
        rm.rsm.behavior_flag = behaviorFlag;
    }
    //  Re-initialize tick-tack at every moment of changing the behavior_flag
    initTimer();
    // reconfigure the control parameters
    reConfigCtrlParams();

    return true;
}

bool newController::setVelHgtCmdFlag(int velHgtCmdFlag){
    if (rm.rsm.velHgtCmd_flag != velHgtCmdFlag){
        rm.rsm.velHgtCmd_flag = velHgtCmdFlag;
    }
    return true;
}

bool newController::setVelTrkFlag(int velTrkFlag){
    if (rm.rsm.velTrk_flag != velTrkFlag){
        rm.rsm.velTrk_flag = velTrkFlag;
    }
    return true;
}

bool newController::setActuatorModeFlag(int actuatorModeFlag){
    if (rm.rsm.actuatorMode_flag != actuatorModeFlag){
        rm.rsm.actuatorMode_flag = actuatorModeFlag;
    }
    // reconfigure the control parameters
    reConfigCtrlParams();

    return true;
}

bool newController::setStandUpFlag(int standUpFlag){
    rm.rsm.standUp_flag = standUpFlag;
    return true;
}

bool newController::setInitFlag(int initialFlag){
    rm.rsm.init_flag = initialFlag;
    return true;
}

bool newController::setDebugFlag(int debugFlag){
    rm.rsm.debug_flag = debugFlag;
    return true;
}

bool newController::setStandParams(double Hn_stand, double time_interpolating, double time_total){
    rm.rsm.Hn_tryStand = Hn_stand;

    if (time_interpolating == 0.0){
        rm.rsm.t_interpolating = 0.001;
    } else{
        rm.rsm.t_interpolating = time_interpolating;
    }

    if (time_total <= rm.rsm.t_interpolating ){
        rm.rsm.totalStandTime = rm.rsm.t_interpolating;
    } else{
        rm.rsm.totalStandTime = time_total;
    }

    return true;
}

bool newController::update(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                           const double * j_pos, const double * j_vel, const double * j_tor){
    switch (rm.rsm.behavior_flag){
    case 0:
        holdInAir(timeCS, Vx_cmd, H_flag_cmd, imu_data, j_pos, j_vel, j_tor);
        break;
    case 1:
        tryStandUp(timeCS, Vx_cmd, H_flag_cmd, imu_data, j_pos, j_vel, j_tor);
        break;
    case 2:
        testInAir(timeCS, Vx_cmd, H_flag_cmd, imu_data, j_pos, j_vel, j_tor);
        break;
    case 3:
        walking(timeCS, Vx_cmd, H_flag_cmd, imu_data, j_pos, j_vel, j_tor);
        break;
    case 4:
        tryStop(timeCS, Vx_cmd, H_flag_cmd, imu_data, j_pos, j_vel, j_tor);
        break;
    case 5:
        testSquat(timeCS, Vx_cmd, H_flag_cmd, imu_data, j_pos, j_vel, j_tor);
        break;
    case 6:
        standBalance(timeCS, Vx_cmd, H_flag_cmd, imu_data, j_pos, j_vel, j_tor);
        break;
    default:
        std::cout << " the 'behavior_flag' is illegal! " << std::endl;
    }

    return true;
}

// simulation
bool newController::update(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                                 const double * j_pos, const double * j_vel, const double * j_tor, const double * grf){


    switch (rm.rsm.behavior_flag){
    case 0:
        holdInAir(timeCS, Vx_cmd, H_flag_cmd, imu_data, j_pos, j_vel, j_tor);
        break;
    case 1:
        tryStandUp(timeCS, Vx_cmd, H_flag_cmd, imu_data, j_pos, j_vel, j_tor, grf);
        break;
    case 2:
        testInAir(timeCS, Vx_cmd, H_flag_cmd, imu_data, j_pos, j_vel, j_tor);
        break;
    case 3:
        walking(timeCS, Vx_cmd, H_flag_cmd, imu_data, j_pos, j_vel, j_tor, grf);
        break;
    case 4:
        tryStop(timeCS, Vx_cmd, H_flag_cmd, imu_data, j_pos, j_vel, j_tor);
        break;
    case 5:
        testSquat(timeCS, Vx_cmd, H_flag_cmd, imu_data, j_pos, j_vel, j_tor);
        break;
    case 6:
        standBalance(timeCS, Vx_cmd, H_flag_cmd, imu_data, j_pos, j_vel, j_tor, grf);
        break;
    default:
        std::cout << " the 'behavior_flag' is illegal! " << std::endl;
    }

    return true;
}

int newController::getNumLogData(){
    if(num_userOut != static_cast<int>(userOut.size())){
        std::cout << "Warning: num_userOut != userOut.size() !" << std::endl;
    }
    return static_cast<int>(userOut.size());
}

bool newController::getLogData(double *logData) {
    for (auto i = 0; i != static_cast<int>(userOut.size()); ++i) {
        logData[i] = userOut.at(i);
    }
    return true;
}
bool newController::getLogData(std::vector<double>& logData){
    for (auto item : userOut){
        logData.push_back(item);
    }
    return true;
}

int newController::getActuatorModeFlag(){
    return rm.rsm.actuatorMode_flag;
}

int newController::getEmerFlag(){
    return rm.rsm.emergency_flag;
}

bool newController::getValueTau(std::vector<double>& Tor) {
    Tor.assign(rm.rtr.tauA_rr_d.begin(), rm.rtr.tauA_rr_d.end());
    return true;
}

bool newController::getValuePVT(std::vector<double>& Pos, std::vector<double>& Vel, std::vector<double>& Tor) {
    Pos.assign(rm.rtr.q_rr_d.begin(), rm.rtr.q_rr_d.end());
    Vel.assign(rm.rtr.qDot_rr_d.begin(), rm.rtr.qDot_rr_d.end());
    Tor.assign(rm.rtr.tauA_rr_d.begin(), rm.rtr.tauA_rr_d.end());
    return true;
}

bool newController::getValuePID(std::vector<double> &P_PID, std::vector<double> &I_PID, std::vector<double> &D_PID){
    P_PID.assign(rm.rtr.p_pid_rr.begin(), rm.rtr.p_pid_rr.end());
    I_PID.assign(rm.rtr.i_pid_rr.begin(), rm.rtr.i_pid_rr.end());
    D_PID.assign(rm.rtr.d_pid_rr.begin(), rm.rtr.d_pid_rr.end());
    return true;
}

bool newController::get_PVTPID_FRRRFLRL_Debug(std::vector<double> &P_PVT, std::vector<double> &V_PVT, std::vector<double> &T_PVT,
                                               std::vector<double> &P_PID, std::vector<double> &I_PID, std::vector<double> &D_PID){
    P_PVT.resize(4);
    V_PVT.resize(4);
    T_PVT.resize(4);
    P_PID.resize(4);
    I_PID.resize(4);
    D_PID.resize(4);
    for (int i = 0; i != 4; i++){
        P_PVT.at(i) = rm.rtr.q_frrl_d[i];
        V_PVT.at(i) = rm.rtr.qDot_frrl_d[i];
        T_PVT.at(i) = rm.rtr.tauA_frrl_d[i];
        P_PID.at(i) = rm.rd.p_pid_frrl[i];
        I_PID.at(i) = rm.rd.i_pid_frrl[i];
        D_PID.at(i) = rm.rd.d_pid_frrl[i];
    }

    return true;
}

/// ============================= the private function members ===================================================== ///

bool newController::holdInAir(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                              const double * j_pos, const double * j_vel, const double * j_tor){
    rm.rsm.timeCs = timeCS;
    // Update Time, t
    if ( rm.rsm.tick > 0){
        rm.rsm.Time += DT;
        rm.rsm.t += DT;
    } else {            // tick <= 0
        rm.rsm.tick = 0;
        rm.rsm.Time = 0.0;
        rm.rsm.t = 0.0;
    }

    // Read Sensor Values
    SE->readSensor(imu_data, j_pos, j_vel, j_tor, rm.rs);
    // Decect Events: TD & LO
//    SE->stateDetecting(rm.rsm, rm.rs);
    // State Estimating
    SE->stateEstimating(rm.rsm, rm.rs, Mario);
    // Parse Command
    MP->parseCmd(Vx_cmd, H_flag_cmd, rm.rsm, rm.rs, rm.rd);
    // plan
    MP->holdInAirPlan(rm.rsm, rm.rs, rm.rd);

    // Task Control
    TC->holdCtrl(rm.rsm, rm.rs, rm.rd, rm.rtr);


    // log
    userOutLog();

    // Reset remark_flag
    if (rm.rsm.remark_flag == 1){
        rm.rsm.remark_flag = 0;
    }

    // TODO: Update s_sw_pre, here?
    rm.rsm.s_sw_pre = rm.rsm.s_sw;

    // Update tick-tack
    rm.rsm.tick++;
    if (rm.rsm.tick >= std::numeric_limits<int>::max()-1000){
        rm.rsm.tick = 1;       // avoid tick out-of range
    }
    return true;
}

bool newController::testInAir(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                              const double * j_pos, const double * j_vel, const double * j_tor){
    return true;
}

bool newController::tryStandUp(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                             const double * j_pos, const double * j_vel, const double * j_tor){

    rm.rsm.timeCs = timeCS;
    // Update Time, t
    if ( rm.rsm.tick > 0){
        rm.rsm.Time += DT;
        rm.rsm.t += DT;
    } else {            // tick <= 0
        rm.rsm.tick = 0;
        rm.rsm.Time = 0.0;
        rm.rsm.t = 0.0;
    }

    // Read Sensor Values
    SE->readSensor(imu_data, j_pos, j_vel, j_tor, rm.rs);
    // Decect Events: TD & LO
//    SE->stateDetecting(rm.rsm, rm.rs);
    // State Estimating
    SE->stateEstimating(rm.rsm, rm.rs, Mario);
    // Parse Command
    MP->parseCmd(Vx_cmd, H_flag_cmd, rm.rsm, rm.rs, rm.rd);
    // plan
    MP->standUpPlan(rm.rsm, rm.rs, rm.rd);

    // Task Control
    TC->standUpCtrl(rm.rsm, rm.rs, rm.rd, rm.rtr);

    // log
    userOutLog();

    // Reset remark_flag
    if (rm.rsm.remark_flag == 1){
        rm.rsm.remark_flag = 0;
    }

    // TODO: Update s_sw_pre, here?
    rm.rsm.s_sw_pre = rm.rsm.s_sw;

    // Update tick-tack
    rm.rsm.tick++;
    if (rm.rsm.tick >= std::numeric_limits<int>::max()-1000){
        rm.rsm.tick = 1;       // avoid tick out-of range
    }

    return true;
}

// simulation
bool newController::tryStandUp(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                const double * j_pos, const double * j_vel, const double * j_tor, const double * grf){
    rm.rsm.timeCs = timeCS;
    // Update Time, t
    if ( rm.rsm.tick > 0){
        rm.rsm.Time += DT;
        rm.rsm.t += DT;
    } else {            // tick <= 0
        rm.rsm.tick = 0;
        rm.rsm.Time = 0.0;
        rm.rsm.t = 0.0;
    }

    // Read Sensor Values
    SE->readSensor(imu_data, j_pos, j_vel, j_tor, grf, rm.rs);
    // Decect Events: TD & LO
//    SE->stateDetecting(rm.rsm, rm.rs);
    // State Estimating
    SE->stateEstimating(rm.rsm, rm.rs, Mario);
    // Parse Command
    MP->parseCmd(Vx_cmd, H_flag_cmd, rm.rsm, rm.rs, rm.rd);
    // plan
    MP->standUpPlan(rm.rsm, rm.rs, rm.rd);

    // Task Control
    TC->standUpCtrl(rm.rsm, rm.rs, rm.rd, rm.rtr);

    // log
    userOutLog();

    // Reset remark_flag
    if (rm.rsm.remark_flag == 1){
        rm.rsm.remark_flag = 0;
    }

    // TODO: Update s_sw_pre, here?
    rm.rsm.s_sw_pre = rm.rsm.s_sw;

    // Update tick-tack
    rm.rsm.tick++;
    if (rm.rsm.tick >= std::numeric_limits<int>::max()-1000){
        rm.rsm.tick = 1;       // avoid tick out-of range
    }

    return true;
}

bool newController::walking(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                            const double * j_pos, const double * j_vel, const double * j_tor){
    rm.rsm.timeCs = timeCS;
    // Update Time, t
    if ( rm.rsm.tick > 0){
        rm.rsm.Time += DT;
        rm.rsm.t += DT;
    } else {            // tick <= 0
        rm.rsm.tick = 0;
        rm.rsm.Time = 0.0;
        rm.rsm.t = 0.0;
    }

    // ===================================== Core =======================================

    // Read Sensor Values
    SE->readSensor(imu_data, j_pos, j_vel, j_tor, rm.rs);
    // Decect Events: TD & LO
    SE->stateDetecting(rm.rsm, rm.rs);
    // State Estimating
    SE->stateEstimating(rm.rsm, rm.rs, Mario);

    // Parse Command
    MP->parseCmd(Vx_cmd, H_flag_cmd, rm.rsm, rm.rs, rm.rd);
    // Motion Plan
    if (rm.rsm.vel_flag == 0){
        MP->walkTrajectoryPlan(rm.rsm, rm.rs, rm.rd);
    }else {
        MP->walkTrajectoryPlan_Ub(rm.rsm, rm.rs, rm.rd);
    }

    // Task Control
    if (rm.rsm.tick == 0){
        TC->setParameters(rm.rsm);
    }
    TC->walkCtrl(rm.rsm, rm.rs, rm.rd, rm.rtr);

    // ===================================== Core =======================================

    // log
    userOutLog();

    // Reset remark_flag
    if (rm.rsm.remark_flag == 1){
        rm.rsm.remark_flag = 0;
    }

    // Update tick-tack
    rm.rsm.tick++;
    if (rm.rsm.tick >= std::numeric_limits<int>::max()-1000){
        rm.rsm.tick = 1;       // avoid tick out-of range
    }

    return true;
}

// simulation
bool newController::walking(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                                  const double * j_pos, const double * j_vel, const double * j_tor, const double * grf){
    rm.rsm.timeCs = timeCS;
    // Update Time, t
    if ( rm.rsm.tick > 0){
        rm.rsm.Time += DT;
        rm.rsm.t += DT;
    } else {            // tick <= 0
        rm.rsm.tick = 0;
        rm.rsm.Time = 0.0;
        rm.rsm.t = 0.0;
    }

    // ===================================== Core =======================================

    // Read Sensor Values
    SE->readSensor(imu_data, j_pos, j_vel, j_tor, grf, rm.rs);
    // Decect Events: TD & LO
    SE->stateDetecting(rm.rsm, rm.rs);
    // State Estimating
    SE->stateEstimating(rm.rsm, rm.rs, Mario);

    // Parse Command
    MP->parseCmd(Vx_cmd, H_flag_cmd, rm.rsm, rm.rs, rm.rd);
    // Motion Plan
    if (rm.rsm.vel_flag == 0){
        MP->walkTrajectoryPlan(rm.rsm, rm.rs, rm.rd);
    }else {
        MP->walkTrajectoryPlan_Ub(rm.rsm, rm.rs, rm.rd);

    }

    // Task Control
    if (rm.rsm.tick == 0){
        TC->setParameters(rm.rsm);
    }
    TC->walkCtrl(rm.rsm, rm.rs, rm.rd, rm.rtr);

    // ===================================== Core =======================================

    // log
    userOutLog();

    // Reset remark_flag
    if (rm.rsm.remark_flag == 1){
        rm.rsm.remark_flag = 0;
    }

    // Update tick-tack
    rm.rsm.tick++;
    if (rm.rsm.tick >= std::numeric_limits<int>::max()-1000){
        rm.rsm.tick = 1;       // avoid tick out-of range
    }

    return true;
}

bool newController::testSquat(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                              const double * j_pos, const double * j_vel, const double * j_tor){
    return true;
}

bool newController::standBalance(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                  const double * j_pos, const double * j_vel, const double * j_tor){
    rm.rsm.timeCs = timeCS;
    // Update Time, t
    if ( rm.rsm.tick > 0){
        rm.rsm.Time += DT;
        rm.rsm.t += DT;
    } else {            // tick <= 0
        rm.rsm.tick = 0;
        rm.rsm.Time = 0.0;
        rm.rsm.t = 0.0;
    }

    // ===================================== Core =======================================

    // Read Sensor Values
    SE->readSensor(imu_data, j_pos, j_vel, j_tor, rm.rs);
    // Decect Events: TD & LO
//    SE->stateDetecting(rm.rsm, rm.rs);
    // State Estimating
    SE->stateEstimating(rm.rsm, rm.rs, Mario);

    // Parse Command
    MP->parseCmd(Vx_cmd, H_flag_cmd, rm.rsm, rm.rs, rm.rd);
    // Motion Plan
    if (rm.rsm.vel_flag == 0){
        MP->standBalancePlan(rm.rsm, rm.rs, rm.rd);
    }else {
        MP->standBalancePlan_Ub(rm.rsm, rm.rs, rm.rd);
    }

    // Task Control
    if (rm.rsm.tick == 0){
        TC->setParameters(rm.rsm);
    }
    TC->standBalanceCtrl(rm.rsm, rm.rs, rm.rd, rm.rtr);

    // ===================================== Core =======================================

    // log
    userOutLog();

    // Reset remark_flag
    if (rm.rsm.remark_flag == 1){
        rm.rsm.remark_flag = 0;
    }

    // Update tick-tack
    rm.rsm.tick++;
    if (rm.rsm.tick >= std::numeric_limits<int>::max()-1000){
        rm.rsm.tick = 1;       // avoid tick out-of range
    }

    return true;
}

// simulation
bool newController::standBalance(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                                const double * j_pos, const double * j_vel, const double * j_tor, const double * grf){
    rm.rsm.timeCs = timeCS;
    // Update Time, t
    if ( rm.rsm.tick > 0){
        rm.rsm.Time += DT;
        rm.rsm.t += DT;
    } else {            // tick <= 0
        rm.rsm.tick = 0;
        rm.rsm.Time = 0.0;
        rm.rsm.t = 0.0;
    }

    // ===================================== Core =======================================

    // Read Sensor Values
    SE->readSensor(imu_data, j_pos, j_vel, j_tor, grf, rm.rs);
    // Decect Events: TD & LO
//    SE->stateDetecting(rm.rsm, rm.rs);
    // State Estimating
    SE->stateEstimating(rm.rsm, rm.rs, Mario);

    // Parse Command
    MP->parseCmd(Vx_cmd, H_flag_cmd, rm.rsm, rm.rs, rm.rd);
    // Motion Plan
    if (rm.rsm.vel_flag == 0){
        MP->standBalancePlan(rm.rsm, rm.rs, rm.rd);
    }else {
        MP->standBalancePlan_Ub(rm.rsm, rm.rs, rm.rd);
    }

    // Task Control
    if (rm.rsm.tick == 0){
        TC->setParameters(rm.rsm);
    }
    TC->standBalanceCtrl(rm.rsm, rm.rs, rm.rd, rm.rtr);

    // ===================================== Core =======================================

    // log
    userOutLog();

    // Reset remark_flag
    if (rm.rsm.remark_flag == 1){
        rm.rsm.remark_flag = 0;
    }

    // Update tick-tack
    rm.rsm.tick++;
    if (rm.rsm.tick >= std::numeric_limits<int>::max()-1000){
        rm.rsm.tick = 1;       // avoid tick out-of range
    }

    return true;
}

bool newController::initTimer() {

    rm.rsm.tick = 0;

    rm.rsm.Time = 0.0;
    rm.rsm.t = 0.0;
    rm.rsm.s = 0.0;

    rm.rsm.TD_flag = 0;
    rm.rsm.s_td = 0.0;
    rm.rsm.t_td = rm.rsm.Ts + rm.rsm.Td;

    rm.rsm.remark_flag = 0;
    rm.rsm.stanceLeg_lastStatus = rm.rsm.stanceLeg;
    rm.rsm.stanceLeg = rm.rsm.firstStance_flag;      // first stance-phase

    rm.rs.T_full_pre = 0.0;

    if (rm.rsm.behavior_flag == 6){
        rm.rsm.state = 2; // double support
    }else{
        rm.rsm.state = 1; // single support
    }

    return true;
}

bool newController::tryStop(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                            const double * j_pos, const double * j_vel, const double * j_tor){
    return true;
}

bool newController::reConfigCtrlParams(){
    // reconfiguration of the control parameters base on 'actuatorMode_flag'
    if (rm.rsm.actuatorMode_flag == 0){         // all Tor-mode
        // PID stance & swing
        for (auto & item : rm.rd.p_pid_Lst){
            item = 0.0;
        }
        for (auto & item : rm.rd.i_pid_Lst){
            item = 0.0;
        }
        for (auto & item : rm.rd.d_pid_Lst){
            item = 0.0;
        }
        for (auto & item : rm.rd.p_pid_Rst){
            item = 0.0;
        }
        for (auto & item : rm.rd.i_pid_Rst){
            item = 0.0;
        }
        for (auto & item : rm.rd.d_pid_Rst){
            item = 0.0;
        }
    }else if (rm.rsm.actuatorMode_flag == 1){   // stTor-swPVT
        // PID
        rm.rd.p_pid_Lst = rm.rd.PP_.p_pid_Lst_;
        rm.rd.i_pid_Lst = rm.rd.PP_.i_pid_Lst_;
        rm.rd.d_pid_Lst = rm.rd.PP_.d_pid_Lst_;
        rm.rd.p_pid_Rst = rm.rd.PP_.p_pid_Rst_;
        rm.rd.i_pid_Rst = rm.rd.PP_.i_pid_Rst_;
        rm.rd.d_pid_Rst = rm.rd.PP_.d_pid_Rst_;
        // PID of stance is zero
        rm.rd.p_pid_Lst.at(2) = 0.0;
        rm.rd.p_pid_Lst.at(3) = 0.0;
        rm.rd.i_pid_Lst.at(2) = 0.0;
        rm.rd.i_pid_Lst.at(3) = 0.0;
        rm.rd.d_pid_Lst.at(2) = 0.0;
        rm.rd.d_pid_Lst.at(3) = 0.0;
        rm.rd.p_pid_Rst.at(0) = 0.0;
        rm.rd.p_pid_Rst.at(1) = 0.0;
        rm.rd.i_pid_Rst.at(0) = 0.0;
        rm.rd.i_pid_Rst.at(1) = 0.0;
        rm.rd.d_pid_Rst.at(0) = 0.0;
        rm.rd.d_pid_Rst.at(1) = 0.0;
    }else if (rm.rsm.actuatorMode_flag == 2){   // all PVT-PID
        // PID
        rm.rd.p_pid_Lst = rm.rd.PP_.p_pid_Lst_;
        rm.rd.i_pid_Lst = rm.rd.PP_.i_pid_Lst_;
        rm.rd.d_pid_Lst = rm.rd.PP_.d_pid_Lst_;
        rm.rd.p_pid_Rst = rm.rd.PP_.p_pid_Rst_;
        rm.rd.i_pid_Rst = rm.rd.PP_.i_pid_Rst_;
        rm.rd.d_pid_Rst = rm.rd.PP_.d_pid_Rst_;
    }else if (rm.rsm.actuatorMode_flag == 3){   // all PV-mode
        // PID
        rm.rd.p_pid_Lst = rm.rd.PP_.p_pid_Lst_;
        rm.rd.i_pid_Lst = rm.rd.PP_.i_pid_Lst_;
        rm.rd.d_pid_Lst = rm.rd.PP_.d_pid_Lst_;
        rm.rd.p_pid_Rst = rm.rd.PP_.p_pid_Rst_;
        rm.rd.i_pid_Rst = rm.rd.PP_.i_pid_Rst_;
        rm.rd.d_pid_Rst = rm.rd.PP_.d_pid_Rst_;
    }else if (rm.rsm.actuatorMode_flag == 4){   // PV-D-mode
        // PID
        for (auto & item : rm.rd.p_pid_Lst){
            item = 0.0;
        }
        for (auto & item : rm.rd.i_pid_Lst){
            item = 0.0;
        }
        for (auto & item : rm.rd.d_pid_Lst){
            item = 0.0;
        }
        for (auto & item : rm.rd.p_pid_Rst){
            item = 0.0;
        }
        for (auto & item : rm.rd.i_pid_Rst){
            item = 0.0;
        }
        for (auto & item : rm.rd.d_pid_Rst){
            item = 0.0;
        }
        // swing leg D
        rm.rd.d_pid_Lst.at(0) = rm.rd.PP_.D_qDot_integral;
        rm.rd.d_pid_Lst.at(1) = rm.rd.PP_.D_qDot_integral;
        rm.rd.d_pid_Rst.at(2) = rm.rd.PP_.D_qDot_integral;
        rm.rd.d_pid_Rst.at(3) = rm.rd.PP_.D_qDot_integral;
    }
    return true;
}

bool  newController::userOutLog(){

//     userOut.size() = 160;
    userOut = {
        rm.rs.q_(0), rm.rs.q_(1), rm.rs.q_(2),
        rm.rs.q_(3), rm.rs.q_(4), rm.rs.q_(5), rm.rs.q_(6),
        rm.rs.q_(7), rm.rs.q_(8), rm.rs.q_(9), rm.rs.q_(10),
        rm.rs.qDot_(0), rm.rs.qDot_(1), rm.rs.qDot_(2),
        rm.rs.qDot_(3), rm.rs.qDot_(4), rm.rs.qDot_(5), rm.rs.qDot_(6),
        rm.rs.qDot_(7), rm.rs.qDot_(8), rm.rs.qDot_(9), rm.rs.qDot_(10),
        rm.rtr.q_d_(0), rm.rtr.q_d_(1), rm.rtr.q_d_(2),
        rm.rtr.q_d_(3), rm.rtr.q_d_(4), rm.rtr.q_d_(5), rm.rtr.q_d_(6),
        rm.rtr.q_d_(7), rm.rtr.q_d_(8), rm.rtr.q_d_(9), rm.rtr.q_d_(10),
        rm.rtr.qDot_d_(0), rm.rtr.qDot_d_(1), rm.rtr.qDot_d_(2),
        rm.rtr.qDot_d_(3), rm.rtr.qDot_d_(4), rm.rtr.qDot_d_(5), rm.rtr.qDot_d_(6),
        rm.rtr.qDot_d_(7), rm.rtr.qDot_d_(8), rm.rtr.qDot_d_(9), rm.rtr.qDot_d_(10),
        rm.rtr.fcPf_opt(0), rm.rtr.fcPf_opt(1), rm.rtr.fcPf_opt(2), rm.rtr.fcPf_opt(3),
        rm.rtr.tauA_cmd(0), rm.rtr.tauA_cmd(1), rm.rtr.tauA_cmd(2), rm.rtr.tauA_cmd(3),
        rm.rs.c_R_W(0), rm.rs.c_R_W(1), rm.rs.c_L_W(0), rm.rs.c_L_W(1),
        rm.rs.c_R_S(0), rm.rs.c_R_S(1), rm.rs.c_L_S(0), rm.rs.c_L_S(1),
        rm.rs.c_U_S(0), rm.rs.c_U_S(1), rm.rs.c_Com_S(0), rm.rs.c_Com_S(1),
        rm.rs.cDot_R_W(0), rm.rs.cDot_R_W(1), rm.rs.cDot_L_W(0), rm.rs.cDot_L_W(1),
        rm.rs.cDot_R_S(0), rm.rs.cDot_R_S(1), rm.rs.cDot_L_S(0), rm.rs.cDot_L_S(1),
        rm.rs.cDot_U_S(0), rm.rs.cDot_U_S(1), rm.rs.cDot_Com_S(0), rm.rs.cDot_Com_S(1),
        rm.rd.c_R_W_d(0), rm.rd.c_R_W_d(1), rm.rd.c_L_W_d(0), rm.rd.c_L_W_d(1),
        rm.rd.c_R_S_d(0), rm.rd.c_R_S_d(1), rm.rd.c_L_S_d(0), rm.rd.c_L_S_d(1),
        rm.rd.c_U_S_d(0), rm.rd.c_U_S_d(1), rm.rd.c_Com_S_d(0), rm.rd.c_Com_S_d(1),
        rm.rd.cDot_R_W_d(0), rm.rd.cDot_R_W_d(1), rm.rd.cDot_L_W_d(0), rm.rd.cDot_L_W_d(1),
        rm.rd.cDot_R_S_d(0), rm.rd.cDot_R_S_d(1), rm.rd.cDot_L_S_d(0), rm.rd.cDot_L_S_d(1),
        rm.rd.cDot_U_S_d(0), rm.rd.cDot_U_S_d(1), rm.rd.cDot_Com_S_d(0), rm.rd.cDot_Com_S_d(1),
        rm.rs.u_R_A(0), rm.rs.u_R_A(1), rm.rs.u_L_A(0), rm.rs.u_L_A(1),       
        rm.rs.fc_grf_R_S(0), rm.rs.fc_grf_R_S(1), rm.rs.fc_grf_L_S(0), rm.rs.fc_grf_L_S(1),
        rm.rd.fc_grf_R_S_ff(0), rm.rd.fc_grf_R_S_ff(1), rm.rd.fc_grf_L_S_ff(0), rm.rd.fc_grf_L_S_ff(1),
        rm.rtr.hgDotRef(0), rm.rtr.hgDotRef(1), rm.rtr.hgDotRef(2),
        rm.rtr.qfDDotRef(0), rm.rtr.qfDDotRef(1), rm.rtr.qfDDotRef(2),
        rm.rtr.cPfDDotRef(0), rm.rtr.cPfDDotRef(1), rm.rtr.cPfDDotRef(2), rm.rtr.cPfDDotRef(3),
        rm.rtr.forcePfRef(0), rm.rtr.forcePfRef(1), rm.rtr.forcePfRef(2), rm.rtr.forcePfRef(3),    
        rm.rtr.xRzRxLzL_ddot_ClosedLoop(0), rm.rtr.xRzRxLzL_ddot_ClosedLoop(1), rm.rtr.xRzRxLzL_ddot_ClosedLoop(2), rm.rtr.xRzRxLzL_ddot_ClosedLoop(3),
        rm.rsm.timeCs, rm.rsm.s, rm.rsm.s_td,
        rm.rsm.s_R, rm.rsm.s_L,
        rm.rd.vx_cmd, rm.rd.vx_tgt,
        rm.rd.H_tgt,
        rm.rs.vx_est, rm.rs.vx_est_nf, rm.rs.vx_preStep, rm.rs.vx_double,
        rm.rs.vz_est, rm.rs.vz_est_nf,
        static_cast<double>(rm.rsm.stanceLeg),
        static_cast<double>(rm.rsm.emergency_flag),
        static_cast<double>(rm.rtr.simpleStatus), static_cast<double>(rm.rtr.nWSR_res), rm.rtr.time_QP, rm.rtr.cputime_res, rm.rtr.time_WBC,
        rm.rd.cDDot_U_S_d(0), rm.rd.cDDot_U_S_d(1),
        rm.rtr.qDDot_opt(0), rm.rtr.qDDot_opt(1), rm.rtr.qDDot_opt(2),
        rm.rtr.cPfDDot_opt(0), rm.rtr.cPfDDot_opt(1), rm.rtr.cPfDDot_opt(2), rm.rtr.cPfDDot_opt(3),
    };

    return true;
}

