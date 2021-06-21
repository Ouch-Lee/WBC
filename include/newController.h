//
// Created by jun on 2020-8-19.
//

#ifndef NEW_CONTROLLER_H
#define NEW_CONTROLLER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "robotMessage.h"
#include "stateEstimation.h"
#include "motionPlan.h"
#include "taskControl.h"


class newController
{
public:

    newController(int massRatioFlag = 1);
    ~newController();

    bool init();    // only call init() one time
    bool setBehaviorFlag(int behaviorFlag);
    bool setVelHgtCmdFlag(int velHgtCmdFlag);
    bool setActuatorModeFlag(int actuatorModeFlag);
    bool setVelTrkFlag(int velTrkFlag);
    bool setStandUpFlag(int standUpFlag);
    bool setInitFlag(int initialFlag);
    bool setStandParams(double Hn_stand, double time_interpolating, double time_total);
    bool setDebugFlag(int debugFlag);

    // call update() every loop
    bool update(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                const double * j_pos, const double * j_vel, const double * j_tor);

    int getNumLogData();
    bool getLogData(double* logData);                   // write userOut to double[] --- C-style array
    bool getLogData(std::vector<double>& logData);      // write userOut to std::vector<double> &
    int getActuatorModeFlag();

    int getEmerFlag();

    // get Tau
    bool getValueTau(std::vector<double>& Tor);
    // get PVT
    bool getValuePVT(std::vector<double>& Pos, std::vector<double>& Vel, std::vector<double>& Tor);
    // get PID gains of PVT
    bool getValuePID(std::vector<double>& P_PID, std::vector<double>& I_PID, std::vector<double>& D_PID);

    /*
     *  for simulation : use sensor "GRF" to estimate "s_st" & "s_sw"
     */
    bool update(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                      const double * j_pos, const double * j_vel, const double * j_tor, const double * grf);
    bool get_PVTPID_FRRRFLRL_Debug(std::vector<double>& P_PVT, std::vector<double>& V_PVT, std::vector<double>& T_PVT,
                                    std::vector<double>& P_PID, std::vector<double>& I_PID, std::vector<double>& D_PID);

private:

    // ============================== private functions ============================================

    bool holdInAir(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                   const double * j_pos, const double * j_vel, const double * j_tor);
    bool testInAir(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                   const double * j_pos, const double * j_vel, const double * j_tor);
    bool tryStandUp(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                  const double * j_pos, const double * j_vel, const double * j_tor);
    bool walking(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                 const double * j_pos, const double * j_vel, const double * j_tor);
    bool tryStop(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                 const double * j_pos, const double * j_vel, const double * j_tor);
    bool testSquat(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                   const double * j_pos, const double * j_vel, const double * j_tor);
    bool standBalance(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                        const double * j_pos, const double * j_vel, const double * j_tor);

    bool initTimer();
    bool reConfigCtrlParams();  // base on 'actuatorMode_flag'

//    bool convertFRRL2RealRobot();
//    bool convertRealRobot2FRRL();

    bool userOutLog();

    /*
     *  for simulation : use sensor "GRF" to estimate "s_st" & "s_sw"
     */
    bool tryStandUp(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                        const double * j_pos, const double * j_vel, const double * j_tor, const double * grf);
    bool walking(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                        const double * j_pos, const double * j_vel, const double * j_tor, const double * grf);
    bool standBalance(double timeCS, double Vx_cmd, int H_flag_cmd, const double * imu_data,
                        const double * j_pos, const double * j_vel, const double * j_tor, const double * grf);
    // =========================== end of private functions ========================================

    // ============================= common variable ===============================================
    // pointer
    RobotDynamics_Mario2D * Mario;
    stateEstimation * SE;
    motionPlan * MP;
    taskControl * TC;

    // massage
    robotMessage rm;

    // for logData
    int num_userOut{160};
    std::vector<double> userOut;
    // ========================== end of common variable ===========================================

    // ============================= Multithreaded Programming =====================================

    // ========================== end of Multithreaded Programming =================================
};

#endif //NEW_CONTROLLER_H
