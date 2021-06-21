/**
 * @file main.cpp
 * @brief the main function file to run wbc-alpha in Webots.
 * @author Jiajun Wang
 * @date 2020-06-28
 * @version
 * @copyright
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include <math.h>

#include <qpOASES.hpp>

#include "webotsInterface.h"
#include "newController.h"
#include "dataProcessToolkit.h"

bool runWebots(bool inLogFlag);
bool runLogWithoutWebots();

/*
 * The arguments of the main() can be specified by the "controllerArgs" field of the Robot node.
 */
int main(int argc, char **argv) {

    bool inLog_flag = true;
    runWebots(inLog_flag);

//    runLogWithoutWebots();

    return 0;
}


bool runWebots(bool inLogFlag){

    int sim_flag = 0; // 0: ZERO, put on 0.37m box; 1: 0.44m stand well

    int simCnt = 0;
    double simTime = 0;

    const int goStandCnt = 0;         // 200
    const double goStandTime = goStandCnt * SAMPLE_TIME;	//second

    const int simStopCnt  = goStandCnt + 30000;
    const double simStopTime = simStopCnt * SAMPLE_TIME;    //second

    webotsRobot webotsDia2D;
    webotsDia2D.initWebots();   // Initialize Webots
    webotState bipedState;
    webotDesired bipedDesired;

    Vec4<double> standPosCmd = Vec4<double>::Zero();
    Vec4<double> jointTorCmd = Vec4<double>::Zero();

    // controller timekeeping
    double timeCtrl{0.0};
    double timeCtrl_all{0.0};
    double timeCtrl_max{0.0};
    int countCtrl{0};

    // Running time
    struct timespec start, end;
    clock_gettime(CLOCK_REALTIME, &start);
    std::cout << "Program strated!" << std::endl;

    // ------------------ for Controller ----------------------------------
    // new controller
    newController newCtrl(1);
    newCtrl.setDebugFlag(1);
    newCtrl.setActuatorModeFlag(0);
    newCtrl.setInitFlag(0);
    newCtrl.setBehaviorFlag(3);

    double Vx_cmd{0.0};
    int Hb_flag_cmd{0};
    double imu_data[9];
    double j_pos[4];
    double j_vel[4];
    double j_tor[4];
    double grf[6];
    std::vector<double> P_PVT(4), V_PVT(4), T_PVT(4), P_PID(4), I_PID(4), D_PID(4);
    // ---------------- end of for Controller ------------------------------

    // for outLog.csv
    std::ofstream outLog_csv;
    std::string file_out_log_csv = "/home/jun/Documents/Webots/Diamond2D/controllers/wbc_alpha_mario2d/local/outLog.csv";
    outLog_csv.open( file_out_log_csv );

    // for inLog.csv
    std::ofstream inLog_dat;
    if (inLogFlag){
        std::string file_in_log_dat = "/home/jun/Documents/Webots/Diamond2D/controllers/wbc_alpha_mario2d/local/inLog.dat";
        inLog_dat.open( file_in_log_dat );
    }

    // Realtime simulation starts & looping
    while (webotsDia2D.robot->step(TIME_STEP) != -1)
    {
        simTime = webotsDia2D.robot->getTime();
        std::cout << "current time: " << simTime << " (s)" << std::endl;

        // for PUSH RECOVERY setting
        double delta_t = 0.01;
        double force_push = 900;
        // Apply external force to upper-body
        if (simTime - goStandTime >= 5 && simTime - goStandTime < 5 + delta_t){
            webotsDia2D.applyUpperBodyForce(Vec3<double>(force_push, 0., 0.));
        }else if (simTime - goStandTime >= 10 && simTime - goStandTime < 10 + delta_t){
            webotsDia2D.applyUpperBodyForce(Vec3<double>(force_push, 0., 0.));
        }else if (simTime - goStandTime >= 15 && simTime - goStandTime < 15 + delta_t){
            webotsDia2D.applyUpperBodyForce(Vec3<double>(force_push, 0., 0.));
        }else if (simTime - goStandTime >= 20 && simTime - goStandTime < 20 + delta_t){
            webotsDia2D.applyUpperBodyForce(Vec3<double>(force_push, 0., 0.));
        }else if (simTime - goStandTime >= 25 && simTime - goStandTime < 25 + delta_t){
            webotsDia2D.applyUpperBodyForce(Vec3<double>(force_push, 0., 0.));
        }

        // Read data from Webots
        webotsDia2D.readData(simTime, bipedState);
        // data transform
        imu_data[1] = bipedState.torsoRpyAct(1,0); imu_data[7] = bipedState.torsoAnvAct(1,0);
        j_pos[0] = bipedState.jointPosAct(0,0); j_pos[1] = bipedState.jointPosAct(1,0);
        j_pos[2] = bipedState.jointPosAct(2,0); j_pos[3] = bipedState.jointPosAct(3,0);
        j_vel[0] = bipedState.jointVelAct(0,0); j_vel[1] = bipedState.jointVelAct(1,0);
        j_vel[2] = bipedState.jointVelAct(2,0); j_vel[3] = bipedState.jointVelAct(3,0);
        grf[0] = bipedState.footGrfAct(0,0); grf[1] = bipedState.footGrfAct(1,0); grf[2] = bipedState.footGrfAct(2,0);
        grf[3] = bipedState.footGrfAct(3,0); grf[4] = bipedState.footGrfAct(4,0); grf[5] = bipedState.footGrfAct(5,0);

        // Send data to Webots
        //*****************************************************//
        if (simCnt < goStandCnt)    // Go to stand pos
        {
            if (sim_flag == 0){
                standPosCmd << 1.89169, 4.39149, 1.89169, 4.39149;
            }else{
                standPosCmd << 2.679999820129237, 3.603185487050349, 2.679999820129237, 3.603185487050349;
            }
            webotsDia2D.sendPosCammand(standPosCmd);
            // do not record
        }
        else    // SetMove
        {
            if (inLogFlag){
                writeDouble2DAT(simTime - goStandTime, inLog_dat);
                writeDouble2DAT(Vx_cmd, inLog_dat);
                writeDouble2DAT(static_cast<double>(Hb_flag_cmd), inLog_dat);
                writeArray2DAT(imu_data, 9, inLog_dat);
                writeArray2DAT(j_pos, 4, inLog_dat);
                writeArray2DAT(j_vel, 4, inLog_dat);
                writeArray2DAT(j_tor, 4, inLog_dat);
                writeArray2DAT(grf, 6, inLog_dat);
                inLog_dat << std::endl;
            }

            // controller timekeeping calculate
            auto start_chrono = std::chrono::system_clock::now();

            if (sim_flag == 0){

                // stand up
                if (simCnt - goStandCnt == 0){
                    newCtrl.setActuatorModeFlag(3);                 // set 'actuatorMode_flag' to all in 'PV-mode'
                    newCtrl.setBehaviorFlag(1);                     // set 'BehaviorFlag' to 'try to stand-up'
                    newCtrl.setStandUpFlag(0);                      // 0: s_init = 0; 1: s_init = 1; 2: stand-balance
//                    newCtrl.setStandParams(0.42, 5, 5);           // totalStandTime <= t_interpolating
                    newCtrl.setStandParams(0.43, 0.5, 0.5);         // totalStandTime <= t_interpolating
                }

                // stand balance
//                if (simCnt - goStandCnt == 5*1000){
//                    newCtrl.setActuatorModeFlag(0);
//                    newCtrl.setBehaviorFlag(6);                     // set 'BehaviorFlag' to 'stand balance'
//                }

                // walk
                if (simCnt - goStandCnt == 0.5*1000){               // 0.5*1000
                    newCtrl.setActuatorModeFlag(2);                 // set 'actuatorMode_flag' to be 0 or 2 or 4 ?
                    newCtrl.setBehaviorFlag(3);                     // set 'BehaviorFlag' to 'walking'
                    newCtrl.setInitFlag(0);
                    Vx_cmd = 0.0;
                }
            }

            // call controller
            newCtrl.update(simTime - goStandTime, Vx_cmd, Hb_flag_cmd, imu_data, j_pos, j_vel, j_tor, grf);
            newCtrl.get_PVTPID_FRRRFLRL_Debug(P_PVT, V_PVT, T_PVT, P_PID, I_PID, D_PID);

            // output to Webots
            bipedDesired.jointPosDes << P_PVT[0], P_PVT[1], P_PVT[2], P_PVT[3];
            bipedDesired.jointVelDes << V_PVT[0], V_PVT[1], V_PVT[2], V_PVT[3];
            bipedDesired.jointTorDes << T_PVT[0], T_PVT[1], T_PVT[2], T_PVT[3];
            bipedDesired.jointPpidDes << P_PID[0], P_PID[1], P_PID[2], P_PID[3];
            bipedDesired.jointIpidDes << I_PID[0], I_PID[1], I_PID[2], I_PID[3];
            bipedDesired.jointDpidDes << D_PID[0], D_PID[1], D_PID[2], D_PID[3];

//            std::cout << "bipedDesired.jointPpidDes = " << bipedDesired.jointPpidDes.transpose() << std::endl;

            jointTorCmd = bipedDesired.jointTorDes
                            + bipedDesired.jointPpidDes.asDiagonal()*(bipedDesired.jointPosDes - bipedState.jointPosAct)
                            + bipedDesired.jointDpidDes.asDiagonal()*(bipedDesired.jointVelDes - bipedState.jointVelAct);
            webotsDia2D.sendTorCammand(jointTorCmd);

//            if (newCtrl.getActuatorModeFlag() == 0 || newCtrl.getActuatorModeFlag() == 1){
//                jointTorCmd = bipedDesired.jointTorDes
//                                + bipedDesired.jointPpidDes.asDiagonal()*(bipedDesired.jointPosDes - bipedState.jointPosAct)
//                                + bipedDesired.jointDpidDes.asDiagonal()*(bipedDesired.jointVelDes - bipedState.jointVelAct);
//                webotsDia2D.sendTorCammand(jointTorCmd);
//            }else if (newCtrl.getActuatorModeFlag() == 3){
//                standPosCmd = bipedDesired.jointPosDes;
//                webotsDia2D.sendPosCammand(standPosCmd);
//            }


            // controller timekeeping calculate
            auto end_chrono = std::chrono::system_clock::now();
            auto duration_chrono = std::chrono::duration_cast<std::chrono::nanoseconds>(end_chrono - start_chrono);
            timeCtrl = double(duration_chrono.count()) * 1e-6;
    //        std::cout << "Controller Timekeeping: " << timeCtrl << " (ms)" << std::endl;
            timeCtrl_all += timeCtrl;
            timeCtrl_max = std::max(timeCtrl_max, timeCtrl);

            // record log
            std::vector<double> logData;
            newCtrl.getLogData(logData);
            writeVector2CSV(logData, outLog_csv);
            outLog_csv << std::endl;

            // loop
            countCtrl++;
        }
        //*****************************************************//

        // Stop simulation at specified time
        if (simTime > simStopTime){
            break;
        }
        simCnt++;
    };

    // Free memory
    webotsDia2D.deleteRobot();
    // close file
    outLog_csv.close();
    inLog_dat.close();

    // Running time
    clock_gettime(CLOCK_REALTIME, &end);
    // controller timekeeping
    double timeCtrl_average{0.0};
    if (countCtrl != 0){
        timeCtrl_average = timeCtrl_all/countCtrl;
    }

    std::cout << "Program ended." << std::endl;
    std::cout << "Test-Time Escaped: " << (end.tv_sec - start.tv_sec) * 1e3 + (end.tv_nsec - start.tv_nsec) * 1e-6 << " (ms)" << std::endl;
    std::cout << "Average Controller Timekeeping: " << timeCtrl_average << " (ms)" << std::endl;
    std::cout << "Maximal Controller Timekeeping: " << timeCtrl_max << " (ms)" << std::endl;

    return true;
}

bool runLogWithoutWebots(){

    // -------------------------------- read inLogCsv -------------------------------------
    std::string file_in_log_dat{"/home/jun/Documents/Webots/Diamond2D/controllers/wbc_alpha_mario2d/local/inLog.dat"};
    int num_Cols{30};
    // time
    int col_t{0};
    std::vector<double> tLog;
    readLog2Vector(file_in_log_dat, num_Cols, col_t, tLog);
    // Vx_cmd
    int col_vx{1};
    std::vector<double> vxLog;
    readLog2Vector(file_in_log_dat, num_Cols, col_vx, vxLog);
    // Hb_flag_cmd
    int col_h{2};
    std::vector<double> hLog;
    readLog2Vector(file_in_log_dat, num_Cols, col_h, hLog);
    // pitch, pitchDot
    int col_pitch[2]{4, 10};
    std::vector<double> pitchLog, pitchDotLog;
    readLog2Vector(file_in_log_dat, num_Cols, col_pitch[0], pitchLog);
    readLog2Vector(file_in_log_dat, num_Cols, col_pitch[1], pitchDotLog);
    // j_pos
    int col_pos[4]{12, 13, 14, 15};
    std::vector<double> q0Log, q1Log, q2Log, q3Log;
    readLog2Vector(file_in_log_dat, num_Cols, col_pos[0], q0Log);
    readLog2Vector(file_in_log_dat, num_Cols, col_pos[1], q1Log);
    readLog2Vector(file_in_log_dat, num_Cols, col_pos[2], q2Log);
    readLog2Vector(file_in_log_dat, num_Cols, col_pos[3], q3Log);
    // j_vel
    int col_vel[4]{16, 17, 18, 19};
    std::vector<double> dq0Log, dq1Log, dq2Log, dq3Log;
    readLog2Vector(file_in_log_dat, num_Cols, col_vel[0], dq0Log);
    readLog2Vector(file_in_log_dat, num_Cols, col_vel[1], dq1Log);
    readLog2Vector(file_in_log_dat, num_Cols, col_vel[2], dq2Log);
    readLog2Vector(file_in_log_dat, num_Cols, col_vel[3], dq3Log);
    // j_tor
    int col_tor[4]{20, 21, 22, 23};
    std::vector<double> u0Log, u1Log, u2Log, u3Log;
    readLog2Vector(file_in_log_dat, num_Cols, col_tor[0], u0Log);
    readLog2Vector(file_in_log_dat, num_Cols, col_tor[1], u1Log);
    readLog2Vector(file_in_log_dat, num_Cols, col_tor[2], u2Log);
    readLog2Vector(file_in_log_dat, num_Cols, col_tor[3], u3Log);
    // grf
    int col_grf[6]{24, 25, 26, 27, 28, 29};
    std::vector<double> grf0Log, grf1Log, grf2Log, grf3Log, grf4Log, grf5Log;
    readLog2Vector(file_in_log_dat, num_Cols, col_grf[0], grf0Log);
    readLog2Vector(file_in_log_dat, num_Cols, col_grf[1], grf1Log);
    readLog2Vector(file_in_log_dat, num_Cols, col_grf[2], grf2Log);
    readLog2Vector(file_in_log_dat, num_Cols, col_grf[3], grf3Log);
    readLog2Vector(file_in_log_dat, num_Cols, col_grf[4], grf4Log);
    readLog2Vector(file_in_log_dat, num_Cols, col_grf[5], grf5Log);
    // ---------------------------- end of read inLogCsv -------------------------------------

    // for outLog.csv
    std::ofstream outLog_csv;
    std::string file_out_log_csv = "/home/jun/Documents/Webots/Diamond2D/controllers/wbc_alpha_mario2d/local/outLogTest.csv";
    outLog_csv.open( file_out_log_csv );

    // Running time
    struct timespec start, end;
    clock_gettime(CLOCK_REALTIME, &start);
    std::cout << "test strated!" << std::endl;

    // ------------------ for Controller ----------------------------------
    // new controller
    newController newCtrl(1);
    newCtrl.setDebugFlag(1);
    newCtrl.setActuatorModeFlag(0);
    newCtrl.setInitFlag(0);
    newCtrl.setBehaviorFlag(3);

    double TimeCS{0.0};
    double Vx_cmd{0.0};
    int Hb_flag_cmd{0};
    double imu_data[9];
    double j_pos[4];
    double j_vel[4];
    double j_tor[4];
    double grf[6];
    // ---------------- end of for Controller ------------------------------

    // for controller run time
    double timeCtrl{0.0};
    double timeCtrl_all{0.0};
    double timeCtrl_max{0.0};

    std::ofstream timeLog_csv;
    std::string file_time_log_csv = "/home/jun/Documents/Webots/Diamond2D/controllers/wbc_alpha_mario2d/local/timeLogTest.csv";
    timeLog_csv.open( file_time_log_csv );

    int Tick{0};
    while(Tick < static_cast<int>(tLog.size())){

//        std::cout << "Tick = " << Tick << std::endl;

        // Controller Timekeeping calculate
        auto start_chrono = std::chrono::system_clock::now();


        TimeCS = tLog[Tick];
        Vx_cmd = vxLog[Tick];
        Hb_flag_cmd = hLog[Tick];
        imu_data[1] = pitchLog[Tick]; imu_data[7] = pitchDotLog[Tick];
        j_pos[0] = q0Log[Tick]; j_pos[1] = q1Log[Tick]; j_pos[2] = q2Log[Tick]; j_pos[3] = q3Log[Tick];
        j_vel[0] = dq0Log[Tick]; j_vel[1] = dq1Log[Tick]; j_vel[2] = dq2Log[Tick]; j_vel[3] = dq3Log[Tick];
        grf[0] = grf0Log[Tick]; grf[1] = grf1Log[Tick]; grf[2] = grf2Log[Tick];
        grf[3] = grf3Log[Tick]; grf[4] = grf4Log[Tick]; grf[5] = grf5Log[Tick];

        // call controller
        newCtrl.update(TimeCS, Vx_cmd, Hb_flag_cmd, imu_data, j_pos, j_vel, j_tor, grf);

        // Controller Timekeeping calculate
        auto end_chrono = std::chrono::system_clock::now();
        auto duration_chrono = std::chrono::duration_cast<std::chrono::nanoseconds>(end_chrono - start_chrono);
        timeCtrl = double(duration_chrono.count()) * 1e-6;
//        std::cout << "Controller Timekeeping: " << timeCtrl << " (ms)" << std::endl;
        timeCtrl_all += timeCtrl;
        timeCtrl_max = std::max(timeCtrl_max, timeCtrl);

        // record log
        std::vector<double> logData;
        newCtrl.getLogData(logData);
        writeVector2CSV(logData, outLog_csv);
        outLog_csv << std::endl;
        // record log
        std::vector<double> timeData;
        timeData.push_back(TimeCS);
        timeData.push_back(timeCtrl);
        writeVector2CSV(timeData, timeLog_csv);
        timeLog_csv << std::endl;

        // loop
        Tick++;
    }
    std::cout << "Tick = " << Tick << std::endl;
    double timeCtrl_average{0.0};
    if (Tick != 0){
        timeCtrl_average = timeCtrl_all/Tick;
    }
    // close
    outLog_csv.close();
    timeLog_csv.close();
    // Running time
    clock_gettime(CLOCK_REALTIME, &end);
    std::cout << "Test ended." << std::endl;
    std::cout << "Test-Time Escaped: " << (end.tv_sec - start.tv_sec) * 1e3 + (end.tv_nsec - start.tv_nsec) * 1e-6 << " (ms)" << std::endl;
    std::cout << "Average Controller Timekeeping: " << timeCtrl_average << " (ms)" << std::endl;
    std::cout << "Maximal Controller Timekeeping: " << timeCtrl_max << " (ms)" << std::endl;

    return true;
}

