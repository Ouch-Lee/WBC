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

/*
 * The arguments of the main() can be specified by the "controllerArgs" field of the Robot node.
 */
int main(int argc, char **argv) {

    bool inLog_flag = false;
    runWebots(inLog_flag);

    return 0;
}


bool runWebots(bool inLogFlag){

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
    // do not modiy below
    newController newCtrl(1);
    newCtrl.setDebugFlag(1); // simulation
    newCtrl.setActuatorModeFlag(0); // actuator in Torque-Mode
    newCtrl.setInitFlag(0);
    newCtrl.setBehaviorFlag(3); // walking
    // do not modiy up

    // Setting. Here, you can modiy
    newCtrl.setVelHgtCmdFlag(1);    // see "robotMessage.h"
    newCtrl.setVelFlag(1);      // 0: CoM; 1: Upper Body
    // Setting

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
    std::string file_out_log_csv = "/home/jun/Documents/Webots/Diamond2D/controllers/wbc_mario2d_SI/local/outLog.csv";
    outLog_csv.open( file_out_log_csv );

    // for inLog.csv
    std::ofstream inLog_dat;
    if (inLogFlag){
        std::string file_in_log_dat = "/home/jun/Documents/Webots/Diamond2D/controllers/wbc_mario2d_SI/local/inLog.dat";
        inLog_dat.open( file_in_log_dat );
    }

    // Realtime simulation starts & looping
    while (webotsDia2D.robot->step(TIME_STEP) != -1)
    {
        simTime = webotsDia2D.robot->getTime();
        std::cout << "current time: " << simTime << " (s)" << std::endl;

        // for PUSH RECOVERY setting
//        double delta_t = 0.01;
//        double force_push = 900;
//        // Apply external force to upper-body
//        if (simTime - goStandTime >= 5 && simTime - goStandTime < 5 + delta_t){
//            webotsDia2D.applyUpperBodyForce(Vec3<double>(force_push, 0., 0.));
//        }else if (simTime - goStandTime >= 10 && simTime - goStandTime < 10 + delta_t){
//            webotsDia2D.applyUpperBodyForce(Vec3<double>(force_push, 0., 0.));
//        }else if (simTime - goStandTime >= 15 && simTime - goStandTime < 15 + delta_t){
//            webotsDia2D.applyUpperBodyForce(Vec3<double>(force_push, 0., 0.));
//        }else if (simTime - goStandTime >= 20 && simTime - goStandTime < 20 + delta_t){
//            webotsDia2D.applyUpperBodyForce(Vec3<double>(force_push, 0., 0.));
//        }else if (simTime - goStandTime >= 25 && simTime - goStandTime < 25 + delta_t){
//            webotsDia2D.applyUpperBodyForce(Vec3<double>(force_push, 0., 0.));
//        }

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
            standPosCmd << 2.679999820129237, 3.603185487050349, 2.679999820129237, 3.603185487050349;
            webotsDia2D.sendPosCammand(standPosCmd);
            // do not record
        }
        else    // your control loops
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

            if (simCnt - goStandCnt == 0){
                Vx_cmd = 0.0;
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

            jointTorCmd = bipedDesired.jointTorDes
                            + bipedDesired.jointPpidDes.asDiagonal()*(bipedDesired.jointPosDes - bipedState.jointPosAct)
                            + bipedDesired.jointDpidDes.asDiagonal()*(bipedDesired.jointVelDes - bipedState.jointVelAct);
            webotsDia2D.sendTorCammand(jointTorCmd);


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
