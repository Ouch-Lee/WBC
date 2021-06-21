//
// Created by jun on 2020-8-19.
//

#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <deque>

#include "robotMessage.h"
#include "DiamondKinematics.h"
#include "planToolkit.h"
#include "robotDynamics_Mario2D.h"

class GrfDector
{
public:
    GrfDector();
    GrfDector(int buffer_volume, double critical_slope);
    ~GrfDector() = default;

    bool update(double s, double force);
    double getRiseFlag();

private:
    std::deque<double> forceDotBuffer;
    int bufferVolume_{3};           // set value in constructor 'GrfDector(int, double)'

    int riseFlag_{-1};

    double forceDot_{0.0};
    double force_pre_{0.0};
    double criticalSlope_{0.8/DT};  // set value in constructor 'GrfDector(int, double)'
};


class stateEstimation
{
public:

    stateEstimation();
    ~stateEstimation();

    bool readSensor(const double * imu_data, const double * j_pos, const double * j_vel , const double * j_tor, robotState & _rs);
    bool stateDetecting(robotStateMachine & _rsm, robotState & _rs);      // only be usefull for 'walking'
    bool kinematicsCalculating(robotStateMachine &_rsm, robotState & _rs, RobotDynamics_Mario2D * mario);
    bool stateEstimating(robotStateMachine & _rsm, robotState & _rs, RobotDynamics_Mario2D * mario);

    // for simulation
    bool readSensor(const double * imu_data, const double * j_pos, const double * j_vel , const double * j_tor, const double *grf, robotState & _rs);

private:

    // fk, ik, jac of leg
    DiaKine::DiamondLeg2D * dl2D;

    // dector of TD
    GrfDector * vGrfDector;

    Eigen::VectorXd com;
    Eigen::VectorXd comDot;

    // ---------- filter param ---------------
    // sf: smoothing factor , i.e. time constant for low-pass filter, set to 1, if no filter
    // NOW, Assignment these parameters in 'stateEstimation.cpp'
    double sf_vx = 0.03;                     // 2dSLIP: 0.01; Guo-2dHZD: 0.03; HIPM walking: 0.03
    double sf_vz = 0.3;                     //
    // -------- end of filter param ----------

    double thres_lo{0.2*24.*GRAVITY};       // Lower virtual Leg force threshold for Phase Switch (N)
    double thres_hi{0.8*24.*GRAVITY};       // Upper virtual Leg force threshold for Phase Switch (N)
    double thres_confidence{0.7};           // when s_st >= thres_confidence, update velocity filter

    /*
     *  correct the error caused by the circular-motion
     */
    double r_center_spindle{1.815};         // the distance from spindle to center of robot (m)
    double d_feet{0.236};                   // the distance between two feet-points (m), foot-point-center : 0.236m
    double ratio_CenByInner{r_center_spindle/(r_center_spindle - 0.5*d_feet)};
    double ratio_CenByOuter{r_center_spindle/(r_center_spindle + 0.5*d_feet)};

    double x_inner{0.0};                    // FK-computed position of x on inner vertical-plane, when inner leg is supporting
    double x_outer{0.0};                    // FK-computed position of x on inner vertical-plane, when outer leg is supporting


};

#endif //STATE_ESTIMATION_H
