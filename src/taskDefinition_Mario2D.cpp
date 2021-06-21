/**
 * @file taskDefinition_Mario2D.cpp
 * @brief Function implementation part of the subclasses, that are declared in taskDefinition_Mario2D.h
 * @author Jiajun Wang
 * @date 2020-09-01
 * @version alpha
 * @copyright Copyright (c) 2015-2020 UBT-Beijing
 */

#include "taskDefinition_Mario2D.h"

bool Mario2dCentroidalMoment::update(const RobotDynamics &robot){
    A.leftCols(robot.NJG) = robot.AG;
    b = ref - robot.AGdotQdot;
    return true;
}

bool Mario2dFloatingBasePose::update(const RobotDynamics &robot){
    A.leftCols(robot.NJG) = robot.Sf;
    b = ref;
    return true;
}

bool Mario2dPointFeetCartPosition::setParameter(const std::vector<double> &params){
    if(params.size() >= 1){
        JcTruncation_ = (params.at(0) >= 0. - 1e-6) ? true : false;
    }else{
        std::cout << "Error : Data empty !" << std::endl;
        return false;
    }
    return true;
}

bool Mario2dPointFeetCartPosition::update(const RobotDynamics &robot){
    if (JcTruncation_){
        A.leftCols(robot.NJG) = robot.J_c;
        A.leftCols(robot.NJF) = Eigen::MatrixXd::Zero(A.rows(), robot.NJF);
    }else{
        A.leftCols(robot.NJG) = robot.J_c;

    }
    b = ref - robot.JdotQdot_c;
    return true;
}

bool Mario2dPointFeetCartForce::update(const RobotDynamics &robot){
    A.rightCols(robot.NFC) = Eigen::MatrixXd::Identity(robot.NFC, robot.NFC);
    b = ref;
    return true;
}

bool Mario2dInternalClosedLoop::update(const RobotDynamics &robot){
    A.leftCols(robot.NJG) = robot.CS.G;
    b = robot.CS.gamma;
    return true;
}

bool Mario2dJointTorqueChange::update(const RobotDynamics &robot){
    A = robot.Pa;
    b = ref - robot.Qa;
    return true;
}

bool Mario2dFeetForceChange::update(const RobotDynamics &robot){
    A.rightCols(robot.NFC) = Eigen::MatrixXd::Identity(robot.NFC, robot.NFC);
    b = ref;
    return true;
}
