/**
 * @file constraintDefinition_Mario2D.cpp
 * @brief Function implementation part of the subclasses, that are declared in constraintDefinition_Mario2D.h
 * @author Jiajun Wang
 * @date 2020-09-01
 * @version alpha
 * @copyright Copyright (c) 2015-2020 UBT-Beijing
 */

#include "constraintDefinition_Mario2D.h"

bool Mario2dDynamicConsistency::update(const RobotDynamics &robot){
    C.leftCols(robot.NJG) = robot.M.topRows(robot.NJF);
    C.rightCols(robot.NFC) = -(robot.J_c*robot.N_ic).transpose().topRows(robot.NJF);
    lbC = -robot.c_ic.head(robot.NJF);
    ubC = -robot.c_ic.head(robot.NJF);
    return true;
}

bool Mario2dFrictionCone::setParameter(const std::vector<double> &params){
    if (params.size() >= 2){
        mu_static_ = params.at(0);
        myInfinity_ = params.at(1);
    }else if(params.size() == 1){
        mu_static_ = params.at(0);
    }else{
        std::cout << "Error : Data empty !" << std::endl;
        return false;
    }
    return true;
}

bool Mario2dFrictionCone::update(const RobotDynamics &robot){
    C.rightCols(robot.NFC) << 1., mu_static_, 0., 0.,
                                1., -mu_static_, 0., 0.,
                                0., 0., 1., mu_static_,
                                0., 0., 1., -mu_static_;
    lbC << 0., -myInfinity_, 0., -myInfinity_;
    ubC << myInfinity_, 0., myInfinity_, 0.;
    return true;
}

bool Mario2dJointTorqueSaturation::setParameter(const std::vector<double> &params){
    if(params.size() >= 1){
        jointTau_limit_ = params.at(0);
    }else{
        std::cout << "Error : Data empty !" << std::endl;
        return false;
    }
    return true;
}

bool Mario2dJointTorqueSaturation::update(const RobotDynamics &robot){
    C = robot.Pa;
    lbC = - jointTau_limit_ * Eigen::VectorXd::Ones(robot.NJA) - robot.Qa;
    ubC = jointTau_limit_ * Eigen::VectorXd::Ones(robot.NJA) - robot.Qa;
    return true;
}

bool Mario2dKneeSingularity::setParameter(const std::vector<double> &params){
    if(params.size() >= 3){
        delta_t_ = params.at(0);
        knee_absMin_ = params.at(1);
        knee_absMax_ = params.at(2);
    }else if(params.size() == 2){
        delta_t_ = params.at(0);
        knee_absMin_ = params.at(1);
    }else if(params.size() == 1){
        delta_t_ = params.at(0);
    }
    else{
        std::cout << "Error : Data empty !" << std::endl;
        return false;
    }
    return true;
}

bool Mario2dKneeSingularity::update(const RobotDynamics &robot){
    C.leftCols(robot.NJG) = 0.5 * delta_t_ * delta_t_ * robot.Sp;
    lbC = Eigen::Vector4d(knee_absMin_, -knee_absMax_, knee_absMin_, -knee_absMax_)
          - robot.Sp * robot.Q - delta_t_ * robot.Sp * robot.Qdot;
    ubC = Eigen::Vector4d(knee_absMax_, -knee_absMin_, knee_absMax_, -knee_absMin_)
          - robot.Sp * robot.Q - delta_t_ * robot.Sp * robot.Qdot;
    return true;
}

bool Mario2dCenterOfPressure::setParameter(const std::vector<double> &params){
    if (params.size() >= 3){
        rear_flag_ = params.at(0);
        distance_feet_ = params.at(1);
        x_cop_lb_ = params.at(2);
        x_cop_ub_ = params.at(3);
    }else if(params.size() == 2){
        rear_flag_ = params.at(0);
        distance_feet_ = params.at(1);
    }else if(params.size() == 1){
        rear_flag_ = params.at(0);
    }else{
        std::cout << "Error : Data empty !" << std::endl;
        return false;
    }
    return true;
}

bool Mario2dCenterOfPressure::update(const RobotDynamics &robot){
    if (rear_flag_ >= 0. - 1e-6){    // left foot is the rear foot
        C.rightCols(robot.NFC) << 0., distance_feet_ - x_cop_lb_, 0., -x_cop_lb_,
                                  0., distance_feet_ - x_cop_ub_, 0., -x_cop_ub_;
        lbC << 0., -myInfinity_;
        ubC << myInfinity_, 0.;
    }else{                      // right foot is the rear foot
        C.rightCols(robot.NFC) << 0., -x_cop_lb_, 0., distance_feet_ - x_cop_lb_,
                                  0., -x_cop_ub_, 0., distance_feet_ - x_cop_ub_;
        lbC << 0., -myInfinity_;
        ubC << myInfinity_, 0.;
    }

    return true;
}
