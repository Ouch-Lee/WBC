/**
 * @file RobotDynamics.cpp
 * @brief Function implementation part of class RobotDynamics
 * @author Jiajun Wang
 * @date 2020-09-14
 * @version alpha
 * @copyright Copyright (c) 2015-2020 UBT-Beijing
 */

#include "robotDynamics.h"

bool RobotDynamics::setQ_Qdot(const Eigen::VectorXd & q, const Eigen::VectorXd & qdot){
    if (check(q, NJG) && check(qdot, NJG)){
        Q = q;
        Qdot = qdot;
        calcWbcDependence_done = false;
    }else{
        return false;
    }
    return true;
}

bool RobotDynamics::displayDynamicInformation(){
    std::cout << "================ Dynamic Information : ================ " << std::endl;
    std::cout << "Q^T = " << std::endl
              << Q.transpose() << std::endl
              << "Qdot^T = " << std::endl
              << Qdot.transpose() << std::endl
              << "M = " << std::endl
              << M << std::endl
              << "M^-1 = " << std::endl
              << invM << std::endl
              << "bng^T = " << std::endl
              << bng.transpose() << std::endl
              << "c_ic^T = " << std::endl
              << c_ic.transpose() << std::endl
              << "AG = " << std::endl
              << AG << std::endl
              << "AGdotQdot^T = " << std::endl
              << AGdotQdot.transpose() << std::endl;
    std::cout << "================ Dynamic Information End ================" << std::endl;
    return true;
}

bool RobotDynamics::isCalcWbcDependenceDone(){
    return calcWbcDependence_done;
}

bool RobotDynamics::check(const Eigen::MatrixXd & M, int row, int col){
    if (M.rows() != row || M.cols() != col){
        std::cout << "Error : Matrix dimensions do not match!" << std::endl;
        return false;
    }
    return true;
}

bool RobotDynamics::check(const Eigen::VectorXd & v, int row){
    if (v.rows() != row){
        std::cout << "Error : Vector dimensions do not match!" << std::endl;
        return false;
    }
    return true;
}
