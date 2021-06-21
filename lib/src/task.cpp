/**
 * @file Task.cpp
 * @brief Function implementation part of class Task
 * @author Jiajun Wang
 * @date 2020-09-01
 * @version alpha
 * @copyright Copyright (c) 2015-2020 UBT-Beijing
 */

#include "task.h"

Task::Task(const std::string & taskName, int taskDim, int varDim){
    name = taskName;
    dim = taskDim;
    varDof = varDim;
    A = Eigen::MatrixXd::Zero(dim, varDof);
    b = Eigen::VectorXd::Zero(dim);
    w = Eigen::VectorXd::Zero(dim);
    ref = Eigen::VectorXd::Zero(dim);
}

bool Task::setParameter(const std::vector<double> & params){
    std::cout << "Nothing to be done, you should rewrite this function, if you want to use it."<< std::endl;
    return true;
}

bool Task::updateRefence(const Eigen::VectorXd & newRef){
    if (check(newRef, dim)){
        ref = newRef;
    }else{
        return false;
    }
    return true;
}

bool Task::updateWeight(const Eigen::VectorXd & newWei){
    if (check(newWei, dim)){
        w = newWei;
    }else{
        return false;
    }
    return true;
}

bool Task::check(const Eigen::MatrixXd & M, int row, int col){
    if (M.rows() != row || M.cols() != col){
        std::cout << "Error : Matrix dimensions do not match!" << std::endl;
        return false;
    }
    return true;
}

bool Task::check(const Eigen::VectorXd & v, int row){
    if (v.rows() != row){
        std::cout << "Error : Vector dimensions do not match!" << std::endl;
        return false;
    }
    return true;
}
