/**
 * @file Constraint.cpp
 * @brief Function implementation part of class Constraint
 * @author Jiajun Wang
 * @date 2020-09-01
 * @version alpha
 * @copyright Copyright (c) 2015-2020 UBT-Beijing
 */

#include "constraint.h"

Constraint::Constraint(const std::string & constrName, int constrDim, int varDim){
    name = constrName;
    dim = constrDim;
    varDof = varDim;
    C = Eigen::MatrixXd::Zero(dim, varDof);
    lbC = Eigen::VectorXd::Zero(dim);
    ubC = Eigen::VectorXd::Zero(dim);
}

bool Constraint::setParameter(const std::vector<double> & params){
    std::cout << "Nothing to be done, you should rewrite this function, if you want to use it."<< std::endl;
    return true;
}

bool Constraint::check(const Eigen::MatrixXd & M, int row, int col){
    if (M.rows() != row || M.cols() != col){
        std::cout << "Error : Matrix dimensions do not match!" << std::endl;
        return false;
    }
    return true;
}

bool Constraint::check(const Eigen::VectorXd & v, int row){
    if (v.rows() != row){
        std::cout << "Error : Vector dimensions do not match!" << std::endl;
        return false;
    }
    return true;
}
