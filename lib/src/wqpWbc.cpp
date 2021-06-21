/**
 * @file WqpWbc.cpp
 * @brief Function implementation part of class WqpWbc
 * @author Jiajun Wang
 * @date 2020-08-27
 * @version alpha
 * @copyright Copyright (c) 2015-2020 UBT-Beijing
 */

#include "wqpWbc.h"

// ======================================== public Functions ====================================================

WqpWbc::WqpWbc(int dimVar, RobotDynamics * roDy):Wbc(dimVar, roDy){
    createNewQP();
    resizeQPMatrixVector();
}

WqpWbc::WqpWbc(const Wbc & wbc_foo):Wbc(wbc_foo){
    createNewQP();
    resizeQPMatrixVector();
}

// ---------------------- rewrite virtual functions ------------------------

WqpWbc::~WqpWbc(){
    delete QP;
    delete _cputime;
    delete [] _primal_Opt;
    delete [] _dual_Opt;
    QP = nullptr;
    _cputime = nullptr;
    _primal_Opt = nullptr;
    _dual_Opt = nullptr;
}

bool WqpWbc::wbcInit(){
    delete QP;
    delete _cputime;
    delete [] _primal_Opt;
    delete [] _dual_Opt;
    QP = nullptr;
    _cputime = nullptr;
    _primal_Opt = nullptr;
    _dual_Opt = nullptr;

    createNewQP();
    resizeQPMatrixVector();
    return true;
}

bool WqpWbc::displayResultInformation() const{

    std::cout << "Numbers: " << std::endl
              << "#################   qpOASES  --  QP Basic Numbers   #################" << std::endl
              << "number of variables :                                     " << QP->getNV() << std::endl
              << "number of free variables :                                " << QP->getNFR() << std::endl
              << "number of fixed variables :                               " << QP->getNFX() << std::endl
              << "number of constraints :                                   " << QP->getNC() << std::endl
              << "number of (implicitly defined) equality constraints :     " << QP->getNEC() << std::endl
              << "number of active constraints :                            " << QP->getNAC() << std::endl
              << "number of inactive constraints :                          " << QP->getNIAC() << std::endl;

    // If use the follow, set qpOption.printLevel not to be PL_NONE !
    std::cout << "Properties : " << qpOASES::MessageHandling::getErrorCodeMessage(QP->printProperties()) << std::endl;
    std::cout << "Options : " << qpOASES::MessageHandling::getErrorCodeMessage(QP->printOptions()) << std::endl;

    return true;
}

bool WqpWbc::setParametersInt(const std::vector<int> &Parameters){
    if (Parameters.empty()){
        return false;
    }else{
        nWSR_des = Parameters.at(0);
    }
    return true;
}

bool WqpWbc::setParametersDouble(const std::vector<double> &Parameters){
    if (Parameters.empty()){
        return false;
    }else{
        cputime_des = Parameters.at(0);
    }
    return true;
}

bool WqpWbc::getAuxiliaryDataInt(std::vector<int> &auxiliaryData){
    auxiliaryData.resize(2);
    getNwsr(auxiliaryData.at(0));
    getSimpleStatusInt(auxiliaryData.at(1));
    return true;
}

bool WqpWbc::getAuxiliaryDataDouble(std::vector<double> &auxiliaryData){
    auxiliaryData.resize(2);
    getOptCost(auxiliaryData.at(0));
    auxiliaryData.at(1) = static_cast<double>(* _cputime);
    return true;
}

// ---------------------- implement pure virtual functions -----------------

bool WqpWbc::updateBound(const Eigen::VectorXd &lb, const Eigen::VectorXd &ub){
    if(check(lb, nV)){
        boundVec_All_lb = lb;
    }else{
        return false;
    }
    if(check(ub, nV)){
        boundVec_All_ub = ub;
    }else{
        return false;
    }
    return true;
}

bool WqpWbc::updateRobotDynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &qDot){
    // Provide opportunities for multi-threaded computing
    if (!robot->isCalcWbcDependenceDone()){
        robot->setQ_Qdot(q, qDot);
        robot->calcWbcDependence();
    }
    return true;
}

bool WqpWbc::wbcSolve(){
    qpUpdate();     // update QP solver
    qpSolve();      // solve QP
    return true;
}

bool WqpWbc::getResultOpt(Eigen::VectorXd &resultOpt){
    getPrimalOpt(resultOpt);
    return true;
}


// ======================================== private Functions ====================================================

bool WqpWbc::createNewQP(){

//    qpOption.setToFast();   // It will cause a decrease in accuracy, so choose carefully.
    qpOption.printLevel = qpOASES::PL_NONE;  // PL_MEDIUM, by default
//    qpOption.initialStatusBounds = ST_INACTIVE; // ST_LOWER, by default

    QP = new qpOASES::SQProblem(nV, nC);
    QP->setOptions(qpOption);

    _cputime = new double(cputime_des);

    _primal_Opt = new qpOASES::real_t[nV];
    _dual_Opt = new qpOASES::real_t[nV + nC];

    nO_change = false;
    nC_change = false;
    nV_change = false;

    init_done = false;

    return true;
}

bool WqpWbc::resizeQPMatrixVector(){
    // ---------------------------- Costs/Objects --------------------------------------
    // Hessian matrix
    HessianMat = Eigen::MatrixXd::Zero(nV,nV);                                 // nV*nV
    // gradient vector
    gradientVec = Eigen::VectorXd::Zero(nV);                                   // nV*1
    // omega
    omegaVec_All = Eigen::VectorXd::Zero(nO);                                  // nO*1
    // A
    TaskMat_All = Eigen::MatrixXd::Zero(nO,nV);                                // nO*nV
    // b
    tgtVec_All = Eigen::VectorXd::Zero(nO);                                    // nO*1
    // ------------------------ Bounds & Constraints -----------------------------------
    // lb,ub
    boundVec_All_lb = Eigen::VectorXd::Zero(nV);                               // nV*1
    boundVec_All_ub = Eigen::VectorXd::Zero(nV);                               // nV*1
    // C
    CstrMat_All = Eigen::MatrixXd::Zero(nC,nV);                                // nC*nV
    CstrMat_All_T = Eigen::MatrixXd::Zero(nV,nC);                              // nV*nC
    // lbC,ubC
    cstrVec_All_lb = Eigen::VectorXd::Zero(nC);                                // nC*1
    cstrVec_All_ub = Eigen::VectorXd::Zero(nC);                                // nC*1

    return true;
}

bool WqpWbc::qpUpdate(){
    if(nC_change || nV_change){
        delete QP;
        delete _cputime;
        delete [] _primal_Opt;
        delete [] _dual_Opt;
        QP = nullptr;
        _cputime = nullptr;
        _primal_Opt = nullptr;
        _dual_Opt = nullptr;

        createNewQP();
        resizeQPMatrixVector();
    }
    calcHessianGradient();          // tasks
    calcConstraintCoefficient();    // constraints
    return true;
}

bool WqpWbc::calcHessianGradient(){
    int startRow_task = 0;
    for(auto level : priorityTaskNames){
        for(auto item : level){
            auto iter = tasks.find(item);
            TaskMat_All.block(startRow_task, 0, iter->second->dim, nV) = iter->second->A;
            tgtVec_All.segment(startRow_task, iter->second->dim) = iter->second->b;
            omegaVec_All.segment(startRow_task, iter->second->dim) = iter->second->w;
            startRow_task += iter->second->dim;
        }
    }
    TaskMat_All = omegaVec_All.asDiagonal() * TaskMat_All;
    tgtVec_All = omegaVec_All.asDiagonal() * tgtVec_All;
    // Hessian matrix, H = A^T * A
    HessianMat = TaskMat_All.transpose() * TaskMat_All;
    // gradient vector, g = - A^T * b
    gradientVec = - TaskMat_All.transpose() * tgtVec_All;
    return true;
}

bool WqpWbc::calcConstraintCoefficient(){
    int startRow_constraint = 0;
    for(auto level : priorityConstraintNames){
        for(auto item : level){
            auto iter = constraints.find(item);
            CstrMat_All.block(startRow_constraint, 0, iter->second->dim, nV) = iter->second->C;
            cstrVec_All_lb.segment(startRow_constraint, iter->second->dim) = iter->second->lbC;
            cstrVec_All_ub.segment(startRow_constraint, iter->second->dim) = iter->second->ubC;
            startRow_constraint += iter->second->dim;
        }
    }
    // the transpose of CstrMat_All
    CstrMat_All_T = CstrMat_All.transpose();
    return true;
}

bool WqpWbc::qpSolve(){
    if(!init_done){
        nWSR = nWSR_des;
        * _cputime = cputime_des;
        statusCode_solving = QP->init(HessianMat.data(), gradientVec.data(),
                                      CstrMat_All_T.data(),
                                      boundVec_All_lb.data(), boundVec_All_ub.data(),
                                      cstrVec_All_lb.data(), cstrVec_All_ub.data(),
                                      nWSR, _cputime);
        if(statusCode_solving > 0){
            std::cout << "init QP : " << qpOASES::MessageHandling::getErrorCodeMessage(statusCode_solving) << std::endl;
            qpReset();
        }else {
            init_done = true;
        }
    }else {
        nWSR = nWSR_des;
        * _cputime = cputime_des;
        statusCode_solving = QP->hotstart(HessianMat.data(), gradientVec.data(),
                                          CstrMat_All_T.data(),
                                          boundVec_All_lb.data(), boundVec_All_ub.data(),
                                          cstrVec_All_lb.data(), cstrVec_All_ub.data(),
                                          nWSR, _cputime);
        if(statusCode_solving > 0){
            std::cout << "hotstart QP : " << qpOASES::MessageHandling::getErrorCodeMessage(statusCode_solving) << std::endl;
            qpReset();
        }
    }
    return true;
}

bool WqpWbc::qpReset(){
    QP->reset();
    init_done = false;
    return true;
}

bool WqpWbc::getPrimalOpt(Eigen::VectorXd & primalOpt){
    if (primalOpt.size() < nV){
        primalOpt.resize(nV);
    }
    QP->getPrimalSolution(_primal_Opt);
    for(int i = 0; i != nV; i++)
    {
        primalOpt(i) = static_cast<double>(_primal_Opt[i]);
    }
    return true;
}

bool WqpWbc::getDualOpt(Eigen::VectorXd & dualOpt){
    if (dualOpt.size() < nV + nC){
        dualOpt.resize(nV + nC);
    }
    QP->getDualSolution(_dual_Opt);
    for(int i = 0; i != nV + nC; i++)
    {
        dualOpt(i) = static_cast<double>(_dual_Opt[i]);
    }
    return true;
}

bool WqpWbc::getNwsr(int & nWSR_res){
    nWSR_res = static_cast<int>(nWSR);
    return true;
}

bool WqpWbc::getSimpleStatusInt(int & simpleStatus){
    if (init_done){
        simpleStatus = 0;
    }else{
        simpleStatus = 1;
    }
    return true;
}

bool WqpWbc::getOptCost(double & cost_opt){
    cost_opt = static_cast<double>(QP->getObjVal());
    return true;
}
