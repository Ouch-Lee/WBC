/**
 * @file NspWbc.cpp
 * @brief Function implementation part of class NspWbc
 * @author Yan Xie
 * @date 2020-09-17
 * @version alpha
 * @copyright Copyright (c) 2015-2020 UBT-Beijing
 */

#include "nspWbc.h"

// ======================================== public Functions ====================================================

NspWbc::NspWbc(int dimVar, RobotDynamics * roDy):Wbc(dimVar, roDy){
    createNewQP();
    resizeQPMatrixVector();
}

NspWbc::NspWbc(const Wbc & wbc_foo):Wbc(wbc_foo){
    createNewQP();
    resizeQPMatrixVector();
}

// ---------------------- rewrite virtual functions ------------------------

NspWbc::~NspWbc(){
    for (int i = 0; i != nLevel; i++) {
        delete QP.at(i);    
        delete _cputime.at(i);
        delete [] _primal_Opt.at(i);
        delete [] _dual_Opt.at(i);
        QP.at(i) = nullptr;
        _cputime.at(i) = nullptr;
        _primal_Opt.at(i) = nullptr;
        _dual_Opt.at(i) = nullptr;
    }
}

bool NspWbc::wbcInit(){
    createNewQP();
    resizeQPMatrixVector();
    return true;
}

bool NspWbc::displayResultInformation() const{
    for (int i = 0; i != nLevel; i++){
        std::cout << "-------------------------   Level " << i << " Information   ----------------------" << std::endl;
        std::cout << "Numbers: " << std::endl
                << "#################   qpOASES  --  QP Basic Numbers   #################" << std::endl
                << "number of variables :                                     " << QP.at(i)->getNV() << std::endl
                << "number of free variables :                                " << QP.at(i)->getNFR() << std::endl
                << "number of fixed variables :                               " << QP.at(i)->getNFX() << std::endl
                << "number of constraints :                                   " << QP.at(i)->getNC() << std::endl
                << "number of (implicitly defined) equality constraints :     " << QP.at(i)->getNEC() << std::endl
                << "number of active constraints :                            " << QP.at(i)->getNAC() << std::endl
                << "number of inactive constraints :                          " << QP.at(i)->getNIAC() << std::endl;

        /// If use the follow, set qpOption.printLevel not to be PL_NONE !
        std::cout << "Properties : " << qpOASES::MessageHandling::getErrorCodeMessage(QP.at(i)->printProperties()) << std::endl;
        std::cout << "Options : " << qpOASES::MessageHandling::getErrorCodeMessage(QP.at(i)->printOptions()) << std::endl;
    }
    return true;
}


bool NspWbc::setParametersInt(const std::vector<int> &Parameters){
    if (Parameters.empty() || Parameters.size() < static_cast<unsigned int>(nLevel)){
        std::cout << "Error : Vector dimensions do not match!" << std::endl;
        return false;
    }else{
        for (int i = 0; i != nLevel; i++){
            nWSR_des.at(i) = Parameters.at(0);
        }
    }
    return true;
}

bool NspWbc::setParametersDouble(const std::vector<double> &Parameters){
    if (Parameters.empty() || Parameters.size() < static_cast<unsigned int>(nLevel)){
        std::cout << "Error : Vector dimensions do not match!" << std::endl;
        return false;
    }else{
        for (int i = 0; i != nLevel; i++){
            cputime_des.at(i) = Parameters.at(0);
        }
    }
    return true;
}

bool NspWbc::getAuxiliaryDataInt(std::vector<int> &auxiliaryData){
    // TODO:
    auxiliaryData.resize(2 * nLevel);
    std::vector<int> NwsrData(nLevel, -1);
    std::vector<int> SimpleStatusData(nLevel, -1);
    getNwsr(NwsrData);
    getSimpleStatusInt(SimpleStatusData);
    for (int i = 0; i != nLevel; i++){
        auxiliaryData.at(i) = NwsrData.at(i);
        auxiliaryData.at(i + nLevel) = SimpleStatusData.at(i);
    }
    return true;
}

bool NspWbc::getAuxiliaryDataDouble(std::vector<double> &auxiliaryData){
    // TODO:
    auxiliaryData.resize(2 * nLevel);
    std::vector<double> CostData(nLevel, -1);
    getOptCost(CostData);
    for (int i = 0; i != nLevel; i++){
        auxiliaryData.at(i) = CostData.at(i);
        auxiliaryData.at(i + nLevel) = static_cast<double>(* _cputime.at(i));
    }
    return true;
}

// ---------------------- implement pure virtual functions -----------------

bool NspWbc::updateBound(const Eigen::VectorXd &lb, const Eigen::VectorXd &ub){
    // TODO:
    for (int i = 0; i != nLevel; i++){
        if(check(lb, nVLevel.at(i))){
            boundVec_All_lb.at(i) = lb;
        }else{
            return false;
        }
        if(check(ub, nVLevel.at(i))){
            boundVec_All_ub.at(i) = ub;
        }else{
            return false;
        }
    }
    return true;
}

bool NspWbc::updateRobotDynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &qDot){
    /// Provide opportunities for multi-threaded computing
    if (!robot->isCalcWbcDependenceDone()){
        robot->setQ_Qdot(q, qDot);
        robot->calcWbcDependence();
    }
    return true;
}

bool NspWbc::wbcSolve(){
    // TODO:
    nspUpdate();     // update QP solver
    nspSolve();
    for (int i = 0; i !=nLevel; i++){
        if (nOLevel.at(i)  == 0){
            if ( i == 0){        
                x_star.at(i) = Eigen::VectorXd::Zero(nVLevel.at(i));
            }else{
                x_star.at(i) = x_star.at(i - 1);
            }
        }else{
            nspUpdateLeveln(i);
            nspSolveLeveln(i);      // solve QP
            getResultOptLeveln(x_star.at(i), i);
            
            if ( i != 0){        
                x_star.at(i) = x_star.at(i - 1) + Z_Mat.at(i - 1) *  x_star.at(i);
            }           
        }
    }
    return true;
}

bool NspWbc::getResultOpt(Eigen::VectorXd &resultOpt){
    // TODO:
   resultOpt = x_star.at(nLevel-1);
    return true;
}


// ======================================== private Functions ====================================================

bool NspWbc::createNewQP(){

    nLevelTask = priorityTaskNames.size();
    nLevelConstraint = priorityConstraintNames.size();
    nLevel = (nLevelTask > nLevelConstraint) ? nLevelTask : nLevelConstraint;

    std::cout << "nLevelTask = " << nLevelTask << std::endl
    << "nLevelConstraint = " << nLevelConstraint << std::endl
    << "nLevel = " << nLevel << std::endl;

    nVLevel.resize(nLevel);
    nOLevel.resize(nLevel);
    nCLevel.resize(nLevel);

    int iLevel = 0;
    for(auto level : priorityTaskNames){
        for(auto item : level){
            auto iter = tasks.find(item);
            nVLevel.at(iLevel) = iter->second->varDof;
            nOLevel.at(iLevel) += iter->second->dim;
        }
    iLevel++;
    }    

    iLevel = 0;
    for(auto level : priorityConstraintNames){
        for(auto item : level){
            auto iter = constraints.find(item);
            nCLevel.at(iLevel) += iter->second->dim;
        }
    iLevel++;
    }    

    std::cout << "nVLevel = "  << std::endl;
    for (auto item : nVLevel){
        std::cout << item << "\t";
    }
    std::cout << std::endl;
    std::cout << "nOLevel = "  << std::endl;
    for (auto item : nOLevel){
        std::cout << item << "\t";
    }
    std::cout << std::endl;
    std::cout << "nCLevel = "  << std::endl;
    for (auto item : nCLevel){
        std::cout << item << "\t";
    }
    std::cout << std::endl;

    QP.resize(nLevel);
    qpOption.resize(nLevel);

    nWSR.resize(nLevel);
    _cputime.resize(nLevel);

    _primal_Opt.resize(nLevel);
    _dual_Opt.resize(nLevel);

    statusCode_solving.resize(nLevel);
    statusCode_getSolution.resize(nLevel);

    init_done.resize(nLevel);
    nWSR_des.resize(nLevel);
    cputime_des.resize(nLevel);

    int sum_nCLevel = 0;
    for (int i = 0; i != nLevel; i++){
        if (nVLevel.at(i) == 0){
            nVLevel.at(i) = *max_element(nVLevel.begin(),nVLevel.end()); 
            std::cout << "Warning : QP empty in level " << i << "!" << std::endl;
        } 

        sum_nCLevel += nCLevel.at(i);
       
        qpOption.at(i).setToFast();
        qpOption.at(i).printLevel = qpOASES::PL_NONE;
        qpOption.at(i).terminationTolerance = 0.00001;
        qpOption.at(i).enableRegularisation = qpOASES::BT_TRUE;

        QP.at(i) = new qpOASES::SQProblem(nVLevel.at(i), sum_nCLevel);
        QP.at(i)->setOptions(qpOption.at(i));

        _primal_Opt.at(i) = new qpOASES::real_t[nVLevel.at(i)];
        _dual_Opt.at(i) = new qpOASES::real_t[nVLevel.at(i) + sum_nCLevel];

        init_done.at(i) = false;
        nWSR_des.at(i) = 100;
        cputime_des.at(i) = 10;

         nWSR.at(i) = nWSR_des.at(i);
        _cputime.at(i) = new double(cputime_des.at(i));       
    }

    return true;
}


bool NspWbc::resizeQPMatrixVector(){
    // ---------------------------- Costs/Objects --------------------------------------
    HessianMat.resize(nLevel);
    gradientVec.resize(nLevel);
    omegaVec_All.resize(nLevel);
    TaskMat_All.resize(nLevel);
    tgtVec_All.resize(nLevel);

    boundVec_All_lb.resize(nLevel);
    boundVec_All_ub.resize(nLevel);
    CstrMat_All.resize(nLevel);
    CstrMat_All_T.resize(nLevel);
    cstrVec_All_lb.resize(nLevel);
    cstrVec_All_ub.resize(nLevel);

    Z_Mat.resize(nLevel);
    x_star.resize(nLevel);

    omegaVec_Level.resize(nLevel);
    TaskMat_Level.resize(nLevel);
    TaskMatHat_Level.resize(nLevel);
    TaskMatHatInv_Level.resize(nLevel);
    tgtVec_Level.resize(nLevel);

    boundVec_Level_lb.resize(nLevel);
    boundVec_Level_ub.resize(nLevel);
    CstrMat_Level.resize(nLevel);
    CstrMat_Level_T.resize(nLevel);
    cstrVec_Level_lb.resize(nLevel);
    cstrVec_Level_ub.resize(nLevel);


    int sum_nCLevel = 0;
    for (int i = 0; i != nLevel; i++){
        sum_nCLevel += nCLevel.at(i);
        /// Hessian matrix
        HessianMat.at(i) = Eigen::MatrixXd::Zero(nVLevel.at(i),nVLevel.at(i));                                 ///< nVi*nVi
        /// gradient vector
        gradientVec.at(i) = Eigen::VectorXd::Zero(nVLevel.at(i));                                   ///< nVi*1
        /// omega
        omegaVec_All.at(i) = Eigen::VectorXd::Zero(nOLevel.at(i));                                  ///< nOi*1
        /// A
        TaskMat_All.at(i) = Eigen::MatrixXd::Zero(nOLevel.at(i),nVLevel.at(i));                                ///< nOi*nVi
        /// b
        tgtVec_All.at(i) = Eigen::VectorXd::Zero(nOLevel.at(i));                                    ///< nOi*1

        // ------------------------ Bounds & Constraints -----------------------------------
        /// lb,ub
        boundVec_All_lb.at(i) = Eigen::VectorXd::Zero(nVLevel.at(i));                               ///< nVi*1
        boundVec_All_ub.at(i) = Eigen::VectorXd::Zero(nVLevel.at(i));                               ///< nVi*1
        /// C
        CstrMat_All.at(i) = Eigen::MatrixXd::Zero(sum_nCLevel,nVLevel.at(i));                                ///< (nC0 + ... +nCi)*nVi
        CstrMat_All_T.at(i) = Eigen::MatrixXd::Zero(nVLevel.at(i),sum_nCLevel);                              ///< nVi*(nC0 + ... +nCi)
        /// lbC,ubC
        cstrVec_All_lb.at(i) = Eigen::VectorXd::Zero(sum_nCLevel);                                ///< (nC0 + ... +nCi)*1
        cstrVec_All_ub.at(i) = Eigen::VectorXd::Zero(sum_nCLevel);                                ///< (nC0 + ... +nCi)*1

        // ------------------------ Bounds & Constraints -----------------------------------
        /// Z, X*
        Z_Mat.at(i) = Eigen::MatrixXd::Zero(nVLevel.at(i), nVLevel.at(i));                                  ///< nVi*nVi
        x_star.at(i) = Eigen::VectorXd::Zero(nVLevel.at(i));                                                              ///< nVi*1
        /// omega_level
        omegaVec_Level.at(i) = Eigen::VectorXd::Ones(nOLevel.at(i));                                  ///< nOi*1
        /// A_level
        TaskMat_Level.at(i) = Eigen::MatrixXd::Zero(nOLevel.at(i),nVLevel.at(i));                                ///< nOi*nVi
        /// AHat_level
        TaskMatHat_Level.at(i) = Eigen::MatrixXd::Zero(nOLevel.at(i),nVLevel.at(i));;                                    ///< nOi*nVi
        /// AHatInv_level
        TaskMatHatInv_Level.at(i) = Eigen::MatrixXd::Zero(nVLevel.at(i),nOLevel.at(i));                                ///< nVi*nOi
        /// b_level
        tgtVec_Level.at(i) = Eigen::VectorXd::Zero(nOLevel.at(i));                                    ///< nOi*1
        /// lb_level,ub_level
        boundVec_Level_lb.at(i) = Eigen::VectorXd::Zero(nVLevel.at(i));                               ///< nVi*1
        boundVec_Level_ub.at(i) = Eigen::VectorXd::Zero(nVLevel.at(i));                               ///< nVi*1
        /// C_level
        CstrMat_Level.at(i) = Eigen::MatrixXd::Zero(nCLevel.at(i),nVLevel.at(i));                                ///< nCi*nVi
        CstrMat_Level_T.at(i) = Eigen::MatrixXd::Zero(nVLevel.at(i),nCLevel.at(i));                              ///< nVi*nCi
        /// lbC_level,ubC_level
        cstrVec_Level_lb.at(i) = Eigen::VectorXd::Zero(nCLevel.at(i));                                ///< nCi*1
        cstrVec_Level_ub.at(i) = Eigen::VectorXd::Zero(nCLevel.at(i));                                ///< nCi*1
    }

    return true;
}

bool NspWbc::nspUpdate(){
    if(nC_change || nV_change){
        // TODO:
        // createNewQP();
        // resizeQPMatrixVector();
    }
    calcHessianGradient();          ///< tasks
    calcConstraintCoefficient();    ///< constraints
    return true;
}

bool NspWbc::calcHessianGradient(){
    int startRow_task = 0;
    int iLevel = 0;
    
    for(auto level : priorityTaskNames){
        for(auto item : level){
            auto iter = tasks.find(item);
            TaskMat_Level.at(iLevel).block(startRow_task, 0, iter->second->dim, nVLevel.at(iLevel)) = iter->second->A;
            tgtVec_Level.at(iLevel).segment(startRow_task, iter->second->dim) = iter->second->b;
            omegaVec_Level.at(iLevel).segment(startRow_task, iter->second->dim) = iter->second->w;
            startRow_task += iter->second->dim;            
        }
        startRow_task = 0;
        iLevel++;
    }
    return true;
}

bool NspWbc::calcConstraintCoefficient(){
    int startRow_constraint = 0;
    int iLevel = 0;
    for(auto level : priorityConstraintNames){
        for(auto item : level){
            auto iter = constraints.find(item);
            CstrMat_Level.at(iLevel).block(startRow_constraint, 0, iter->second->dim, nVLevel.at(iLevel)) = iter->second->C;
            cstrVec_Level_lb.at(iLevel).segment(startRow_constraint, iter->second->dim) = iter->second->lbC;
            cstrVec_Level_ub.at(iLevel).segment(startRow_constraint, iter->second->dim) = iter->second->ubC;
            startRow_constraint += iter->second->dim;
        }
        startRow_constraint = 0;
        iLevel++;
    }
    return true;
}

bool NspWbc::nspSolve(){
    for (int i = 0; i != nLevel; i++){
        if (nOLevel.at(i) == 0){
            if (i == 0){
                Z_Mat.at(i) = Eigen::MatrixXd::Identity(nVLevel.at(i), nVLevel.at(i));
            }else{
                Z_Mat.at(i) = Z_Mat.at(i-1);
            }
        }else{
            if (i == 0){
                TaskMatHatInv_Level.at(i) = TaskMat_Level.at(i).transpose() * (TaskMat_Level.at(i) * TaskMat_Level.at(i).transpose()).inverse();
                Z_Mat.at(i) = Eigen::MatrixXd::Identity(nVLevel.at(i), nVLevel.at(i)) - TaskMatHatInv_Level.at(i) * TaskMat_Level.at(i);
            }else{
                TaskMatHat_Level.at(i) = TaskMat_Level.at(i) * Z_Mat.at(i - 1);
                TaskMatHatInv_Level.at(i) =  TaskMatHat_Level.at(i).transpose() * (TaskMatHat_Level.at(i) * TaskMatHat_Level.at(i).transpose()).inverse();
                Z_Mat.at(i) = Z_Mat.at(i -1) * (Eigen::MatrixXd::Identity(nVLevel.at(i), nVLevel.at(i)) - TaskMatHatInv_Level.at(i) * TaskMatHat_Level.at(i));
            }
        }

    }
    return true;
}

bool NspWbc::nspUpdateLeveln(const int & iLevel){
    // ---------------------------- Costs/Objects --------------------------------------
    if (iLevel == 0){
        TaskMat_All.at(iLevel) = TaskMat_Level.at(iLevel);
        tgtVec_All.at(iLevel) = tgtVec_Level.at(iLevel);
    }else{
        TaskMat_All.at(iLevel) = TaskMat_Level.at(iLevel) * Z_Mat.at(iLevel -1);
        tgtVec_All.at(iLevel) = tgtVec_Level.at(iLevel) - TaskMat_Level.at(iLevel) * x_star.at(iLevel -1);
    }

    omegaVec_All.at(iLevel) = omegaVec_Level.at(iLevel);
    TaskMat_All.at(iLevel) = omegaVec_All.at(iLevel).asDiagonal() * TaskMat_All.at(iLevel);
    tgtVec_All.at(iLevel) = omegaVec_All.at(iLevel).asDiagonal() * tgtVec_All.at(iLevel);  
         
    /// Hessian matrix, H = A^T * A
    HessianMat.at(iLevel) = TaskMat_All.at(iLevel).transpose() * TaskMat_All.at(iLevel);
    /// gradient vector, g = - A^T * b
    gradientVec.at(iLevel) = - TaskMat_All.at(iLevel).transpose() * tgtVec_All.at(iLevel);

    // ------------------------ Bounds & Constraints -----------------------------------
    int  startRow_constraint = 0;
    for (int i = 0; i != iLevel + 1; i++){
        CstrMat_All.at(iLevel).block(startRow_constraint, 0, nCLevel.at(i), nVLevel.at(i)) = CstrMat_Level.at(i);
        cstrVec_All_lb.at(iLevel).segment(startRow_constraint, nCLevel.at(i)) = cstrVec_Level_lb.at(i);
        cstrVec_All_ub.at(iLevel).segment(startRow_constraint, nCLevel.at(i)) = cstrVec_Level_ub.at(i);
        startRow_constraint += nCLevel.at(i);
    }

    if (iLevel == 0){
        cstrVec_All_lb.at(iLevel) = cstrVec_All_lb.at(iLevel);
        cstrVec_All_ub.at(iLevel) = cstrVec_All_ub.at(iLevel);
        CstrMat_All.at(iLevel)  = CstrMat_All.at(iLevel);
    }else{
        cstrVec_All_lb.at(iLevel) = cstrVec_All_lb.at(iLevel) - CstrMat_All.at(iLevel) * x_star.at(iLevel - 1);
        cstrVec_All_ub.at(iLevel) = cstrVec_All_ub.at(iLevel) - CstrMat_All.at(iLevel) * x_star.at(iLevel - 1);
        CstrMat_All.at(iLevel)  = CstrMat_All.at(iLevel) * Z_Mat.at(iLevel -1);
    }
    /// the transpose of CstrMat_All
    CstrMat_All_T.at(iLevel) = CstrMat_All.at(iLevel).transpose();
    return true;
}


bool NspWbc::nspSolveLeveln(const int & iLevel){
    if(!init_done.at(iLevel)){
        nWSR.at(iLevel) = nWSR_des.at(iLevel);
        * _cputime.at(iLevel) = cputime_des.at(iLevel);
        statusCode_solving.at(iLevel) = QP.at(iLevel)->init(HessianMat.at(iLevel).data(), gradientVec.at(iLevel).data(),
                                      CstrMat_All_T.at(iLevel).data(),
                                      boundVec_All_lb.at(iLevel).data(), boundVec_All_ub.at(iLevel).data(),
                                      cstrVec_All_lb.at(iLevel).data(), cstrVec_All_ub.at(iLevel).data(),
                                      nWSR.at(iLevel), _cputime.at(iLevel));
        if(statusCode_solving.at(iLevel) > 0){
            std::cout << "init QP : " << qpOASES::MessageHandling::getErrorCodeMessage(statusCode_solving.at(iLevel)) << std::endl;
            qpResetLeveln(iLevel);
        }else {
            init_done.at(iLevel) = true;
        }
    }else {
        nWSR.at(iLevel) = nWSR_des.at(iLevel);
        * _cputime.at(iLevel) = cputime_des.at(iLevel);
        statusCode_solving.at(iLevel) = QP.at(iLevel)->hotstart(HessianMat.at(iLevel).data(), gradientVec.at(iLevel).data(),
                                          CstrMat_All_T.at(iLevel).data(),
                                          boundVec_All_lb.at(iLevel).data(), boundVec_All_ub.at(iLevel).data(),
                                          cstrVec_All_lb.at(iLevel).data(), cstrVec_All_ub.at(iLevel).data(),
                                          nWSR.at(iLevel), _cputime.at(iLevel));
        if(statusCode_solving.at(iLevel) > 0){
            std::cout << "hotstart QP : " << qpOASES::MessageHandling::getErrorCodeMessage(statusCode_solving.at(iLevel)) << std::endl;
            qpResetLeveln(iLevel);
        }
    }
    return true;
}

bool NspWbc::qpResetLeveln(const int & iLevel){
    QP.at(iLevel)->reset();
    init_done.at(iLevel) = false;

    return true;
}

bool NspWbc::getPrimalOptLeveln(Eigen::VectorXd & primalOpt, const int & iLevel){
    if (primalOpt.size() < nVLevel.at(iLevel)){
        primalOpt.resize(nVLevel.at(iLevel));
    }
    QP.at(iLevel)->getPrimalSolution(_primal_Opt.at(iLevel));
    for(int i = 0; i != nVLevel.at(iLevel); i++)
    {
        primalOpt(i) = static_cast<double>(_primal_Opt.at(iLevel)[i]);
    }
    return true;
}

bool NspWbc::getResultOptLeveln(Eigen::VectorXd &resultOpt, const int & iLevel){
    // TODO:
    getPrimalOptLeveln(resultOpt, iLevel);
    return true;
}


bool NspWbc::getNwsr(std::vector<int> & nWSR_res){
    for (int i = 0; i != nLevel; i++){
        nWSR_res.at(i) = static_cast<int>(nWSR.at(i));
    }
    return true;
}

bool NspWbc::getSimpleStatusInt(std::vector<int> & simpleStatus){
    for (int i = 0; i != nLevel; i++){
        if (init_done.at(i)){
            simpleStatus.at(i)= 0;
        }else{
            simpleStatus.at(i) = 1;
        }
    }
    return true;
}

bool NspWbc::getOptCost(std::vector<double> & cost_opt){
    for (int i = 0; i != nLevel; i++){
        cost_opt.at(i) = static_cast<double>(QP.at(i)->getObjVal()) + 0.5 * tgtVec_All.at(i).transpose() * tgtVec_All.at(i);
    }
    return true;
}
