/**
 * @file WqpWbc.cpp
 * @brief Function implementation part of class WqpWbc
 * @author Jiajun Wang
 * @date 2020-08-27
 * @version alpha
 * @copyright Copyright (c) 2015-2020 UBT-Beijing
 */

#include "wqpWbc.h"
using namespace std;
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

    // //Jacobian
    J1 = Eigen::MatrixXd::Zero(3,nV);  
    J2 = Eigen::MatrixXd::Zero(3,nV);  
    J3 = Eigen::MatrixXd::Zero(4,nV);  
    J4 = Eigen::MatrixXd::Zero(4,nV);  
    J5 = Eigen::MatrixXd::Zero(4,nV);  
    J6 = Eigen::MatrixXd::Zero(4,nV);  
    J7 = Eigen::MatrixXd::Zero(4,nV);  

    J_level1 =Eigen::MatrixXd::Zero(4,nV);  
    J_level2 =Eigen::MatrixXd::Zero(4,nV);  
    J_level3 =Eigen::MatrixXd::Zero(3,nV);  
    J_level4 =Eigen::MatrixXd::Zero(15,nV);  

    J_level1_pinv =Eigen::MatrixXd::Zero(nV,4);  
    J_level2_pinv =Eigen::MatrixXd::Zero(nV,4);  
    J_level3_pinv =Eigen::MatrixXd::Zero(nV,3);  
    J_level4_pinv =Eigen::MatrixXd::Zero(nV,15);  
    // maniputation variable
    R1 =Eigen::VectorXd::Zero(3);
    R2 =Eigen::VectorXd::Zero(3);
    R3 =Eigen::VectorXd::Zero(4);
    R4 =Eigen::VectorXd::Zero(4);
    R5 =Eigen::VectorXd::Zero(4);
    R6 =Eigen::VectorXd::Zero(4);
    R7 =Eigen::VectorXd::Zero(4);

    R_level1 =Eigen::VectorXd::Zero(4);
    R_level2 =Eigen::VectorXd::Zero(4);
    R_level3 =Eigen::VectorXd::Zero(3);
    R_level4 =Eigen::VectorXd::Zero(15);

    X =Eigen::VectorXd::Zero(nV);
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
    calcJAndB();
    calc_X();
    calcHessianGradient();          // tasks
    calcConstraintCoefficient();    // constraints
    return true;
}

bool WqpWbc::calcJAndB(){
    int startRow_task = 0;
    int startRow_level4=0;
   int ilevel=0;
    for(auto level : priorityTaskNames){   
        for(auto item : level){  
            auto iter = tasks.find(item);
            TaskMat_All.block(startRow_task, 0, iter->second->dim, nV) = iter->second->A;
            tgtVec_All.segment(startRow_task, iter->second->dim) = iter->second->b;
            omegaVec_All.segment(startRow_task, iter->second->dim) = iter->second->w;
            startRow_task += iter->second->dim;
            // cout  << "the " << ilevel << "th b is:" << iter->second->b << endl;
            // cout << "response tgt_ALL is: " << tgtVec_All << endl;
            switch (ilevel)
            {
            case 0:
                J1 =iter->second->A;
                R1 = iter->second->b;
                // cout<< "the 1th task" << iter->first << endl;
                // cout << "J1 added suscessfully" << endl;
                break;
            case 1:
                J2 =iter->second->A;
                 R2 = iter->second->b;
                // cout << "J2 added suscessfully" << endl;
                // cout << "the size of  R2" << R2.rows() << endl;  3
                break;
             case 2:
                J3 =iter->second->A;
                 R3  = iter->second->b;
                // cout << "J3 added suscessfully" << endl;
                break;
            case 3:
                 J4 =iter->second->A;
                  R4  = iter->second->b;
                //   cout <<"the 4 th task: " << iter ->first <<endl;
                //  cout << "J4 added suscessfully" << endl;
                // cout << "the size of  R4" << R4.rows() << endl;  4
                break;
            case 4:
                 J5 =iter->second->A;
                  R5  = iter->second->b;
                //  cout << "J5 added suscessfully" << endl;
                break;
            case 5:
                 J6 =iter->second->A;
                  R6  = iter->second->b;
                //  cout << "J6 added suscessfully" << endl;
                // cout << "the size of  R6" << R6.rows() << endl;  4
                break; 
            case 6:
                 J7 =iter->second->A;
                  R7  = iter->second->b;
                //   cout << "the 7th task: " <<iter ->first << endl;
                //  cout << "J7 added suscessfully" << endl;
                // cout << "the size of  R7" << R7.rows() << endl;  4
                break;
            default:
                break;
            }
           ilevel++;
    } 
      std::cout << "---------------------------- "<< std::endl;
    }
    J_level1 =J5;
    R_level1 =R5;
    
    J_level2 =J3;
    R_level2 =R3;

    J_level3 = J1;
    R_level3 =R1;

    J_level4.block(startRow_level4,0,J2.rows(),nV)=J2;
    R_level4.segment(startRow_level4,3) =R2;
    startRow_level4 +=J2.rows();
    J_level4.block(startRow_level4,0,J4.rows(),nV)=J4;
    R_level4.segment(startRow_level4,4) =R4;
    startRow_level4 +=J4.rows();
    J_level4.block(startRow_level4,0,J6.rows(),nV)=J6;
    R_level4.segment(startRow_level4,4) =R6;
    startRow_level4 +=J6.rows();
    J_level4.block(startRow_level4,0,J7.rows(),nV)=J7;
    R_level4.segment(startRow_level4,4) =R7;
    startRow_level4 +=J7.rows();
    // cout << "row and cal of J_level4 :"<<J_level4.rows() << " "<< J_level4.cols() << endl;

    J_level1_pinv= pinv_eigen_based(J_level1, 0.0);
    J_level2_pinv= pinv_eigen_based(J_level2, 0.0);
    J_level3_pinv =pinv_eigen_based (J_level3,0.0);
    //  cout << J_level3_pinv << endl;
    J_level4_pinv =pinv_Matirx(J_level4);

    return true;
}

bool WqpWbc::calc_X(){
    Eigen::MatrixXd A2_wave;
    Eigen::MatrixXd A3_wave;
    Eigen::MatrixXd A4_wave;
    Eigen::MatrixXd unitMatrix_15;
    Eigen::VectorXd vec_u;
    Eigen::VectorXd vec_z;
    Eigen::VectorXd vec_y;
    Eigen::VectorXd vec_x;
    
    A2_wave.resize(4,nV);
    A3_wave.resize(3,nV);
    A4_wave.resize(15,nV);
    unitMatrix_15 = Eigen::MatrixXd::Identity(15,15);
    vec_u.resize(nV);
    vec_z.resize(nV);
    vec_y.resize(nV);
    vec_x.resize(nV);
    //J_level is the Ai ; J_level_pinv is the  pseudoJacobian ;R_level is b
    A2_wave =J_level2 *(unitMatrix_15 -J_level1_pinv*J_level1);
    A3_wave = J_level3 * ( unitMatrix_15 -J_level1_pinv*J_level1 ) * (unitMatrix_15 - pinv_eigen_based(A2_wave,0.0)*A2_wave);
    A4_wave = J_level4 * (unitMatrix_15 - J_level1_pinv * J_level1 ) * (unitMatrix_15 - pinv_eigen_based(A2_wave, 0.0) *A2_wave) * (unitMatrix_15 -pinv_eigen_based(A3_wave,0.0)*A3_wave );   //15*15
    
    vec_u = pinv_Matirx(A4_wave) * ( R_level4 - J_level4 * J_level1_pinv *R_level1 -J_level4 *(unitMatrix_15 -J_level1_pinv * J_level1) * pinv_eigen_based(A2_wave, 0.0) 
    *(R_level2 - J_level2 * J_level1_pinv *R_level1 ) -J_level4 *( unitMatrix_15 -J_level1_pinv *J_level1) *( unitMatrix_15 - pinv_eigen_based(A2_wave, 0.0) *A2_wave) *pinv_eigen_based(A3_wave,0.0)
    *(R_level3 - J_level3* J_level1_pinv * R_level1 -J_level3 *(unitMatrix_15 -J_level1_pinv *J_level1) * pinv_eigen_based(A2_wave, 0.0) * (R_level2 - J_level2 * J_level1_pinv * R_level1) ) );
    // cout << vec_u << endl;

    vec_z = pinv_eigen_based(A3_wave, 0.0) * (R_level3 - J_level3* J_level1_pinv * R_level1 -J_level3 * ( unitMatrix_15 -J_level1_pinv* J_level1) * pinv_eigen_based(A2_wave, 0.0) *( R_level2 - J_level2 
     * J_level1_pinv * R_level1) )+ (unitMatrix_15 - pinv_eigen_based(A3_wave, 0.0) * A3_wave)  *vec_u;
    //  cout << vec_z << endl;

    vec_y =pinv_eigen_based(A2_wave, 0.0) * (R_level2 - J_level2* J_level1_pinv * R_level1)+ (unitMatrix_15 - pinv_eigen_based(A2_wave, 0.0) * A2_wave)  *vec_z;
    // cout << vec_z << endl;

    vec_x =J_level1_pinv * R_level1 + (unitMatrix_15 - J_level1_pinv* J_level1)  *vec_y;
    // cout << vec_x << endl;

    X=vec_x;
    cout <<"X: " << X.transpose() << endl;
    return true;

}

bool WqpWbc::calcHessianGradient(){
    Eigen::MatrixXd I_15;
    I_15 =Eigen::MatrixXd::Identity(15,15);

    TaskMat_All = omegaVec_All.asDiagonal() * TaskMat_All;
    tgtVec_All = omegaVec_All.asDiagonal() * tgtVec_All;
    // Hessian matrix, H = A^T * A
     HessianMat = TaskMat_All.transpose() * TaskMat_All;

    // for QP-NSP H=2*I
    // HessianMat = I_15;
    // cout<< "H:" << HessianMat << endl;
    // gradient vector, g = - A^T * b
    gradientVec = - TaskMat_All.transpose() * tgtVec_All;
    // for NSP-QP g= =-X
    // gradientVec =X.transpose();
    // cout << "g:" << X.transpose() << endl;
    return true;
}



bool WqpWbc::calcConstraintCoefficient(){
    int startRow_constraint = 0;
    for(auto level : priorityConstraintNames){    // only 1 col
        for(auto item : level){    // 4 contrait -> 4 rows
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

    Eigen::MatrixXd WqpWbc::pinv_eigen_based(Eigen::MatrixXd & origin, float er) {  // notice: this function is infiseable for row=col
    // 进行svd分解
    if ( origin.rows() == origin.cols()){
            cout << "sorry, this function is not availiable for row = col, please use the fucntion 'pinv_Matrix ' " << endl;
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(origin,
                                                 Eigen::ComputeThinU |
                                                 Eigen::ComputeThinV);
    // 构建SVD分解结果
    Eigen::MatrixXd U = svd_holder.matrixU();
    Eigen::MatrixXd V = svd_holder.matrixV();
    Eigen::MatrixXd D = svd_holder.singularValues();

    // 构建S矩阵
    Eigen::MatrixXd S(V.cols(), U.cols());
    S.setZero();

    for (unsigned int i = 0; i < D.size(); ++i) {

        if (D(i, 0) > er) {
            S(i, i) = 1 / D(i, 0);
        } else {
            S(i, i) = 0;
        }
    }
    // pinv_matrix = V * S * U^T
    return V * S * U.transpose();
}


Eigen::MatrixXd WqpWbc::pinv_Matirx(Eigen::MatrixXd & origin) {
	int m_row =origin.rows();
	int m_col =origin.cols();
	vector<vector<float>> vec;
	vec.clear();
	 vector<float> temp;  
	for (int i=0; i<m_row; i++){
		 temp.clear();
		for (int j=0; j<m_col ; j++){
			// temp.at(j) =origin(i,j);
			temp.push_back(origin(i,j));
		}  
		// cout << "-----------------"<< endl;
	vec.push_back(temp);
	}
	const int rows{ m_row }, cols{ m_col };
	vector<double> vec_;
	for (int i = 0; i < rows; ++i) {
		vec_.insert(vec_.begin() + i * cols, vec[i].begin(), vec[i].end());
	}
	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> m(vec_.data(), rows, cols);
	auto svd = m.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
	const auto & singularValues =svd.singularValues();
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> singularValuesInv(m.cols(), m.rows());
	singularValuesInv.setZero();
	double  pinvtoler = 1.e-6; // choose your tolerance wisely
	for (unsigned int i = 0; i < singularValues.size(); ++i) {
		if (singularValues(i) > pinvtoler)
			singularValuesInv(i, i) = 1.0f / singularValues(i);
		else
			singularValuesInv(i, i) = 0.f;
	}
	Eigen::MatrixXd pinvmat(m_col,m_row);
	pinvmat =svd.matrixV() *singularValuesInv*svd.matrixU().transpose();
	return pinvmat;
	// return true;
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
