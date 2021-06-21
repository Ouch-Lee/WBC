/**
 * @file WqpWbc.h
 * @brief declaration of the WqpWbc class, a subclass of abstract class wbc
 * @author Jiajun Wang
 * @date 2020-08-27
 * @version alpha
 * @copyright Copyright (c) 2015-2020 UBT-Beijing
 */

#ifndef WBC_WQPWBCSOLVER_H
#define WBC_WQPWBCSOLVER_H

#include <qpOASES.hpp>

#include "wbc.h"

/**
 * @class WqpWbc
 * @brief The WqpWbc class, a subclass derived from the abstract class Wbc, using the weight-QP-based WBC algorithm.
 * @note The QP solver is qpOASES
 * @see {https://github.com/coin-or/qpOASES}
 */
class WqpWbc : public Wbc {
public:

    /**
     * @brief WqpWbc Constructor
     * @param dimVar Dimension of Variables in the WBC problem, i.e. optimization
     * @param roDy Control object of WBC problem
     */
    WqpWbc(int dimVar, RobotDynamics * roDy);

    // ---------------------- rewrite virtual functions ------------------------

    ~WqpWbc();

    bool wbcInit();

    bool displayResultInformation() const;

    /**
     * @brief Set the parameters of the integer type
     * @param Parameters Input, the value of nWSR_res
     * @return
     */
    bool setParametersInt(const std::vector<int> & Parameters);

    /**
     * @brief Set the parameters of the double type
     * @param Parameters Input, the value of cputime_des
     * @return
     */
    bool setParametersDouble(const std::vector<double> & Parameters);

    /**
     * @brief Get Auxiliary Data of the integer type
     * @param auxiliaryData Output, the value of nWSR, simpleStatus
     * @return
     */
    bool getAuxiliaryDataInt(std::vector<int> & auxiliaryData);

    /**
     * @brief Get Auxiliary Data of the double type
     * @param auxiliaryData Output, the value of costOpt, _cputime
     * @return
     */
    bool getAuxiliaryDataDouble(std::vector<double> & auxiliaryData);


    // ---------------------- implement pure virtual functions -----------------

    bool updateBound(const Eigen::VectorXd & lb, const Eigen::VectorXd & ub);

    bool updateRobotDynamics(const Eigen::VectorXd & q, const Eigen::VectorXd & qDot);

    bool wbcSolve();

    bool getResultOpt(Eigen::VectorXd & resultOpt);


private:

    qpOASES::SQProblem * QP = nullptr;
    qpOASES::Options qpOption;

    qpOASES::int_t nWSR;
    qpOASES::real_t * _cputime = nullptr;

    qpOASES::returnValue statusCode_solving;
    qpOASES::returnValue statusCode_getSolution;

    qpOASES::real_t * _primal_Opt = nullptr;
    qpOASES::real_t * _dual_Opt = nullptr;

    bool init_done{false};
    int nWSR_des{100};
    double cputime_des{10};

    // ---------------------------- Costs/Objects --------------------------------------

    /// Hessian matrix, H = A^T * A
    Eigen::MatrixXd HessianMat;                                     ///< nV*nV, a Real symmetric matrix
    /// gradient vector, g = - A^T * b
    Eigen::VectorXd gradientVec;                                    ///< nV*1
    /// weight
    Eigen::VectorXd omegaVec_All;                                   ///< nO*1
    /// A
    Eigen::MatrixXd TaskMat_All;                                    ///< nO*nV
    /// b
    Eigen::VectorXd tgtVec_All;                                     ///< nO*1

    // ------------------------ Bounds & Constraints -----------------------------------

    /// lb,ub
    Eigen::VectorXd boundVec_All_lb;                                ///< nV*1
    Eigen::VectorXd boundVec_All_ub;                                ///< nV*1
    /// C
    Eigen::MatrixXd CstrMat_All;                                    ///< nC*nV
    Eigen::MatrixXd CstrMat_All_T;                                  ///< nV*nC, the transpose of CstrMat_All
    /// lbC,ubC
    Eigen::VectorXd cstrVec_All_lb;                                 ///< nC*1
    Eigen::VectorXd cstrVec_All_ub;                                 ///< nC*1

    // ------------------------ private Functions -----------------------------------
    bool createNewQP();
    bool resizeQPMatrixVector();

    /**
     * @brief qpUpdate Calculate H,g,C,... for qpOASES
     * @return
     */
    bool qpUpdate();
    bool calcHessianGradient();
    bool calcConstraintCoefficient();
    bool qpSolve();
    bool qpReset();

    bool getPrimalOpt(Eigen::VectorXd & primalOpt);
    bool getDualOpt(Eigen::VectorXd & dualOpt);

    bool getNwsr(int & nWSR_res);
    bool getSimpleStatusInt(int & simpleStatus);
    bool getOptCost(double & cost_opt);

};

#endif // WBC_WQPWBCSOLVER_H
