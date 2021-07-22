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
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Core>
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

    /**
     * @brief Copy constructor of WqpWbc
     * @param wbc_foo the wbc instance to be copied from
     */
    WqpWbc(const Wbc &wbc_foo);

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

    // 7 basic Jacobian for each task
    Eigen::MatrixXd J1;
    Eigen::MatrixXd J2;
    Eigen::MatrixXd J3;
    Eigen::MatrixXd J4;
    Eigen::MatrixXd J5;
    Eigen::MatrixXd J6;
    Eigen::MatrixXd J7;
    
    // 4 synthetic Jacobian for 4 priority level
    Eigen::MatrixXd J_level1;
    Eigen::MatrixXd J_level2;
    Eigen::MatrixXd J_level3;
    Eigen::MatrixXd J_level4;

    // 3 Pinv Jacobian 
    Eigen::MatrixXd J_level1_pinv;
    Eigen::MatrixXd J_level2_pinv;
    Eigen::MatrixXd J_level3_pinv;
    Eigen::MatrixXd J_level4_pinv;

    // 7 manipulation vatriable R, as well as b
    Eigen::VectorXd R1;
    Eigen::VectorXd R2;
    Eigen::VectorXd R3;
    Eigen::VectorXd R4;
    Eigen::VectorXd R5;
    Eigen::VectorXd R6;
    Eigen::VectorXd R7;

    // 4 systhetic vatiables for 4  priority level
    Eigen::VectorXd R_level1;
    Eigen::VectorXd R_level2;
    Eigen::VectorXd R_level3;
    Eigen::VectorXd R_level4;

    //calculate the X as the desire Joint paramas
    Eigen::VectorXd X;
    
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
    bool calcJAndB();    // this function is aimed to calculate Jacobian(A) and manipulation variable(b) for each task
    bool calc_X();          //this function is aimed to calculate X_desire according to the equations provide by Wang
    bool calcHessianGradient();  //calculate H and g based on the former paramas
    bool calcConstraintCoefficient();
    bool qpSolve();
    bool qpReset();
    Eigen::MatrixXd pinv_eigen_based(Eigen::MatrixXd & origin, float er) ;   //a function to calculate pseudo-inverse of a Matrix ,er stands for error; but when row=col it's useless
    Eigen::MatrixXd pinv_Matirx(Eigen::MatrixXd & origin);   // suit for all kind matrix to calculate pseudo-inverse
    // Eigen::VectorXd calcaTask_X( int levels,);
    bool getPrimalOpt(Eigen::VectorXd & primalOpt);
    bool getDualOpt(Eigen::VectorXd & dualOpt);

    bool getNwsr(int & nWSR_res);
    bool getSimpleStatusInt(int & simpleStatus);
    bool getOptCost(double & cost_opt);

};

#endif // WBC_WQPWBCSOLVER_H
