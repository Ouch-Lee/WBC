/**
 * @file NspWbc.h
 * @brief declaration of the NspWbc class, a subclass of abstract class wbc
 * @author Yan Xie
 * @date 2020-09-17
 * @version alpha
 * @copyright Copyright (c) 2015-2020 UBT-Beijing
 */

#ifndef WBC_NSPWBCSOLVER_H
#define WBC_NSPWBCSOLVER_H

#include <qpOASES.hpp>

#include "wbc.h"

/**
 * @class NspWbc
 * @brief The NspWbc class, a subclass derived from the abstract class Wbc, using the hierarchy-QP-based WBC algorithm.
 * @note The QP solver is qpOASES
 * @see {https://github.com/coin-or/qpOASES}
 */
class NspWbc : public Wbc {
public:

    /**
     * @brief NspWbc Constructor
     * @param dimVar Dimension of Variables in the WBC problem, i.e. optimization
     * @param roDy Control object of WBC problem
     */
    NspWbc(int dimVar, RobotDynamics * roDy);

    /**
     * @brief Copy constructor of HqpWbc
     * @param wbc_foo the wbc instance to be copied from
     */
    NspWbc(const Wbc &wbc_foo);

    // ---------------------- rewrite virtual functions ------------------------

    ~NspWbc();

    bool wbcInit();

    bool displayResultInformation() const;
    /**
     * @brief Set the parameters of the integer type
     * @param Parameters Input, the value of nWSR_res
     * @return
     */
    bool setParametersInt(const std::vector<int> & Parameters);                 ///< nWSR
    /**
     * @brief Set the parameters of the double type
     * @param Parameters Input, the value of cputime_des
     * @return
     */
    bool setParametersDouble(const std::vector<double> & Parameters);           ///< _cpu_time
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

    std::vector<qpOASES::SQProblem *> QP;
    std::vector<qpOASES::Options> qpOption;

    std::vector<qpOASES::int_t> nWSR;
    std::vector<qpOASES::real_t *> _cputime;

    std::vector<qpOASES::returnValue> statusCode_solving;
    std::vector<qpOASES::returnValue> statusCode_getSolution;

    std::vector<qpOASES::real_t *> _primal_Opt;
    std::vector<qpOASES::real_t *> _dual_Opt;

    std::vector<bool> init_done;

    std::vector<int> nWSR_des;
    std::vector<double> cputime_des;

    // ---------------------------- Costs/Objects --------------------------------------

    /// Hessian matrix, H = A^T * A
    std::vector<Eigen::MatrixXd> HessianMat;                                     ///< nVi*nVi, a Real symmetric matrix
    /// gradient vector, g = - A^T * b
    std::vector<Eigen::VectorXd> gradientVec;                                    ///< nVi*1
    /// omega
    std::vector<Eigen::VectorXd> omegaVec_All;                                   ///< nOi*1
    /// A
    std::vector<Eigen::MatrixXd> TaskMat_All;                                    ///< nOi*nVi
    /// b
    std::vector<Eigen::VectorXd> tgtVec_All;                                     ///< nOi*1

    // ------------------------ Bounds & Constraints -----------------------------------

    /// lb,ub
    std::vector<Eigen::VectorXd> boundVec_All_lb;                                ///< nVi*1
    std::vector<Eigen::VectorXd> boundVec_All_ub;                                ///< nVi*1
    /// C
    std::vector<Eigen::MatrixXd> CstrMat_All;                                    ///< nCi*nVi
    std::vector<Eigen::MatrixXd> CstrMat_All_T;                                  ///< nVi*nCi, the transpose of CstrMat_All
    /// lbC,ubC
    std::vector<Eigen::VectorXd> cstrVec_All_lb;                                 ///< nCi*1
    std::vector<Eigen::VectorXd> cstrVec_All_ub;                                 ///< nCi*1

    // ------------------------ private Functions -----------------------------------
    bool createNewQP();
    bool resizeQPMatrixVector();

    /**
     * @brief qpUpdate Calculate H,g,C,... for qpOASES
     * @return
     */
    bool nspUpdate();
    bool calcHessianGradient();
    bool calcConstraintCoefficient();

    bool nspSolve();
    bool nspUpdateLeveln(const int & iLevel);
    bool nspSolveLeveln(const int & iLevel);
    bool qpResetLeveln(const int & iLevel);

    bool getResultOptLeveln(Eigen::VectorXd & resultOpt, const int & iLevel);
    bool getPrimalOptLeveln(Eigen::VectorXd & primalOpt, const int & iLevel);
    bool getDualOptLeveln(Eigen::VectorXd & dualOpt, const int & iLevel);

    bool getNwsr(std::vector<int> & nWSR_res);
    bool getSimpleStatusInt(std::vector<int> & simpleStatus);
    bool getOptCost(std::vector<double> & cost_opt);

    int nLevel{0};
    int nLevelTask{0};
    int nLevelConstraint{0};

    std::vector<int> nVLevel;
    std::vector<int> nOLevel;
    std::vector<int> nCLevel;

    /// Z, X*
    std::vector<Eigen::MatrixXd> Z_Mat;                                                         ///< nVi*nVi
    std::vector<Eigen::VectorXd> x_star;                                                        ///< nVi*1

    /// omega_level
    std::vector<Eigen::VectorXd> omegaVec_Level;                                   ///< nOi*1
    /// A_level
    std::vector<Eigen::MatrixXd> TaskMat_Level;                                    ///< nOi*nVi
    /// AHat_level
    std::vector<Eigen::MatrixXd> TaskMatHat_Level;                                    ///< nOi*nVi
    /// AHatInv_level
    std::vector<Eigen::MatrixXd> TaskMatHatInv_Level;                                    ///< nVi*nOi
    /// b_level
    std::vector<Eigen::VectorXd> tgtVec_Level;                                     ///< nOi*1
    /// lb_level,ub_level
    std::vector<Eigen::VectorXd> boundVec_Level_lb;                                ///< nVi*1
    std::vector<Eigen::VectorXd> boundVec_Level_ub;                                ///< nVi*1
    /// C_level
    std::vector<Eigen::MatrixXd> CstrMat_Level;                                    ///< nCi*nVi
    std::vector<Eigen::MatrixXd> CstrMat_Level_T;                                  ///< nVi*nCi, the transpose of CstrMat_Level
    /// lbC_level,ubC_level
    std::vector<Eigen::VectorXd> cstrVec_Level_lb;                                 ///< nCi*1
    std::vector<Eigen::VectorXd> cstrVec_Level_ub;                                 ///< nCi*1    
};

#endif // WBC_HQPWBCSOLVER_H
