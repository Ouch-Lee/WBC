/**
 * @file RobotDynamics.h
 * @brief Declaration of the RobotDynamics class
 * @author Jiajun Wang
 * @date 2020-09-14
 * @version alpha
 * @copyright Copyright (c) 2015-2020 UBT-Beijing
 */

#ifndef WBC_ROBOTDYNAMICS_H
#define WBC_ROBOTDYNAMICS_H

#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Dense>
#include <rbdl/rbdl.h>

/**
 * @class RobotDynamics
 * @brief The RobotDynamics class
 * @details Dynamics calculation depends on the library 'RBDL'.
 * @par
 * @see {https://rbdl.github.io/index.html}
 * @note
 * - This is an abstract class.
 * - The public members 'CS, c_ic, N_ic' are designed for parallel robots, and generally speaking, these variables have no meaning for serial robots.
 */
class RobotDynamics {

public:

    RobotDynamics() = default;
    virtual ~RobotDynamics() = default;

    /**
     * @brief Set value of Q and Qdot
     * @param q Generalized position
     * @param qdot Generalized velocity
     * @return
     * @note Always call this function first! As "Q, Qdot" are the independent variables of all the RBDL funtions.
     */
    virtual bool setQ_Qdot(const Eigen::VectorXd & q, const Eigen::VectorXd & qdot);

    /**
     * @brief Display some necessary information
     * @return
     */
    virtual bool displayDynamicInformation();

    /**
     * @brief Get a boolean flag indicating whether the function calcWbcDependence() was executed successfully.
     * @return A boolean value
     */
    virtual bool isCalcWbcDependenceDone();

    /**
     * @brief Calculate WBC Dependence
     * @details The main content of the function is to calculate the robot dynamics dependencies for WBC
     * @note
     *  - Must call this function after setQ_Qdot();
     *  - In the end of this function, set calcWbcDependence_done to be true.
     */
    virtual bool calcWbcDependence() = 0;

    /**
     * @brief Get the total mass of the robot
     * @return totalMass of the model
     */
    virtual double getTotalMass() = 0;

    int NB;                                                 ///< Number of moving Bodys
    int NJG;                                                ///< Number of Joints of Generalized coordinates (q, qdot, qddot, tau). NJG = NJF + NJJ, normally.
    int NJF;                                                ///< Number of Joints Free-floating
    int NJJ;                                                ///< Number of non-floating-base Joints, including actuated & underactuated(paissive) joints. NJJ = NJA + NJP
    int NJA;                                                ///< Number of Joints Actuated (torque_actuated)
    int NJP;                                                ///< Number of Passive joints that do not contain floating bases
    int NFC;                                                ///< Number of Forces describing Contact

    RigidBodyDynamics::Model * model  = nullptr;            ///< Class that contains all information about the rigid body model. Such as gravity.

    RigidBodyDynamics::ConstraintSet CS;                    ///< Structure that contains both constraint information and workspace memory. For parallel robot

    RigidBodyDynamics::Math::VectorNd Q;                    ///< NJG*1, Generalized position
    RigidBodyDynamics::Math::VectorNd Qdot;                 ///< NJG*1, Generalized velocity

    RigidBodyDynamics::Math::MatrixNd M;                    ///< NJG*NJG, Mass matrix
    RigidBodyDynamics::Math::MatrixNd invM;                 ///< NJG*NJG, inverse of Mass matrix
    RigidBodyDynamics::Math::VectorNd bng;                  ///< NJG*1, non-linear effects term caused by Coriolis and Gravity without considering internal-constraints

    RigidBodyDynamics::Math::VectorNd c_ic;                 ///< NJG*1, non-linear effects term when considering internal-constraints. for parallel robot
    RigidBodyDynamics::Math::MatrixNd N_ic;                 ///< NJG*NJG, Dynamically Consistent Null Space Projection matrix of Jacobian_InternalConstraint. for parallel robot

    RigidBodyDynamics::Math::MatrixNd Sf;                   ///< NJF*NJG, Selection matrix of floating-base joints
    RigidBodyDynamics::Math::MatrixNd Sj;                   ///< NJJ*NJG, Selection matrix of non-floating-base joints
    RigidBodyDynamics::Math::MatrixNd Sa;                   ///< NJA*NJG, Selection matrix of actuated joints or Actuation matrix
    RigidBodyDynamics::Math::MatrixNd Sp;                   ///< NJP*NJG, Selection matrix of passive joints that do not contain floating bases

    RigidBodyDynamics::Math::MatrixNd AG;                   ///< NJF*NJG, Centroidal Momentum Matrix (CMM)
    RigidBodyDynamics::Math::VectorNd AGdotQdot;            ///< NJF*1, centroidal momentum bias force

    RigidBodyDynamics::Math::MatrixNd J_c;                  ///< NFC*NJG, Jacobian_contact, used to represent the end of the branches
    RigidBodyDynamics::Math::VectorNd JdotQdot_c;           ///< NFC*1, J_c * Qdot

    RigidBodyDynamics::Math::MatrixNd J_t;                  ///< ?*NJG, Free variable (Placeholder), e.g. Jacobian_torso, Jacobian_elbow, etc.
    RigidBodyDynamics::Math::VectorNd JdotQdot_t;           ///< ?*1, J_t * Qdot

    RigidBodyDynamics::Math::MatrixNd Pa;                   ///< NJA*?, TauActuated = Pa * x^T + Qa, x is the generalized variables, e.g. NJA*(NJG+NFC), x = [Qddot, f_c]'
    RigidBodyDynamics::Math::VectorNd Qa;                   ///< NJA*1, TauActuated = Pa * x^T + Qa

protected:

    bool calcWbcDependence_done{false};

    bool check(const Eigen::MatrixXd & M, int row, int col);
    bool check(const Eigen::VectorXd & v, int row);

};

#endif // WBC_ROBOTDYNAMICS_H
