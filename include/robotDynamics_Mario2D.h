/**
 * @file RobotDynamics_Mario2D.h
 * @brief declaration of the wqpWbc class, a subclass of abstract class RobotDynamics_Mario2D
 * @author Jiajun Wang
 * @date 2020-07-01
 * @version alpha
 * @copyright Copyright (c) 2015-2020 UBT-Beijing
 */

#ifndef WBC_ROBOTDYNAMICS_MARIO2D_H
#define WBC_ROBOTDYNAMICS_MARIO2D_H

#include "robotDynamics.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

/**
 * @brief The RobotDynamics_Mario2D class, for ROBOT : Mario2D
 */
class RobotDynamics_Mario2D : public RobotDynamics{

public:

    /**
     * @brief Constructor function
     * @param massRatioFlag Flag for robot mass configuration
     * @param loopFlag Flag: 0, open-tree; 1, closed-loop
     */
    RobotDynamics_Mario2D(int massRatioFlag = 0, int loopFlag = 1);
    ~RobotDynamics_Mario2D();

    /**
     * @brief Set value of Q, Qdot, and some flags
     * @param q Generalized position
     * @param qdot Generalized velocity
     * @return
     * @note Always call this function first! As "Q, Qdot" are the independent variables of all the RBDL funtions.
     */
    bool setQ_Qdot(const Eigen::VectorXd & q, const Eigen::VectorXd & qdot) override;

    bool calcWbcDependence() override;

    double getTotalMass() override;

    /**
     * @brief Fetch: MassRatioFlag
     * @return
     */
    double getMassRatioFlag();

    /**
     * @brief Fetch: massUpperBody
     * @return
     */
    double getMassUpperBody();

    /**
     * @brief Fetch: 0.5*(totalMass - massUpperBody)
     * @return
     */
    double getMassLeg();

    /**
     * @brief Fetch: IyUpperBody
     * @return
     */
    double getIyUpperBody();

    /**
     * @brief Update kinematics of RBDL model, only Position and Velocity
     * @return
     */
    bool updateKinematics_PosVel();

    /**
     * @brief Update kinematics of RBDL model, only acceleration with a zero value
     * @return
     */
    bool updateKinematics_AccZero();

    /**
     * @brief Calculate : the position and velocity of Center Of Mass.
     *      using RBDL function : Utils::CalcCenterOfMass(), fast & recommend ~
     * @return
     */
    bool centerOfMassPosVel(Eigen::VectorXd & com, Eigen::VectorXd & comDot);

    /**
     * @brief reset the flag : isPosVelUpdated to be false
     * @return
     */
    bool resetIsPosVelUpdated();

private:

    // -------- flags --------
    int MassRatioFlag{0};
    int LoopFlag{0};                    // 0: 4 open-trees; 1: 2 closed-loops. default = 1
    int FootContactFlag{0};             // 0: Jc = Jf, fore-open-tree; others: Jc = Jr, rear-open-tree
    bool UpdateKinematicsFlag{false};
    bool isPosVelUpdated{false};
    // -------- flags --------

    // ------------------------- Inertial Property -------------------------------------
    // mass : kg
    double totalMass{0.};
    double massUpperBody{16.9543};  // 17.28507 - 0.3308
    double massThighFore{0.19822};
    double massThighRear{0.22853};
    double massShinFore{0.21739};
    double massShinRear{0.26221};
    // inertia : kg/m^2
    double IyUpperBody{0.26};
    double IyThighFore{0.0010348566};
    double IyThighRear{0.00119301835};
    double IyShinFore{0.00328762843};
    double IyShinRear{0.00449769813};
    // CoM position : m
    Vector3d xyzComUpperBody;
    Vector3d xyzComThighFore;
    Vector3d xyzComThighRear;
    Vector3d xyzComShinFore;
    Vector3d xyzComShinRear;
    // ------------------------- Inertial Property -------------------------------------

    // ---------------------------- modeling ------------------------------
    std::vector<unsigned int> bodyID;
    std::vector<unsigned int> jointActuatedID;
    std::vector<unsigned int> parent;
    std::vector<Joint> joint;
    std::vector<Body> body;
    std::vector<std::string> bodyName;
    // ---------------------------- modeling ------------------------------


    // ---------------------------- for Centroidal Dynamics Calculation ----------------------------
    VectorNd gravityBias;           // NJG*1, i.e. non-linear effects term caused by gravity
    VectorNd cic_minus_g;           // NJG*1, i.e. non-linear effects term exclude gravity when considering internal-constraints

    MatrixNd Mf;                    // NJF*NJF, floating-base component of mass matrix
    MatrixNd Phi;                   // NJF*NJF, joint matrix ,describe any selection of generalized velocity for the floating base
    MatrixNd Psi;                   // NJF*NJF, Psi = Phi^-1
    MatrixNd I_locked_B;            // NJF*NJF, composite-rigid-body inertia (locked inertia) of the system I_C_B
    VectorNd p_CoM_B;               // 3*1, CoM position in frame 'B' : [x, y=0, z]^T
    MatrixNd X_BG_Star;             // NJF*NJF, Transpose of the spatial transformation, from 'G' to 'B', extracted from a 6*6 spatial transformation
    MatrixNd T_AG;                  // NJF*NJG, T_AG = X_BG_Star * J_FB^-1^T * Sf

    MatrixNd jabobian6DFloatingBase;
    MatrixNd J_FB_Temp;
    MatrixNd Phi_R;
    MatrixNd Phi_Temp;
    MatrixNd X_BG_Star_6D;
    //---------------------------- for Centroidal Dynamics Calculation ----------------------------

    // -------- CS 'G*Qddot - Gamma == 0' --------
    MatrixNd Lambda_G_ic;           // 4*4, i.e. Apparent Inertia of internal constraint, for CS.G
    MatrixNd invG_DC;               // 4*NJG, i.e. Dynamically Consistent Inverse of Jacobian_internal, for CS.G
    // -------- CS 'G*Qddot - Gamma == 0' --------

    // -------- local intermediate variable --------
    // --- Update Kinematics Custom ---
    VectorNd QddotZero;
    // --- Centroidal Dynamics ---
    Vector3d CoM3d;
    Vector3d CoMdot3d;
    // --- Inverse Dynamics Term ---
    MatrixNd invUa;
    // --- Jacobian Calculating ---
    MatrixNd jacobianFRpfTemp;
    MatrixNd jacobianRRpfTemp;
    MatrixNd jacobianFLpfTemp;
    MatrixNd jacobianRLpfTemp;
    Vector3d JdotQdot_FRpfTemp;
    Vector3d JdotQdot_RRpfTemp;
    Vector3d JdotQdot_FLpfTemp;
    Vector3d JdotQdot_RLpfTemp;
    // --- skew() Calculating ---
    MatrixNd sR;
    // -------- local intermediate variable --------

    std::vector<SpatialVector> *_6dForce_ext = nullptr;
    VectorNd FxFzRL_S_ext;          // NFC*1

    VectorNd Qddot;                 // NJG*1
    VectorNd TauA;                  // NJA*1, Actuated joint Torques
    VectorNd TauG;                  // NJG*1, Generalized Torques of generalized coordinate

    MatrixNd jacobianFRpf;          // 2*NJG
    MatrixNd jacobianRRpf;          // 2*NJG
    MatrixNd jacobianFLpf;          // 2*NJG
    MatrixNd jacobianRLpf;          // 2*NJG
    MatrixNd jacobianRightLoop;     // 2*NJG
    MatrixNd jacobianLeftLoop;      // 2*NJG
    MatrixNd jacobianLoops;         // 4*NJG
    MatrixNd jacobianPfRight;       // 2*NJG
    MatrixNd jacobianPfLeft;        // 2*NJG
    MatrixNd jacobianPfRL;          // 4*NJG

    VectorNd JdotQdot_FRpf;         // 2*1
    VectorNd JdotQdot_RRpf;         // 2*1
    VectorNd JdotQdot_FLpf;         // 2*1
    VectorNd JdotQdot_RLpf;         // 2*1
    VectorNd JdotQdot_RightLoop;    // 2*1
    VectorNd JdotQdot_LeftLoop;     // 2*1
    VectorNd JdotQdot_Loops;        // 4*1
    VectorNd JdotQdot_PfRight;      // 2*1
    VectorNd JdotQdot_PfLeft;       // 2*1
    VectorNd JdotQdot_PfRL;         // 4*1

    /**
     * @brief Set external forces
     * @param FxFz_R vector2d, external 2D-forces (FxFz) acting on the Right-point-foot, in inertial-reference-frame.
     * @param FxFz_L vector2d, external 2D-forces (FxFz) acting on the Left-point-foot, in inertial-reference-frame.
     * @return
     * @note Must call this function after setQ_Qdot()
     */
    bool setExternalForces(const Vector2d& FxFz_R, const Vector2d& FxFz_L);
    /**
     * @brief Set external forces
     * @param FxFz_R_FxFz_L vector4d, 2 external 2D-forces (FxFz) acting on the Right-point-foot & Left-point-foot, in inertial-reference-frame.
     * @return
     * @note Must call this function after setQ_Qdot()
     */
    bool setExternalForces(const VectorNd& FxFz_R_FxFz_L);

    bool setFootContactFlag(int footContactFlag);                           ///< set: FootContactFlag

    /**
     * @brief set the value of UpdateKinematicsFlag
     * @param updateKinematicsFlag A boolean, set this value to UpdateKinematicsFlag;
     * @return
     */
    bool setUpdateKinematicsFlag(bool updateKinematicsFlag);

    /**
     * @brief get the value of flag : isPosVelUpdated
     * @return the value of isPosVelUpdated
     */
    bool getUpdateKinematicsFlag();

    /**
     * @brief set the value of calcWbcDependence_done
     * @param tureOrFlase A boolean, set this value to calcWbcDependence_done;
     * @return
     */
    bool setWbcDependenceFlag(bool tureOrFlase);

    bool setQddot(const VectorNd& qdd);                                     ///< set: Qddot
    bool setTauActuated(const VectorNd& tau_a);                             ///< set: TauA

    int getFootContactFlag();                                               ///< fetch: FootContactFlag

    bool getQddot(VectorNd& qdd);                                           ///< fetch: Qddot
    bool getTauGeneralized(VectorNd& tau_G);                                ///< fetch: TauG
    bool getTauActuated(VectorNd& tau_A);                                   ///< fetch: TauA

    /**
     * @brief Calculate: Jacobian Information. Update : J_c, JdotQdot_c
     * @return
     */
    bool calcJacobianInformation();

    /**
     * @brief Calculate: Model, CS(H C G gamma), M, invM, bng, c_ic, N_ic, for LoopFlag == 0 and 1
     * @return
     * @note Must call the follow functions after setQ_Qdot() and calcJacobianInformation(), Custom is faster than ConstraintSet
     */
    bool calcClosedLoopsDynamicallyConsistent_Custom();

    /**
     * @brief Calculate: Model, CS(H C G gamma), M, invM, bng, c_ic, N_ic, for LoopFlag == 1
     * @return
     * @note Must call the follow functions after setQ_Qdot(), Custom is faster than ConstraintSet
     */
    bool calcClosedLoopsDynamicallyConsistent_ConstraintSet();

    /**
     * @brief Calculate Centroidal Dynamics Descriptors: AG, AGdotQdot
     * @return
     * @note Must call the follow functions after setQ_Qdot() and calcClosedLoopsDynamicallyConsistent_*()
     */
    bool calcCentroidalDynamicsDescriptors();

    /**
     * @brief Calculate: Jc, JdotQdot_c, Pa, Qa
     * @return
     * @note Must call the follow functions after setQ_Qdot() and calcJacobianInformation() and calcClosedLoopsDynamicallyConsistent_*()
     */
    bool calcInverseDynamicsDescriptors();

    /**
     * @brief Calculate: CoM, CoMdot, use p_CoM_B & AG. This function runs slowly, NOT recommend!
     * @return
     * @note Must call the follow functions after calcCentroidalDynamicsDescriptors().
     */
    bool calcCenterOfMassPosVel_useAG(Eigen::VectorXd &com, Eigen::VectorXd &comDot);


    bool ForwardDynamics();     ///< calculate: Qddot

    bool InverseDynamics();     ///< calculate: TauA

    bool calcJacobianFRpf();
    bool calcJacobianRRpf();
    bool calcJacobianFLpf();
    bool calcJacobianRLpf();
    bool calcJacobianRightLoop();
    bool calcJacobianLeftLoop();
    bool calcJacobianLoops();
    bool calcJacobianPfRight();
    bool calcJacobianPfLeft();
    bool calcJacobianPfRL();

    bool calcJdotQdot_FRpf();
    bool calcJdotQdot_RRpf();
    bool calcJdotQdot_FLpf();
    bool calcJdotQdot_RLpf();
    bool calcJdotQdot_RightLoop();
    bool calcJdotQdot_LeftLoop();
    bool calcJdotQdot_Loops();
    bool calcJdotQdot_PfRight();
    bool calcJdotQdot_PfLeft();
    bool calcJdotQdot_PfRL();

    MatrixNd skew(const VectorNd & w);

    /* ----------------- for multi thread programming -------------------- */
    bool calcJacobianInformationAndNonlinearEffects();
    bool calcInertiaMatrixAndInverse();
    bool calcClosedLoopsNullSpaceAndNonlinear();
    bool calcGravityBiasAndCentroidalPsi();
    bool calcInverseDynamicsTerms();
    bool calcCentroidalDynamicsTerms();
    /* ----------------- for multi thread programming -------------------- */

};

#endif // WBC_ROBOTDYNAMICS_MARIO2D_H
