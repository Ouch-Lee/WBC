/**
 * @file RobotDynamics_Mario2D.cpp
 * @brief Function implementation part of class RobotDynamics_Mario2D
 * @author Jiajun Wang
 * @date 2020-07-01
 * @version alpha
 * @copyright Copyright (c) 2015-2020 UBT-Beijing
 */

#include "robotDynamics_Mario2D.h"

// ======================================== Public Functions ==========================================

RobotDynamics_Mario2D::RobotDynamics_Mario2D(int massRatioFlag, int loopFlag) {

    // -------- flags --------
    MassRatioFlag = massRatioFlag;
    LoopFlag = loopFlag;
    FootContactFlag = 0;
    // -------- flags --------

    // ------------------------- public members of Base class -------------------
    NB = 11;
    NJG = 11;
    NJF = 3;
    NJJ = 8;
    NJA = 4;
    NJP = 4;
    NFC = 4;

    Q = VectorNd :: Zero(NJG);
    Qdot = VectorNd :: Zero(NJG);

    M = MatrixNd :: Zero(NJG, NJG);
    invM = MatrixNd :: Zero(NJG, NJG);
    bng = VectorNd :: Zero(NJG);

    c_ic = VectorNd :: Zero(NJG);
    N_ic = MatrixNd :: Zero(NJG, NJG);

    Sf = MatrixNd :: Zero(NJF, NJG);
    Sj = MatrixNd :: Zero(NJJ, NJG);
    Sa = MatrixNd :: Zero(NJA, NJG);
    Sp = MatrixNd :: Zero(NJP, NJG);

    AG = MatrixNd :: Zero(NJF, NJG);
    AGdotQdot = VectorNd :: Zero(NJF);

    J_c = MatrixNd :: Zero(NFC, NJG);
    JdotQdot_c = VectorNd :: Zero(NFC);

    J_t = MatrixNd :: Zero(NJF, NJG);
    JdotQdot_t = VectorNd :: Zero(NJF);

    Pa = MatrixNd :: Zero(NJA, NJG+NFC);
    Qa = VectorNd :: Zero(NJA);
    // ------------------------- public members of Base class -------------------

    // ----------------- Inertial Property -----------------
    xyzComUpperBody = Vector3d(0.002, 0., 0.09938); //0.002
    xyzComThighFore = Vector3d(0.00003, 0., 0.08248);
    xyzComThighRear = Vector3d(0.00001, 0., 0.09189);
    xyzComShinFore = Vector3d(0., 0., 0.15132);
    xyzComShinRear = Vector3d(0., 0., 0.18463);

    if (MassRatioFlag == 1){
        // real
        massShinFore = 1.29711-0.0102;
        massShinRear = 1.55285-0.0376;
        IyShinFore = 0.01556;
        IyShinRear = 0.01656;
        xyzComShinFore = Vector3d(0., 0., 0.14303);
        xyzComShinRear = Vector3d(0., 0., 0.15835);
        // fake
//        massShinFore = 1.5;
//        massShinRear = 1.5;
//        IyShinFore = 0.014;
//        IyShinRear = 0.014;
//        xyzComShinFore = Vector3d(0., 0., 0.16);
//        xyzComShinRear = Vector3d(0., 0., 0.16);
    } else if (MassRatioFlag == 2){
        // fake
        massThighFore = 1.5;
        massThighRear = 1.5;
        IyThighFore = 0.005;
        IyThighRear = 0.005;
        xyzComThighFore = Vector3d(0., 0., 0.08);
        xyzComThighRear = Vector3d(0., 0., 0.08);
    } else if (MassRatioFlag == 3){
        // fake
        massThighFore = 1.5;
        massThighRear = 1.5;
        IyThighFore = 0.005;
        IyThighRear = 0.005;
        xyzComThighFore = Vector3d(0., 0., 0.08);
        xyzComThighRear = Vector3d(0., 0., 0.08);
        // real
        massShinFore = 1.29711-0.0102;
        massShinRear = 1.55285-0.0376;
        IyShinFore = 0.01556;
        IyShinRear = 0.01656;
        xyzComShinFore = Vector3d(0., 0., 0.14303);
        xyzComShinRear = Vector3d(0., 0., 0.15835);
    }
    // ----------------- Inertial Property -----------------

    // ---------------------------- modeling ------------------------------
    rbdl_check_api_version(RBDL_API_VERSION);
    model = new Model();
    model -> gravity = Vector3d(0., 0., -9.80665);

    bodyID.resize(NB);      // {1, 2, ..., 10, 11}
    jointActuatedID .resize(NJA);
    jointActuatedID = {4, 6, 8, 10};
    parent.resize(NB);
    parent = {0, 1, 2, 3, 4, 3, 6, 3, 8, 3, 10};
    joint.resize(NJG);
    body.resize(NB);
    bodyName.resize(NB);
    bodyName = {"Cross", "Pin", "Torso",
                "FR_thigh", "FR_shin",
                "RR_thigh", "RR_shin",
                "FL_thigh", "FL_shin",
                "RL_thigh", "RL_shin"};
    // ---------------------------- modeling ------------------------------

    // -------- for Centroidal Dynamics Calculation -------
    gravityBias = VectorNd :: Zero(NJG);
    cic_minus_g = VectorNd :: Zero(NJG);

    Mf = MatrixNd :: Zero(NJF, NJF);
    Phi = MatrixNd :: Zero(NJF, NJF);
    Psi = MatrixNd :: Zero(NJF, NJF);
    I_locked_B = MatrixNd :: Zero(NJF, NJF);
    p_CoM_B = VectorNd :: Zero(3);
    X_BG_Star = MatrixNd :: Zero(NJF, NJF);
    T_AG = MatrixNd :: Zero(NJF, NJG);

    jabobian6DFloatingBase = MatrixNd :: Zero(6,NJG);
    J_FB_Temp = MatrixNd :: Zero(6,NJF);
    Phi_R = MatrixNd :: Zero(6,6);
    Phi_Temp = MatrixNd :: Zero(6,NJF);
    X_BG_Star_6D = MatrixNd :: Zero(6,6);
    // -------- for Centroidal Dynamics Calculation -------

    // -------- CS 'G*Qddot - Gamma == 0' --------
    Lambda_G_ic = MatrixNd :: Zero(4, 4);
    invG_DC = MatrixNd :: Zero(4, NJG);
    // -------- CS 'G*Qddot - Gamma == 0' --------

    // -------- local intermediate variable --------
    QddotZero = VectorNd :: Zero(NJG);
    CoM3d = Vector3d :: Zero();
    CoMdot3d = Vector3d :: Zero();
    invUa = MatrixNd :: Zero(NJA, NJA);
    jacobianFRpfTemp = MatrixNd :: Zero(3, NJG);
    jacobianRRpfTemp = MatrixNd :: Zero(3, NJG);
    jacobianFLpfTemp = MatrixNd :: Zero(3, NJG);
    jacobianRLpfTemp = MatrixNd :: Zero(3, NJG);
    JdotQdot_FRpfTemp = Vector3d :: Zero();
    JdotQdot_RRpfTemp = Vector3d :: Zero();
    JdotQdot_FLpfTemp = Vector3d :: Zero();
    JdotQdot_RLpfTemp = Vector3d :: Zero();
    sR = MatrixNd :: Zero(3,3);
    // -------- local intermediate variable --------

    FxFzRL_S_ext = VectorNd :: Zero(NFC);

    Qddot = VectorNd :: Zero(NJG);
    TauA = VectorNd :: Zero(NJA);
    TauG = VectorNd :: Zero(NJG);

    jacobianFRpf = MatrixNd :: Zero(2,NJG);
    jacobianRRpf = MatrixNd :: Zero(2,NJG);
    jacobianFLpf = MatrixNd :: Zero(2,NJG);
    jacobianRLpf = MatrixNd :: Zero(2,NJG);
    jacobianRightLoop = MatrixNd :: Zero(2,NJG);
    jacobianLeftLoop = MatrixNd :: Zero(2,NJG);
    jacobianLoops = MatrixNd :: Zero(4,NJG);
    jacobianPfRight = MatrixNd :: Zero(2,NJG);
    jacobianPfLeft = MatrixNd :: Zero(2,NJG);
    jacobianPfRL = MatrixNd :: Zero(4,NJG);

    JdotQdot_FRpf = VectorNd :: Zero(2);
    JdotQdot_RRpf = VectorNd :: Zero(2);
    JdotQdot_FLpf = VectorNd :: Zero(2);
    JdotQdot_RLpf = VectorNd :: Zero(2);
    JdotQdot_RightLoop = VectorNd :: Zero(2);
    JdotQdot_LeftLoop = VectorNd :: Zero(2);
    JdotQdot_Loops = VectorNd :: Zero(4);
    JdotQdot_PfRight = VectorNd :: Zero(2);
    JdotQdot_PfLeft = VectorNd :: Zero(2);
    JdotQdot_PfRL = VectorNd :: Zero(4);

    // -------- public: Sf, Sj, Sa, Sp --------
    for (int i = 0; i != NJA; ++i){
        Sa(i, jointActuatedID[i] - 1) = 1.;
    }
    Sj.rightCols(NJJ) = MatrixNd::Identity(NJJ,NJJ);
    Sf.leftCols(NJF) = Matrix3dIdentity;
    for (int i = 0; i != NJP; ++i){
        Sp(i, jointActuatedID[i]) = 1.;
    }
    // -------- public: Sf, Sj, Sa, Sp --------


    // ---------------------------- modeling process ------------------------------

    // prismatic Translate X
    joint[0] = Joint(JointTypePrismatic, Vector3d(1., 0., 0.));
    // prismatic Translate Z
    joint[1] = Joint(JointTypePrismatic, Vector3d(0., 0., 1.));
    // revolute Rotate Y
    joint[2] = Joint(JointTypeRevolute, Vector3d(0., 1., 0.));
    // -------- revolute Rotate Y --------
    joint[3] = Joint(JointTypeRevolute, Vector3d(0., 1., 0.));
    joint[4] = Joint(JointTypeRevolute, Vector3d(0., 1., 0.));
    joint[5] = Joint(JointTypeRevolute, Vector3d(0., 1., 0.));
    joint[6] = Joint(JointTypeRevolute, Vector3d(0., 1., 0.));
    joint[7] = Joint(JointTypeRevolute, Vector3d(0., 1., 0.));
    joint[8] = Joint(JointTypeRevolute, Vector3d(0., 1., 0.));
    joint[9] = Joint(JointTypeRevolute, Vector3d(0., 1., 0.));
    joint[10] = Joint(JointTypeRevolute, Vector3d(0., 1., 0.));

    // assign body information (mass, com, inertia_C)
    Matrix3d *bodyInitR = new Matrix3d[NB];
    Vector3d *bodyInitP = new Vector3d[NB];
    // virtual floating body (Cross, Pin)
    for (int i = 0; i < NJF-1; ++i) {
        body[i] = Body(0., Vector3dZero, Matrix3dZero);
        bodyInitR[i].setIdentity();
        bodyInitP[i].setZero();
    }
    // Torso
    bodyInitR[2].setIdentity();
    bodyInitP[2].setZero();
    body[2] = Body(massUpperBody, xyzComUpperBody, Matrix3d(1e-4, 0., 0.,
                                                            0., IyUpperBody, 0.,
                                                            0., 0., 1e-4));
    // FR_thigh
    bodyInitR[3].setIdentity();
    bodyInitP[3] << 0.0485, -0.09, 0.;
    body[3] = Body(massThighFore, xyzComThighFore, Matrix3d(1e-4, 0., 0.,
                                                            0., IyThighFore, 0.,
                                                            0., 0., 1e-4));
    // FR_shin
    bodyInitR[4].setIdentity();
    bodyInitP[4] << 0., 0., 0.16;
    body[4] = Body(massShinFore, xyzComShinFore, Matrix3d(1e-4, 0., 0.,
                                                            0., IyShinFore, 0.,
                                                            0., 0., 1e-4));
    // RR_thigh
    bodyInitR[5].setIdentity();
    bodyInitP[5] << -0.0485, -0.09, 0.;
    body[5] = Body(massThighRear, xyzComThighRear, Matrix3d(1e-4, 0., 0.,
                                                            0., IyThighRear, 0.,
                                                            0., 0., 1e-4));
    // RR_thin
    bodyInitR[6].setIdentity();
    bodyInitP[6] << 0., 0., 0.16;
    body[6] = Body(massShinRear, xyzComShinRear, Matrix3d(1e-4, 0., 0.,
                                                            0., IyShinRear, 0.,
                                                            0., 0., 1e-4));
    // FL_thigh
    bodyInitR[7].setIdentity();
    bodyInitP[7] << 0.0485, 0.09, 0.;
    body[7] = Body(massThighFore, xyzComThighFore, Matrix3d(1e-4, 0., 0.,
                                                            0., IyThighFore, 0.,
                                                            0., 0., 1e-4));
    // FL_shin
    bodyInitR[8].setIdentity();
    bodyInitP[8] << 0., 0., 0.16;
    body[8] = Body(massShinFore, xyzComShinFore, Matrix3d(1e-4, 0., 0.,
                                                            0., IyShinFore, 0.,
                                                            0., 0., 1e-4));
    // RL_thigh
    bodyInitR[9].setIdentity();
    bodyInitP[9] << -0.0485, 0.09, 0.;
    body[9] = Body(massThighRear, xyzComThighRear, Matrix3d(1e-4, 0., 0.,
                                                            0., IyThighRear, 0.,
                                                            0., 0., 1e-4));
    // RL_thin
    bodyInitR[10].setIdentity();
    bodyInitP[10] << 0., 0., 0.16;
    body[10] = Body(massShinRear, xyzComShinRear, Matrix3d(1e-4, 0., 0.,
                                                            0., IyShinRear, 0.,
                                                            0., 0., 1e-4));

    // Model Assembly
    for (int i = 0; i < NB; ++i) {
        bodyID[i] = model -> AddBody(parent[i], SpatialTransform(bodyInitR[i], bodyInitP[i]), joint[i], body[i], bodyName.at(i));
        totalMass += model -> mBodies[i+1].mMass;
    }

    if (LoopFlag == 1){
        // -------- Add loop constraints to the ConstraintSet --------
        CS.AddLoopConstraint(5,
                                  7,
                                  SpatialTransform(Matrix3dIdentity, Vector3d(0., 0., 0.32)),
                                  SpatialTransform(Matrix3dIdentity, Vector3d(0., 0., 0.32)),
                                  SpatialVector(0., 0., 0., 1., 0., 0.),
                                  false,
                                  0.1,
                                  "lcRightPf_x");
        CS.AddLoopConstraint(5,
                                  7,
                                  SpatialTransform(Matrix3dIdentity, Vector3d(0., 0., 0.32)),
                                  SpatialTransform(Matrix3dIdentity, Vector3d(0., 0., 0.32)),
                                  SpatialVector(0., 0., 0., 0., 0., 1.),
                                  false,
                                  0.1,
                                  "lcRightPf_z");
        CS.AddLoopConstraint(9,
                                  11,
                                  SpatialTransform(Matrix3dIdentity, Vector3d(0., 0., 0.32)),
                                  SpatialTransform(Matrix3dIdentity, Vector3d(0., 0., 0.32)),
                                  SpatialVector(0., 0., 0., 1., 0., 0.),
                                  false,
                                  0.1,
                                  "lcLeftPf_x");
        CS.AddLoopConstraint(9,
                                  11,
                                  SpatialTransform(Matrix3dIdentity, Vector3d(0., 0., 0.32)),
                                  SpatialTransform(Matrix3dIdentity, Vector3d(0., 0., 0.32)),
                                  SpatialVector(0., 0., 0., 0., 0., 1.),
                                  false,
                                  0.1,
                                  "lcLeftPf_z");

        // Initialize and allocate memory for the ConstraintSet.
        CS.Bind(*model);

    }else{  
        CS.clear();         // Clear all variables in the constraint set.
    }

    // ---------------------------- modeling ------------------------------

    // External forces acting on the body in base coordinates (optional, defaults to nullptr)
    _6dForce_ext = new std::vector<Math::SpatialVector>(model->dof_count + 1);
    for(auto& item : *_6dForce_ext){
        item = SpatialVector(0., 0., 0., 0., 0., 0.);
    }

    delete [] bodyInitR;
    delete [] bodyInitP;
}

RobotDynamics_Mario2D::~RobotDynamics_Mario2D(){
    delete model;
    model = nullptr;
    delete _6dForce_ext;
    _6dForce_ext = nullptr;
}

bool RobotDynamics_Mario2D::setQ_Qdot(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot){
    if (!isPosVelUpdated){
        RobotDynamics::setQ_Qdot(q, qdot);
    }
    return true;
}

bool RobotDynamics_Mario2D::calcWbcDependence(){
    if (!isPosVelUpdated){
        updateKinematics_PosVel();
    }
    updateKinematics_AccZero();
    calcJacobianInformation();
    calcClosedLoopsDynamicallyConsistent_Custom();      // Custom is faster than ConstraintSet
    calcCentroidalDynamicsDescriptors();
    calcInverseDynamicsDescriptors();
    calcWbcDependence_done = true;
    isPosVelUpdated = false;
    return true;
}

double RobotDynamics_Mario2D::getTotalMass(){
    return totalMass;
}

double RobotDynamics_Mario2D::getMassRatioFlag(){
    return MassRatioFlag;
}

double RobotDynamics_Mario2D::getMassUpperBody(){
    return massUpperBody;
}

double RobotDynamics_Mario2D::getMassLeg(){
    return 0.5*(totalMass - massUpperBody);
}

double RobotDynamics_Mario2D::getIyUpperBody(){
    return IyUpperBody;
}

bool RobotDynamics_Mario2D::updateKinematics_PosVel(){
    UpdateKinematicsCustom(*model, &Q, &Qdot, nullptr);
    isPosVelUpdated = true;
    return true;
}

bool RobotDynamics_Mario2D::updateKinematics_AccZero(){
    UpdateKinematicsCustom(*model, nullptr, nullptr, &QddotZero);
    return true;
}

bool RobotDynamics_Mario2D::centerOfMassPosVel(Eigen::VectorXd &com, Eigen::VectorXd &comDot){
    updateKinematics_PosVel();
    Utils::CalcCenterOfMass(*model, Q, Qdot, nullptr, totalMass, CoM3d, &CoMdot3d, nullptr, nullptr, nullptr, UpdateKinematicsFlag);
    com << CoM3d(0),
           CoM3d(2);
    comDot << CoMdot3d(0),
              CoMdot3d(2);
    return true;
}

bool RobotDynamics_Mario2D::resetIsPosVelUpdated(){
    isPosVelUpdated = false;
    return true;
}

// ======================================== Private Functions ==========================================

bool RobotDynamics_Mario2D :: setExternalForces(const Vector2d &FxFz_R, const Vector2d &FxFz_L){
    if (check(FxFz_R, 2) && check(FxFz_L, 2)){
        Vector3d _3dForce_R_S_ext = Matrix3dIdentity*Vector3d(FxFz_R(0), 0., FxFz_R(1)); ///< as the frame's orientation is the same one.
        Vector3d Pf_R_S = CalcBodyToBaseCoordinates(*model, Q, 7, Vector3d(0., 0., 0.32), UpdateKinematicsFlag);
        Vector3d _3dCouple_R_S_ext = Pf_R_S.cross(_3dForce_R_S_ext);
        _6dForce_ext->at(7) << _3dCouple_R_S_ext,
                                _3dForce_R_S_ext;
        Vector3d _3dForce_L_S_ext = Matrix3dIdentity*Vector3d(FxFz_L(0), 0., FxFz_L(1)); ///< as the frame's orientation is the same one.
        Vector3d Pf_L_S = CalcBodyToBaseCoordinates(*model, Q, 11, Vector3d(0., 0., 0.32), UpdateKinematicsFlag);
        Vector3d _3dCouple_L_S_ext = Pf_L_S.cross(_3dForce_L_S_ext);
        _6dForce_ext->at(11) << _3dCouple_L_S_ext,
                                _3dForce_L_S_ext;
        FxFzRL_S_ext << FxFz_R,
                        FxFz_L;
    }else {
        return false;
    }
    return true;
}

bool RobotDynamics_Mario2D :: setExternalForces(const VectorNd &FxFz_R_FxFz_L){
    if (check(FxFz_R_FxFz_L, 4)){
        setExternalForces(FxFz_R_FxFz_L.head(2), FxFz_R_FxFz_L.tail(2));
    }else{
        std::cout << "the external forces should be a 4-dim vector!" << std::endl;
        return false;
    }
    return true;
}

bool RobotDynamics_Mario2D :: setFootContactFlag(int footContactFlag){
    FootContactFlag = footContactFlag;
    return true;
}

bool RobotDynamics_Mario2D::setUpdateKinematicsFlag(bool updateKinematicsFlag){
    UpdateKinematicsFlag = updateKinematicsFlag;
    return true;
}

bool RobotDynamics_Mario2D::getUpdateKinematicsFlag(){
    return isPosVelUpdated;
}

bool RobotDynamics_Mario2D::setWbcDependenceFlag(bool tureOrFlase){
    calcWbcDependence_done = tureOrFlase;
    return true;
}

bool RobotDynamics_Mario2D :: setQddot(const VectorNd &qdd){
    if (check(qdd, NJG)){
        Qddot = qdd;
    }else{
        return false;
    }
    return true;
}

bool RobotDynamics_Mario2D :: setTauActuated(const VectorNd& tau_a){
    if (check(tau_a, NJG)){
        TauA = tau_a;
    }else{
        return false;
    }
    return true;
}

int RobotDynamics_Mario2D :: getFootContactFlag(){
    return FootContactFlag;
}

bool RobotDynamics_Mario2D :: getQddot(VectorNd &qdd){
    qdd = Qddot;
    return true;
}

bool RobotDynamics_Mario2D :: getTauGeneralized(VectorNd &tau_G){
    TauG = Sa.transpose()*TauA;
    tau_G = TauG;
    return true;
}

bool RobotDynamics_Mario2D :: getTauActuated(VectorNd &tau_A){
    tau_A = TauA;
    return true;
}

bool RobotDynamics_Mario2D::calcJacobianInformation(){

    calcJacobianFRpf();
    calcJacobianRRpf();
    calcJacobianFLpf();
    calcJacobianRLpf();
    // public: J_c
    J_c << jacobianRRpf,
           jacobianRLpf;

    calcJdotQdot_FRpf();
    calcJdotQdot_RRpf();
    calcJdotQdot_FLpf();
    calcJdotQdot_RLpf();
    // pubic: JdotQdot_c
    JdotQdot_c << JdotQdot_RRpf,
                  JdotQdot_RLpf;

    return true;
}

bool RobotDynamics_Mario2D :: calcClosedLoopsDynamicallyConsistent_Custom(){

    // public: CS.G
    jacobianLoops << jacobianFRpf - jacobianRRpf,
                     jacobianFLpf - jacobianRLpf;
    CS.G = jacobianLoops;

    // public: CS.gamma
    JdotQdot_Loops << JdotQdot_FRpf - JdotQdot_RRpf,
                      JdotQdot_FLpf - JdotQdot_RLpf;
    CS.gamma = - JdotQdot_Loops;

    CompositeRigidBodyAlgorithm(*model, Q, M, UpdateKinematicsFlag);    // public: M
    CS.H = M;
    NonlinearEffects(*model, Q, Qdot, bng, nullptr);                    // public: bng
    CS.C = bng;

    invM = M.inverse();                                                 // public: invM

    Lambda_G_ic = (CS.G*invM*CS.G.transpose()).inverse();
    invG_DC = invM*CS.G.transpose()*Lambda_G_ic;
    N_ic = MatrixNd::Identity(NJG, NJG) - invG_DC*CS.G;                         // public: N_ic
    c_ic = N_ic.transpose()*bng - CS.G.transpose()*Lambda_G_ic*CS.gamma;        // public: c_ic

    return true;
}

bool RobotDynamics_Mario2D :: calcClosedLoopsDynamicallyConsistent_ConstraintSet(){

    if (LoopFlag == 0){
        std::cout << "There is no ConstraintSet, please treat the model as a Open-Tree, and use the custom function !" << std::endl;
        calcClosedLoopsDynamicallyConsistent_Custom();
        return true;
    }else{
        CalcConstrainedSystemVariables(*model, Q, Qdot, VectorNd::Zero(NJG), CS, nullptr);   // public: model, CS (H, C, G, gamma)
        M = CS.H;                                                                   // public: M
        bng = CS.C;                                                                 // public: bng

        invM = M.inverse();                                                         // public: invM

        Lambda_G_ic = (CS.G*invM*CS.G.transpose()).inverse();
        invG_DC = invM*CS.G.transpose()*Lambda_G_ic;
        N_ic = MatrixNd::Identity(NJG, NJG) - invG_DC*CS.G;                         // public: N_ic
        c_ic = N_ic.transpose()*bng - CS.G.transpose()*Lambda_G_ic*CS.gamma;        // public: c_ic

        return true;
    }
}

bool RobotDynamics_Mario2D :: calcCentroidalDynamicsDescriptors(){

    NonlinearEffects(*model, Q, VectorNd::Zero(NJG), gravityBias, nullptr);  // calculate: gravityBias

    CalcPointJacobian6D(*model, Q, 3, Vector3dZero, jabobian6DFloatingBase, UpdateKinematicsFlag);
    J_FB_Temp = jabobian6DFloatingBase.leftCols(NJF);
    Phi_R.topLeftCorner(3,3) = CalcBodyWorldOrientation(*model, Q, 3, UpdateKinematicsFlag);
    Phi_R.bottomRightCorner(3,3) = Phi_R.topLeftCorner(3,3);
    Phi_Temp = Phi_R*J_FB_Temp;
    Phi.row(0) = Phi_Temp.block(1,0,1,NJF);
    Phi.row(1) = Phi_Temp.block(3,0,1,NJF);
    Phi.row(2) = Phi_Temp.block(5,0,1,NJF);
    Psi = Phi.inverse();

    Mf = Sf*M*Sf.transpose();
    I_locked_B = Psi.transpose() * Mf * (Psi);
    p_CoM_B << -I_locked_B(0,2)/totalMass,
                0.,
                I_locked_B(0,1)/totalMass;
    X_BG_Star_6D.topLeftCorner(3,3) = (Phi_R.topLeftCorner(3,3)).transpose();
    X_BG_Star_6D.bottomRightCorner(3,3) = X_BG_Star_6D.topLeftCorner(3,3);
    X_BG_Star_6D.topRightCorner(3,3) = X_BG_Star_6D.topLeftCorner(3,3) * (skew(p_CoM_B).transpose());
    X_BG_Star << X_BG_Star_6D(1,1), X_BG_Star_6D(1,3), X_BG_Star_6D(1,5),
                 X_BG_Star_6D(3,1), X_BG_Star_6D(3,3), X_BG_Star_6D(3,5),
                 X_BG_Star_6D(5,1), X_BG_Star_6D(5,3), X_BG_Star_6D(5,5);
    T_AG = X_BG_Star * (Psi.transpose()) * Sf;

    AG = T_AG * M;                              // public: AG
    cic_minus_g = c_ic - N_ic.transpose()*gravityBias;
    AGdotQdot = T_AG * cic_minus_g;             // public: AGdotQdot

    return true;
}

bool RobotDynamics_Mario2D :: calcInverseDynamicsDescriptors(){

    invUa = (Sa*N_ic.transpose()*Sa.transpose()).inverse();
    // public: Pa
    Pa.leftCols(NJG) = invUa*Sa*M;
    Pa.rightCols(NFC) = -invUa*Sa*(J_c*N_ic).transpose();
    // public: Qa
    Qa = invUa*Sa*c_ic;

    return true;
}

bool RobotDynamics_Mario2D :: calcCenterOfMassPosVel_useAG(Eigen::VectorXd &com, Eigen::VectorXd &comDot){
    CoM3d = CalcBodyToBaseCoordinates(*model, Q, 3, Vector3dZero, UpdateKinematicsFlag) +
            CalcBodyWorldOrientation(*model, Q, 3, UpdateKinematicsFlag).transpose()*p_CoM_B;
    com << CoM3d(0),
           CoM3d(2);
    comDot = (1./totalMass*AG*Qdot).tail(2);

    return true;
}

bool RobotDynamics_Mario2D :: ForwardDynamics(){
    Qddot = invM*((Sa*N_ic).transpose()*TauA + (J_c*N_ic).transpose()*FxFzRL_S_ext - c_ic);
    return true;
}

bool RobotDynamics_Mario2D :: InverseDynamics(){
    Eigen::VectorXd qddot_fc = Eigen::VectorXd::Zero(NJG + NFC);
    qddot_fc << Qddot,
                FxFzRL_S_ext;
    TauA = Pa*qddot_fc + Qa;
    return true;
}

bool RobotDynamics_Mario2D :: calcJacobianFRpf() {
    CalcPointJacobian(*model, Q, 5, Vector3d(0., 0., 0.32), jacobianFRpfTemp, UpdateKinematicsFlag);
    jacobianFRpf << jacobianFRpfTemp.block(0,0,1,NJG),
                    jacobianFRpfTemp.block(2,0,1,NJG);
    return true;
}

bool RobotDynamics_Mario2D :: calcJacobianRRpf() {
    CalcPointJacobian(*model, Q, 7, Vector3d(0., 0., 0.32), jacobianRRpfTemp, UpdateKinematicsFlag);
    jacobianRRpf << jacobianRRpfTemp.block(0,0,1,NJG),
                    jacobianRRpfTemp.block(2,0,1,NJG);
    return true;
}

bool RobotDynamics_Mario2D :: calcJacobianFLpf() {
    CalcPointJacobian(*model, Q, 9, Vector3d(0., 0., 0.32), jacobianFLpfTemp, UpdateKinematicsFlag);
    jacobianFLpf << jacobianFLpfTemp.block(0,0,1,NJG),
                    jacobianFLpfTemp.block(2,0,1,NJG);
    return true;
}

bool RobotDynamics_Mario2D :: calcJacobianRLpf() {
    CalcPointJacobian(*model, Q, 11, Vector3d(0., 0., 0.32), jacobianRLpfTemp, UpdateKinematicsFlag);
    jacobianRLpf << jacobianRLpfTemp.block(0,0,1,NJG),
                    jacobianRLpfTemp.block(2,0,1,NJG);
    return true;
}

bool RobotDynamics_Mario2D :: calcJacobianRightLoop(){
    calcJacobianFRpf();
    calcJacobianRRpf();
    jacobianRightLoop = jacobianFRpf - jacobianRRpf;
    return true;
}

bool RobotDynamics_Mario2D :: calcJacobianLeftLoop(){
    calcJacobianFLpf();
    calcJacobianRLpf();
    jacobianLeftLoop = jacobianFLpf - jacobianRLpf;
    return true;
}

bool RobotDynamics_Mario2D :: calcJacobianLoops(){
    calcJacobianRightLoop();
    calcJacobianLeftLoop();
    jacobianLoops << jacobianRightLoop,
                    jacobianLeftLoop;
    return true;
}

bool RobotDynamics_Mario2D :: calcJacobianPfRight(){
    if (FootContactFlag != 0){
        calcJacobianRRpf();
        jacobianPfRight = jacobianRRpf;
    }else {
        calcJacobianFRpf();
        jacobianPfRight = jacobianFRpf;
    }
    return true;
}

bool RobotDynamics_Mario2D :: calcJacobianPfLeft(){
    if (FootContactFlag != 0){
        calcJacobianRLpf();
        jacobianPfLeft = jacobianRLpf;
    }else {
        calcJacobianFLpf();
        jacobianPfLeft = jacobianFLpf;
    }
    return true;
}

bool RobotDynamics_Mario2D :: calcJacobianPfRL() {
    calcJacobianPfRight();
    calcJacobianPfLeft();
    jacobianPfRL << jacobianPfRight,
                    jacobianPfLeft;
    return true;
}

bool RobotDynamics_Mario2D :: calcJdotQdot_FRpf(){
    JdotQdot_FRpfTemp = CalcPointAcceleration(*model, Q, Qdot, VectorNd::Zero(NJG), 5, Vector3d(0., 0., 0.32), UpdateKinematicsFlag);
    JdotQdot_FRpf << JdotQdot_FRpfTemp.head(1),
                    JdotQdot_FRpfTemp.tail(1);
    return true;
}

bool RobotDynamics_Mario2D :: calcJdotQdot_RRpf(){
    JdotQdot_RRpfTemp = CalcPointAcceleration(*model, Q, Qdot, VectorNd::Zero(NJG), 7, Vector3d(0., 0., 0.32), UpdateKinematicsFlag);
    JdotQdot_RRpf << JdotQdot_RRpfTemp.head(1),
                    JdotQdot_RRpfTemp.tail(1);
    return true;
}

bool RobotDynamics_Mario2D :: calcJdotQdot_FLpf(){
    JdotQdot_FLpfTemp = CalcPointAcceleration(*model, Q, Qdot, VectorNd::Zero(NJG), 9, Vector3d(0., 0., 0.32), UpdateKinematicsFlag);
    JdotQdot_FLpf << JdotQdot_FLpfTemp.head(1),
                    JdotQdot_FLpfTemp.tail(1);
    return true;
}

bool RobotDynamics_Mario2D :: calcJdotQdot_RLpf(){
    JdotQdot_RLpfTemp = CalcPointAcceleration(*model, Q, Qdot, VectorNd::Zero(NJG), 11, Vector3d(0., 0., 0.32), UpdateKinematicsFlag);
    JdotQdot_RLpf << JdotQdot_RLpfTemp.head(1),
                    JdotQdot_RLpfTemp.tail(1);
    return true;
}

bool RobotDynamics_Mario2D :: calcJdotQdot_RightLoop(){
    calcJdotQdot_FRpf();
    calcJdotQdot_RRpf();
    JdotQdot_RightLoop = JdotQdot_FRpf - JdotQdot_RRpf;
    return true;
}

bool RobotDynamics_Mario2D :: calcJdotQdot_LeftLoop(){
    calcJdotQdot_FLpf();
    calcJdotQdot_RLpf();
    JdotQdot_LeftLoop = JdotQdot_FLpf - JdotQdot_RLpf;
    return true;
}

bool RobotDynamics_Mario2D :: calcJdotQdot_Loops(){
    calcJdotQdot_RightLoop();
    calcJdotQdot_LeftLoop();
    JdotQdot_Loops << JdotQdot_RightLoop,
                        JdotQdot_LeftLoop;
    return true;
}

bool RobotDynamics_Mario2D :: calcJdotQdot_PfRight(){
    if (FootContactFlag != 0){
        calcJdotQdot_RRpf();
        JdotQdot_PfRight = JdotQdot_RRpf;
    }else {
        calcJdotQdot_FRpf();
        JdotQdot_PfRight = JdotQdot_FRpf;
    }
    return true;
}

bool RobotDynamics_Mario2D :: calcJdotQdot_PfLeft(){
    if (FootContactFlag != 0){
        calcJdotQdot_RLpf();
        JdotQdot_PfLeft = JdotQdot_RLpf;
    }else {
        calcJdotQdot_FLpf();
        JdotQdot_PfLeft = JdotQdot_FLpf;
    }
    return true;
}

bool RobotDynamics_Mario2D :: calcJdotQdot_PfRL(){
    calcJdotQdot_PfRight();
    calcJdotQdot_PfLeft();
    JdotQdot_PfRL << JdotQdot_PfRight,
                    JdotQdot_PfLeft;
    return true;
}

MatrixNd RobotDynamics_Mario2D :: skew(const VectorNd& w) {
    sR(0,1) = -w(2);
    sR(0,2) = w(1);
    sR(1,0) = w(2);
    sR(1,2) = -w(0);
    sR(2,0) = -w(1);
    sR(2,1) = w(0);
    return sR;
}

/* ----------------- for multi thread programming -------------------- */

bool RobotDynamics_Mario2D::calcJacobianInformationAndNonlinearEffects(){

    calcJacobianFRpf();
    calcJacobianRRpf();
    calcJacobianFLpf();
    calcJacobianRLpf();
    // public: J_c
    J_c << jacobianRRpf,
           jacobianRLpf;
    // public: CS.G
    jacobianLoops << jacobianFRpf - jacobianRRpf,
                     jacobianFLpf - jacobianRLpf;
    CS.G = jacobianLoops;

    calcJdotQdot_FRpf();
    calcJdotQdot_RRpf();
    calcJdotQdot_FLpf();
    calcJdotQdot_RLpf();
    // pubic: JdotQdot_c
    JdotQdot_c << JdotQdot_RRpf,
                  JdotQdot_RLpf;
    // public: CS.gamma
    JdotQdot_Loops << JdotQdot_FRpf - JdotQdot_RRpf,
                      JdotQdot_FLpf - JdotQdot_RLpf;
    CS.gamma = - JdotQdot_Loops;

    // public: bng
    NonlinearEffects(*model, Q, Qdot, bng, nullptr);
    CS.C = bng;

    return true;
}

bool RobotDynamics_Mario2D::calcInertiaMatrixAndInverse(){

    CompositeRigidBodyAlgorithm(*model, Q, M, UpdateKinematicsFlag);    // public: M
    CS.H = M;
    invM = M.inverse();                                                 // public: invM

    return true;
}

bool RobotDynamics_Mario2D::calcClosedLoopsNullSpaceAndNonlinear(){

    Lambda_G_ic = (CS.G*invM*CS.G.transpose()).inverse();
    invG_DC = invM*CS.G.transpose()*Lambda_G_ic;
    N_ic = MatrixNd::Identity(NJG, NJG) - invG_DC*CS.G;                         // public: N_ic
    c_ic = N_ic.transpose()*bng - CS.G.transpose()*Lambda_G_ic*CS.gamma;        // public: c_ic

    return true;
}

bool RobotDynamics_Mario2D::calcGravityBiasAndCentroidalPsi(){

    NonlinearEffects(*model, Q, VectorNd::Zero(NJG), gravityBias, nullptr);  // calculate: gravityBias
    updateKinematics_AccZero();     // for next controll frame, update body acceleration in RBDL.

    /* --------- Centroidal Dynamics Intermediate variables -------- */
    CalcPointJacobian6D(*model, Q, 3, Vector3dZero, jabobian6DFloatingBase, UpdateKinematicsFlag);
    J_FB_Temp = jabobian6DFloatingBase.leftCols(NJF);
    Phi_R.topLeftCorner(3,3) = CalcBodyWorldOrientation(*model, Q, 3, UpdateKinematicsFlag);
    Phi_R.bottomRightCorner(3,3) = Phi_R.topLeftCorner(3,3);
    Phi_Temp = Phi_R*J_FB_Temp;
    Phi.row(0) = Phi_Temp.block(1,0,1,NJF);
    Phi.row(1) = Phi_Temp.block(3,0,1,NJF);
    Phi.row(2) = Phi_Temp.block(5,0,1,NJF);
    Psi = Phi.inverse();

    return true;
}

bool RobotDynamics_Mario2D::calcInverseDynamicsTerms(){

    invUa = (Sa*N_ic.transpose()*Sa.transpose()).inverse();
    Pa.leftCols(NJG) = invUa*Sa*M;                              // public: Pa
    Pa.rightCols(NFC) = -invUa*Sa*(J_c*N_ic).transpose();       // public: Pa
    Qa = invUa*Sa*c_ic;                                         // public: Qa

    return true;
}

bool RobotDynamics_Mario2D::calcCentroidalDynamicsTerms(){

    Mf = Sf*M*Sf.transpose();
    I_locked_B = Psi.transpose() * Mf * Psi;
    p_CoM_B << -I_locked_B(0,2)/totalMass,
                0.,
                I_locked_B(0,1)/totalMass;
    X_BG_Star_6D.topLeftCorner(3,3) = (Phi_R.topLeftCorner(3,3)).transpose();
    X_BG_Star_6D.bottomRightCorner(3,3) = X_BG_Star_6D.topLeftCorner(3,3);
    X_BG_Star_6D.topRightCorner(3,3) = X_BG_Star_6D.topLeftCorner(3,3) * (skew(p_CoM_B).transpose());
    X_BG_Star << X_BG_Star_6D(1,1), X_BG_Star_6D(1,3), X_BG_Star_6D(1,5),
                 X_BG_Star_6D(3,1), X_BG_Star_6D(3,3), X_BG_Star_6D(3,5),
                 X_BG_Star_6D(5,1), X_BG_Star_6D(5,3), X_BG_Star_6D(5,5);
    T_AG = X_BG_Star * (Psi.transpose()) * Sf;

    AG = T_AG * M;                              // public: AG
    cic_minus_g = c_ic - N_ic.transpose()*gravityBias;
    AGdotQdot = T_AG * cic_minus_g;             // public: AGdotQdot

    return true;
}

/* ----------------- for multi thread programming -------------------- */

