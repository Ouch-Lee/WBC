/*
 * created by Jun, 2020-07-02
 */

#ifndef DIAMONDKINEMATICS_H
#define DIAMONDKINEMATICS_H

#include <iostream>
#include <vector>
#include <cmath>        // std::fabs(), atan(), sin(), cos()
#include <algorithm>    // std::max() & min()
#include <chrono>       // std::chrono::system_clock::now()
#include <stdlib.h>     // Generate random numbers
#include <Eigen/Dense>

namespace DiaKine {

/*
 * DK2Vec, a struct contains two Eigen::VectorXd, e.g. theta and thetadot
 */
struct DK2Vec{
    Eigen::VectorXd Q1;
    Eigen::VectorXd Q2;
    DK2Vec& operator= ( const DK2Vec& foo);
};

/*
 * DK3Vec, a struct contains three Eigen::VectorXd, e.g. theta thetadot and thetaddot
 */
struct DK3Vec{
    Eigen::VectorXd Q1;
    Eigen::VectorXd Q2;
    Eigen::VectorXd Q3;
    DK3Vec& operator= ( const DK3Vec& foo);
};

/*
 * planar 5 bar mechanism, two are driven motors, the other three are passive joints
 */
class fiveBarKine{
public:
    Eigen::Vector2d fkPlanar5bar(const Eigen::Vector2d& theta, const Eigen::Vector3d& lparamsThree);
    Eigen::Vector2d ikPlanar5bar(const Eigen::Vector2d& endXZ, const Eigen::Vector3d& lparamsThree);
    Eigen::Matrix2d jacoPlanar5bar(const Eigen::Vector2d& theta, const Eigen::Vector3d& lparamsThree);
    // TODO:
//    Eigen::Matrix2d jacoDotPlanar5bar(const Eigen::Vector2d& theta, const Eigen::Vector2d& thetaDot, const Eigen::Vector3d& lparamsThree);
private:
    DK2Vec ikPlanar2bar(const Eigen::Vector2d& endXY, const Eigen::Vector2d& lparamsTwo);
    double pi{3.141592654};
};

/*
** DiamondLeg3D class contains all the functions needed for diamond leg 3D applications
** DiamondLeg2D class contains all the functions needed for diamond leg 2D applications

    for 3D application:
        xyz = rotx(phi) * roty(theta) * [r;0;0]
        x =  r*cos(theta)
        y =  r*sin(theta)*sin(phi)
        z = -r*sin(theta)*cos(phi)
        CartesianCoord: 3*1, [x, y, z]'
        SphericalCoord: 3*1, [r, theta, phi]' ; theta range [0,pi); phi range [-pi, pi]
    for 2D application:
        CartesianCoord: 2*1, [x, z]'
        PolarCoord:     2*1, [r, theta]' ; theta range [0,pi);
*/

class DiamondLeg3D{
public:
    DiamondLeg3D();
    DiamondLeg3D(const Eigen::Vector3d& lengthParamsThree);
    ~DiamondLeg3D();
    Eigen::Vector3d cartes2Polar3D(const Eigen::Vector3d& xyz);
    Eigen::Vector3d polar2Cartes3D(const Eigen::Vector3d& rtp);
    // TODO:
//    Eigen::Vector3d fkPosCartes3D(const Eigen::Vector3d& theta);
//    Eigen::Vector3d fkPosPolar3D(const Eigen::Vector3d& theta);
//    Eigen::Vector3d ikPosCartes3D(const Eigen::Vector3d& endXYZ);
//    Eigen::Vector3d ikPosPolar3D(const Eigen::Vector3d& endRTP);
//    Eigen::Matrix3d jacoCartes3D(const Eigen::Vector3d& theta);
//    Eigen::Matrix3d jacoPolar3D(const Eigen::Vector3d& theta);
//    Eigen::Matrix3d jacoDotCartes3D(const Eigen::Vector3d& theta, const Eigen::Vector3d& thetaDot);
//    Eigen::Matrix3d jacoDotPolar3D(const Eigen::Vector3d& theta, const Eigen::Vector3d& thetaDot);
//    Eigen::Vector3d forceCartes2Polar3D(const Eigen::Vector3d& forceXYZ, const Eigen::Vector3d& xyz);
//    Eigen::Vector3d forcePolar2Cartes3D(const Eigen::Vector3d& forceRTP, const Eigen::Vector3d& rtp);
private:
    Eigen::Vector3d lparams;
    double pi{3.141592654};
    double atanZeroPI(double x);
};

/*
 *  for DiamondLeg2D, the joint space Zero-position :
 *
 *      |   |
 *      |   |
 *      |   |       --> shin
 *      |   |
 *      @   @       --> KNEE, passive: [q_knee_fore, q_knee_rear]'
 *      |   |
 *      |   |       --> thigh
 *      @   @       --> HIP, actuated: [q_hip_fore, q_hip_rear]'
 *    =========     --> torso
 * (rear) | (fore)
 *
 *       /\
 *     z ||  => x
 *
 */
class DiamondLeg2D{
public:
    DiamondLeg2D();
    DiamondLeg2D(const Eigen::Vector3d& lengthParamsThree);
    ~DiamondLeg2D();
    void setLinkParams(const Eigen::Vector3d& lengthParamsThree);

    // Actuated Joints Kinematics
    Eigen::Vector2d cartes2Polar2D(const Eigen::Vector2d& xz);
    Eigen::Vector2d polar2Cartes2D(const Eigen::Vector2d& rt);
    Eigen::Vector2d fkPosCartes2D(const Eigen::Vector2d& theta);
    Eigen::Vector2d fkPosPolar2D(const Eigen::Vector2d& theta);
    Eigen::Vector2d ikPosCartes2D(const Eigen::Vector2d& endXZ);
    Eigen::Vector2d ikPosPolar2D(const Eigen::Vector2d& endRT);
    Eigen::Matrix2d jacoCartes2D(const Eigen::Vector2d& theta);
    Eigen::Matrix2d jacoPolar2D(const Eigen::Vector2d& theta);
    // TODO:
//    Eigen::Vector2d forceCartes2Polar2D(const Eigen::Vector2d& forceXZ, const Eigen::Vector2d& xz);
//    Eigen::Vector2d forcePolar2Cartes2D(const Eigen::Vector2d& forceRT, const Eigen::Vector2d& rt);

    // Passive-Tree HipKnee Kinematics
    Eigen::Vector2d fkPosCartes2D_HipKnee(const Eigen::Vector2d& q_HipKnee);
    Eigen::Matrix2d jacoCartes2D_HipKnee(const Eigen::Vector2d& q_HipKnee);
    void calHipKnee2D(const Eigen::Vector2d &q_hip_ForeRear, const Eigen::Vector2d &qdot_hip_ForeRear,
                      Eigen::Vector2d &q_HipKnee_fore, Eigen::Vector2d &q_HipKnee_rear,
                      Eigen::Vector2d &qdot_HipKnee_fore, Eigen::Vector2d &qdot_HipKnee_rear);
    void calHipKnee2D(const Eigen::Vector2d &q_hip_ForeRear, const Eigen::Vector2d &qdot_hip_ForeRear,
                      const Eigen::Vector2d &xz_foot, const Eigen::Vector2d &xzDot_foot,
                      double &q_Knee_fore, double &q_Knee_rear,
                      double &qdot_Knee_fore, double &qdot_Knee_rear);

    // Given Cartesian coordinates in Support Point-foot Frame, calculate generalized coordinates.
    void Cart2Config(const Eigen::Vector3d& xzp_base, const Eigen::Vector2d& xz_pfR, const Eigen::Vector2d& xz_pfL,
                     const Eigen::Vector3d& xzp_dot_base, const Eigen::Vector2d& xz_dot_pfR, const Eigen::Vector2d& xz_dot_pfL,
                     Eigen::VectorXd& q_Actuated, Eigen::VectorXd& q, Eigen::VectorXd& qDot);
    // Given generalized Cartesian forces in 'W' Frame, calculate actuated joint torque.
    void Force2Torque(double pitch, const Eigen::Vector2d &q_hip_ForeRear, const Eigen::Vector2d &Fc_W,
                      Eigen::Vector2d &tau_ForeRear);

private:
    DiamondLeg3D* pointer3D;
    fiveBarKine* pointer5bar;
    Eigen::Vector3d lparams;
    double pi{3.141592654};
};


// ============================= Test Functions ===================================
bool KinematicsTest1();         // Single: Test a full-function
bool KinematicsTest2(int num);  // Random: Actuated Joints Kinematics Test
bool KinematicsTest3(int num);  // Random: Passive-Tree HipKnee Kinematics + Actuated Joints Kinematics Test

// ---------- Actuated Joints Kinematics Test ---------------------------
// fk : actual qa, dqa --> cartesian [x, z],[dx, dz]
bool fkAct2Cart(const Eigen::Vector2d &qAct, const Eigen::Vector2d &qdAct,
                Eigen::Vector2d &xz, Eigen::Vector2d &xdzd);
// ik : cart [x, z],[dx, dz] --> desired q, dq;
bool ikCart2Des(const Eigen::Vector2d &xz, const Eigen::Vector2d &xdzd,
                Eigen::Vector2d &q, Eigen::Vector2d &qd);
// ------- end of Actuated Joints Kinematics Test -----------------------

// ------------------------- tools ----------------------------
bool isVecEqual(const Eigen::VectorXd & vec_a, const Eigen::VectorXd & vec_b, const double epsilon);

// ========================== end of Test Functions ================================

}

#endif // DIAMONDKINEMATICS_H
