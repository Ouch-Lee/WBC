/*
 *  Header for Webots
 */
 
#ifndef WEBOTS_INTERFACE_H
#define WEBOTS_INTERFACE_H

#ifndef PI
    #define PI 3.141592654
#endif // PI

#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <webots/GPS.hpp>
#include <webots/Node.hpp>

#include <vector>
#include <Eigen/Dense>

// Rotation Matrix 2D
template <typename T>
using RotMat2 = typename Eigen::Matrix<T, 2, 2>;

// Rotation Matrix 3D
template <typename T>
using RotMat = typename Eigen::Matrix<T, 3, 3>;

// 2x1 Vector
template <typename T>
using Vec2 = typename Eigen::Matrix<T, 2, 1>;

// 3x1 Vector
template <typename T>
using Vec3 = typename Eigen::Matrix<T, 3, 1>;

// 4x1 Vector
template <typename T>
using Vec4 = typename Eigen::Matrix<T, 4, 1>;

// 6x1 Vector
template <typename T>
using Vec6 = Eigen::Matrix<T, 6, 1>;

// 10x1 Vector
template <typename T>
using Vec10 = Eigen::Matrix<T, 10, 1>;

// 12x1 Vector
template <typename T>
using Vec12 = Eigen::Matrix<T, 12, 1>;

// 18x1 Vector
template <typename T>
using Vec18 = Eigen::Matrix<T, 18, 1>;

// 28x1 vector
template <typename T>
using Vec28 = Eigen::Matrix<T, 28, 1>;

// 3x3 Matrix
template <typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;

// 4x1 Vector
template <typename T>
using Quat = typename Eigen::Matrix<T, 4, 1>;

// Spatial Vector (6x1, all subspaces)
template <typename T>
using SVec = typename Eigen::Matrix<T, 6, 1>;

// Spatial Transform (6x6)
template <typename T>
using SXform = typename Eigen::Matrix<T, 6, 6>;

// 6x6 Matrix
template <typename T>
using Mat6 = typename Eigen::Matrix<T, 6, 6>;

// 12x12 Matrix
template <typename T>
using Mat12 = typename Eigen::Matrix<T, 12, 12>;

// 18x18 Matrix
template <typename T>
using Mat18 = Eigen::Matrix<T, 18, 18>;

// 28x28 Matrix
template <typename T>
using Mat28 = Eigen::Matrix<T, 28, 28>;

// 3x4 Matrix
template <typename T>
using Mat34 = Eigen::Matrix<T, 3, 4>;

// 3x4 Matrix
template <typename T>
using Mat23 = Eigen::Matrix<T, 2, 3>;

// 4x4 Matrix
template <typename T>
using Mat4 = typename Eigen::Matrix<T, 4, 4>;

// 10x1 Vector
template <typename T>
using MassProperties = typename Eigen::Matrix<T, 10, 1>;

// Dynamically sized vector
template <typename T>
using DVec = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;

// Dynamically sized matrix
template <typename T>
using DMat = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

// Dynamically sized matrix with spatial vector columns
template <typename T>
using D6Mat = typename Eigen::Matrix<T, 6, Eigen::Dynamic>;

// Dynamically sized matrix with cartesian vector columns
template <typename T>
using D3Mat = typename Eigen::Matrix<T, 3, Eigen::Dynamic>;

// std::vector (a list) of Eigen things
template <typename T>
using vectorAligned = typename std::vector<T, Eigen::aligned_allocator<T>>;


using namespace webots;

#define TIME_STEP    (1)
#define TORQUE_LIMIT (300.0f)
#define MOTOR_LIMIT  (3*PI)
#define SAMPLE_TIME  (0.001f)

/* motor names */
typedef enum
{
    FR_HIP = 0x00,
    RR_HIP = 0x01,
    FL_HIP = 0x02,
    RL_HIP = 0x03,
}motorNameTypeDef;

/* leg names */
typedef enum
{
    R = 0x00,
    L = 0x01,
}legNameTypeDef;

/* position sensor names */
typedef enum
{
    Pla_Cro = 0x00,
    Cro_Pin = 0x01,
    Pin_Tor = 0x02,
    FR_Hip = 0x03,
    FR_Kne = 0x04,
    RR_Hip = 0x05,
    RR_Kne = 0x06,
    FL_Hip = 0x07,
    FL_Kne = 0x08,
    RL_Hip = 0x09,
    RL_Kne = 0x10,
}sensorNameTypeDef;

struct webotState
{
    Vec4<double> jointPosAct = Vec4<double>::Zero();
    Vec4<double> jointPosAct_prev = Vec4<double>::Zero();
    Vec4<double> jointVelAct = Vec4<double>::Zero();
    Vec4<double> jointTorAct = Vec4<double>::Zero();
    Vec6<double> footGrfAct = Vec6<double>::Zero();
    Vec3<double> torsoPosAct = Vec3<double>::Zero();
    Vec3<double> torsoPosAct_prev = Vec3<double>::Zero();
    Vec3<double> torsoLivAct = Vec3<double>::Zero();
    Vec3<double> torsoRpyAct = Vec3<double>::Zero();
    Vec3<double> torsoAnvAct = Vec3<double>::Zero();
};

struct webotDesired
{
    Vec4<double> jointPosDes = Vec4<double>::Zero();
    Vec4<double> jointPosDes_prev = Vec4<double>::Zero();
    Vec4<double> jointVelDes = Vec4<double>::Zero();
    Vec4<double> jointTorDes = Vec4<double>::Zero();
    Vec4<double> jointPpidDes = Vec4<double>::Zero();
    Vec4<double> jointIpidDes = Vec4<double>::Zero();
    Vec4<double> jointDpidDes = Vec4<double>::Zero();
    Vec6<double> footGrfDes = Vec6<double>::Zero();
    Vec3<double> torsoPosDes = Vec3<double>::Zero();
    Vec3<double> torsoPosDes_prev = Vec3<double>::Zero();
    Vec3<double> torsoLivDes = Vec3<double>::Zero();
    Vec3<double> torsoRpyDes = Vec3<double>::Zero();
    Vec3<double> torsoAnvDes = Vec3<double>::Zero();
};

class webotsRobot
{
  public:

    /* handle */
//    Robot *robot = new Robot();
    Supervisor *robot = new Supervisor();

    void initWebots();
    void deleteRobot();

    bool readData(double simTime, webotState &quadState);

    bool sendTorCammand(Vec4<double> jointTorTar);
    bool sendPosCammand(Vec4<double> jointPosTar);

    bool applyUpperBodyForce(Vec3<double> ubForce);

  private:
    
    /* motors */
    Motor *FR_HIP_motor;
    Motor *RR_HIP_motor;
    Motor *FL_HIP_motor;
    Motor *RL_HIP_motor;


    /* encoders */
    PositionSensor *Pla_Cro_pos_sensor;
    PositionSensor *Cro_Pin_pos_sensor;
    PositionSensor *Pin_Tor_pos_sensor;

    PositionSensor *FR_Hip_pos_sensor;
    PositionSensor *FR_Kne_pos_sensor;

    PositionSensor *RR_Hip_pos_sensor;
    PositionSensor *RR_Kne_pos_sensor;

    PositionSensor *FL_Hip_pos_sensor;
    PositionSensor *FL_Kne_pos_sensor;

    PositionSensor *RL_Hip_pos_sensor;
    PositionSensor *RL_Kne_pos_sensor;

    /* touch sensors */
    TouchSensor *R_touch_sensor;
    TouchSensor *L_touch_sensor;

    /* IMU */
    InertialUnit *imu;
    /* Gyro */
    Gyro *gyro;
    /* GPS */
    GPS *gps;

    /* Supervisor Add Force */
    Node * upperBody_node;

    /* Actuated Motors*/
    double getMotorPos(motorNameTypeDef motorName);
    double getMotorVel(motorNameTypeDef motorName);
    double getMotorTorque(motorNameTypeDef motorName);

    /* Position Sensors*/
    double getSensorPos(sensorNameTypeDef sensorName);
//    double getSensorVel(sensorNameTypeDef sensorName);

    Vec4<double> getActMotorPos();
    Vec4<double> getActMotorVel();
    Vec4<double> getActMotorTor();

    RotMat<double> Roty(double theta);
    Vec3<double> getGRF(legNameTypeDef legName);
    Vec6<double> getGRFs_W();

    Vec3<double> getTorsoOrientation();
    Vec3<double> getTorsoAngularVelocity();
    Vec3<double> getTorsoPosition();
    Vec3<double> getTorsoVel();

    bool setMotorTorque(motorNameTypeDef motorName, double torque);
    bool setMotorPosition(motorNameTypeDef motorName, double position);

};


#endif

