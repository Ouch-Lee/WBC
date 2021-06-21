
#include "webotsInterface.h"

using namespace webots;

/*------------------------------------PUBLIC---------------------------------------------*/

/*
Initialize Webots
*/ 
void webotsRobot::initWebots()
{
    FR_HIP_motor = robot->getMotor("FR_rotational_motor");
    RR_HIP_motor = robot->getMotor("RR_rotational_motor");
    FL_HIP_motor = robot->getMotor("FL_rotational_motor");
    RL_HIP_motor = robot->getMotor("RL_rotational_motor");

    Pla_Cro_pos_sensor = robot->getPositionSensor("PlaCro_prismatic_sensor");
    Cro_Pin_pos_sensor = robot->getPositionSensor("CroPin_prismatic_sensor");
    Pin_Tor_pos_sensor = robot->getPositionSensor("PinTor_rotational_sensor");

    FR_Hip_pos_sensor = robot->getPositionSensor("FR_rotational_sensor");
    FR_Kne_pos_sensor = robot->getPositionSensor("FR_knee_rotational_sensor");

    RR_Hip_pos_sensor = robot->getPositionSensor("RR_rotational_sensor");
    RR_Kne_pos_sensor = robot->getPositionSensor("RR_knee_rotational_sensor");

    FL_Hip_pos_sensor = robot->getPositionSensor("FL_rotational_sensor");
    FL_Kne_pos_sensor = robot->getPositionSensor("FL_knee_rotational_sensor");

    RL_Hip_pos_sensor = robot->getPositionSensor("RL_rotational_sensor");
    RL_Kne_pos_sensor = robot->getPositionSensor("RL_knee_rotational_sensor");

    R_touch_sensor = robot->getTouchSensor("Right_touch_sensor");
    L_touch_sensor = robot->getTouchSensor("Left_touch_sensor");

    imu = robot->getInertialUnit("inertial_unit");
    gyro = robot->getGyro("gyro");
    gps = robot->getGPS("gps");

    upperBody_node = robot->getFromDef("Diamond2d");

    Pla_Cro_pos_sensor->enable(TIME_STEP);
    Cro_Pin_pos_sensor->enable(TIME_STEP);
    Pin_Tor_pos_sensor->enable(TIME_STEP);

    FR_Hip_pos_sensor->enable(TIME_STEP);
    FR_Kne_pos_sensor->enable(TIME_STEP);

    RR_Hip_pos_sensor->enable(TIME_STEP);
    RR_Kne_pos_sensor->enable(TIME_STEP);

    FL_Hip_pos_sensor->enable(TIME_STEP);
    FL_Kne_pos_sensor->enable(TIME_STEP);

    RL_Hip_pos_sensor->enable(TIME_STEP);
    RL_Kne_pos_sensor->enable(TIME_STEP);

    R_touch_sensor->enable(TIME_STEP);
    L_touch_sensor->enable(TIME_STEP);

    imu->enable(TIME_STEP);
    gyro->enable(TIME_STEP);
    gps->enable(TIME_STEP);

}

/*
 * Description:   Release memory
*/
void webotsRobot::deleteRobot()
{
    delete robot;
}

/*
 * Description:   Read data to robot class
*/
bool webotsRobot::readData(double simTime, webotState & bipedState)
{
    // Motor pos
    bipedState.jointPosAct_prev = bipedState.jointPosAct;
    bipedState.jointPosAct(0, 0) = getMotorPos(FR_HIP);
    bipedState.jointPosAct(1, 0) = getMotorPos(RR_HIP);
    bipedState.jointPosAct(2, 0) = getMotorPos(FL_HIP);
    bipedState.jointPosAct(3, 0) = getMotorPos(RL_HIP);
    //Motor vel
    if (simTime > SAMPLE_TIME)
    {
      bipedState.jointVelAct = (bipedState.jointPosAct - bipedState.jointPosAct_prev) / SAMPLE_TIME;
    }
    else
    {
      bipedState.jointVelAct = Vec4<double>::Zero();
    }
    //Motor torque
    bipedState.jointTorAct(0, 0) = getMotorTorque(FR_HIP);
    bipedState.jointTorAct(1, 0) = getMotorTorque(RR_HIP);
    bipedState.jointTorAct(2, 0) = getMotorTorque(FL_HIP);
    bipedState.jointTorAct(3, 0) = getMotorTorque(RL_HIP);
    //Foot GRF
    bipedState.footGrfAct = getGRFs_W();
    //Torso state
    bipedState.torsoPosAct_prev = bipedState.torsoPosAct;
    bipedState.torsoPosAct = getTorsoPosition();
    if (simTime > SAMPLE_TIME)
    {
        bipedState.torsoLivAct = (bipedState.torsoPosAct - bipedState.torsoPosAct_prev) / SAMPLE_TIME;
    }
    else
    {
        bipedState.torsoLivAct = Vec3<double>::Zero();
    }
    bipedState.torsoRpyAct = getTorsoOrientation();
    bipedState.torsoAnvAct = getTorsoAngularVelocity();

    return true;
}

/*
 * Description:   Send torque cammand to Webots
*/
bool webotsRobot::sendTorCammand(Vec4<double> jointTor_Cmd)
{
    //Motor torque
    setMotorTorque(FR_HIP, jointTor_Cmd(0, 0));
    setMotorTorque(RR_HIP, jointTor_Cmd(1, 0));
    setMotorTorque(FL_HIP, jointTor_Cmd(2, 0));
    setMotorTorque(RL_HIP, jointTor_Cmd(3, 0));

    return true;
}

/*
 * Description:   Send pos cammand to Webots
*/
bool webotsRobot::sendPosCammand(Vec4<double> jointPos_Cmd)
{
    //Motor pos
    setMotorPosition(FR_HIP, jointPos_Cmd(0, 0));
    setMotorPosition(RR_HIP, jointPos_Cmd(1, 0));
    setMotorPosition(FL_HIP, jointPos_Cmd(2, 0));
    setMotorPosition(RL_HIP, jointPos_Cmd(3, 0));

    return true;
}

/*
 * Description:   Apply external force to upper-body
*/
bool webotsRobot::applyUpperBodyForce(const Vec3<double> ubForce){
    const double force[3] = {ubForce(0), ubForce(1), ubForce(2)};
    upperBody_node->addForce(force, false);
    return true;
}

/*------------------------------------PRIVATE---------------------------------------------*/

/*
 * Description:   Get the motor position in radians from joint encoder
 */
double webotsRobot::getMotorPos(motorNameTypeDef motorName)
{
    double angle = 0;
    switch (motorName){
    case FR_HIP:  {  angle = FR_Hip_pos_sensor->getValue(); break;  }
    case RR_HIP:  {  angle = RR_Hip_pos_sensor->getValue(); break;  }
    case FL_HIP:  {  angle = FL_Hip_pos_sensor->getValue(); break;  }
    case RL_HIP:  {  angle = RL_Hip_pos_sensor->getValue(); break;  }

    default:break;
    }
    return angle;
}

/*
 * Description:   Get the motor velocity in radians from joint encoder
 * This way is not recommended!
 */
double webotsRobot::getMotorVel(motorNameTypeDef motorName)
{
    double vel = 0;
    switch (motorName){
    case FR_HIP:  {  vel = FR_HIP_motor->getVelocity(); break;  }
    case RR_HIP:  {  vel = RR_HIP_motor->getVelocity(); break;  }
    case FL_HIP:  {  vel = FL_HIP_motor->getVelocity(); break;  }
    case RL_HIP:  {  vel = RL_HIP_motor->getVelocity(); break;  }

    default:break;
    }
    return vel;
}

/*
 * Description:   Get the motor torque in radians from joint encoder
 */
double webotsRobot::getMotorTorque(motorNameTypeDef motorName)
{
    double torque = 0;
    switch (motorName){
    case FR_HIP:  {  torque = FR_HIP_motor->getTorqueFeedback(); break;  }
    case RR_HIP:  {  torque = RR_HIP_motor->getTorqueFeedback(); break;  }
    case FL_HIP:  {  torque = FL_HIP_motor->getTorqueFeedback(); break;  }

    default:break;
    }
    return torque;
}


/*
 * Description:   Get the joint position in radians from joint encoder
 */
double webotsRobot::getSensorPos(sensorNameTypeDef sensorName)
{
    double pos = 0;
    switch (sensorName){
    case Pla_Cro:   {  pos = Pla_Cro_pos_sensor->getValue(); break;  }
    case Cro_Pin:   {  pos = Cro_Pin_pos_sensor->getValue(); break;  }
    case Pin_Tor:   {  pos = Pin_Tor_pos_sensor->getValue(); break;  }
    case FR_Hip:  {  pos = FR_Hip_pos_sensor->getValue(); break;  }
    case FR_Kne:  {  pos = FR_Kne_pos_sensor->getValue(); break;  }
    case RR_Hip:  {  pos = RR_Hip_pos_sensor->getValue(); break;  }
    case RR_Kne:  {  pos = RR_Kne_pos_sensor->getValue(); break;  }
    case FL_Hip:  {  pos = FL_Hip_pos_sensor->getValue(); break;  }
    case FL_Kne:  {  pos = FL_Kne_pos_sensor->getValue(); break;  }
    case RL_Hip:  {  pos = RL_Hip_pos_sensor->getValue(); break;  }
    case RL_Kne:  {  pos = RL_Kne_pos_sensor->getValue(); break;  }

    default:break;
    }
    return pos;
}

/*
 * Description:   Get motors pos of robot
 */
Vec4<double> webotsRobot::getActMotorPos()
{
    Vec4<double> angle;
    angle << getMotorPos(FR_HIP), getMotorPos(RR_HIP), getMotorPos(FL_HIP), getMotorPos(RL_HIP);
    return angle;
}

/*
 * Description:   Get motors vel of robot
 */
Vec4<double> webotsRobot::getActMotorVel()
{
    Vec4<double> vel;
    vel << getMotorVel(FR_HIP), getMotorVel(RR_HIP), getMotorVel(FL_HIP), getMotorVel(RL_HIP);
    return vel;
}

/*
 * Description:   Get motors torque of robot
 */
Vec4<double> webotsRobot::getActMotorTor()
{
    Vec4<double> torque;
    torque << getMotorTorque(FR_HIP), getMotorTorque(RR_HIP), getMotorTorque(FL_HIP), getMotorTorque(RL_HIP);
    return torque; 
}


/*
 * Description:   get the RotMat about Y(pitch)
 */
RotMat<double> webotsRobot::Roty(double theta){
    RotMat<double> res;
    res << std::cos(theta), 0.0,  std::sin(theta),
            0.0, 1.0, 0.0,
           -std::sin(theta), 0.0, std::cos(theta);
    return res;
}

/*
 * Description:   Ground Reactive Forces
 */
Vec3<double> webotsRobot::getGRF(legNameTypeDef legName)
{
    Vec3<double> GRF = Vec3<double>::Zero();
    switch (legName){
    case R:{
        const double* data = R_touch_sensor->getValues();
        GRF << data[0], data[1], data[2];
        break;
    }
    case L:{
        const double* data = L_touch_sensor->getValues();
        GRF << data[0], data[1], data[2];
        break;
    }
    default:{
        break;
    }
    }
    return GRF;
}

/*
 * Description:   Ground Reactive Forces in World Frame
 */
Vec6<double> webotsRobot::getGRFs_W(){
    Vec6<double> GRFs;
    // for Right-leg
    double theta1R = Pin_Tor_pos_sensor->getValue();
    double theta2R = RR_Hip_pos_sensor->getValue();
    double theta3R = RR_Kne_pos_sensor->getValue();
    GRFs.block<3, 1>(0, 0) = Roty(theta1R)*Roty(theta2R)*Roty(theta3R)*getGRF(R);
    // for Left-leg
    double theta1L = Pin_Tor_pos_sensor->getValue();
    double theta2L = RL_Hip_pos_sensor->getValue();
    double theta3L = RL_Kne_pos_sensor->getValue();
    GRFs.block<3, 1>(3, 0) = Roty(theta1L)*Roty(theta2L)*Roty(theta3L)*getGRF(L);
    return GRFs;
}

/*
 * Description:   Get roll pitch yaw in radians from IMU data
 */
Vec3<double> webotsRobot::getTorsoOrientation()
{
//    const double* data = imu->getRollPitchYaw();
    Vec3<double> eulerAngle(0, Pin_Tor_pos_sensor->getValue(), 0);
    return eulerAngle;
}

/*
 * Description:   Get roll pitch yaw in radians per second from Gyro data
 */
Vec3<double> webotsRobot::getTorsoAngularVelocity()
{
    const double* data = gyro->getValues();
    Vec3<double> eulerAngleRate(data[0], data[1], data[2]);
    return eulerAngleRate;
}

/*
 * Description:   Get global torso position from GPS data
 */
Vec3<double> webotsRobot::getTorsoPosition()
{
    const double* data = gps->getValues();
    Vec3<double> pos(data[0],data[1],data[2]);
    return pos;
}

/*
 * Description:   Compute global torso velocity (To be developped)
 */
Vec3<double> webotsRobot::getTorsoVel()
{
    Vec3<double> vel = Vec3<double>::Zero();
    return vel;
}


/*
 * Description:   Set the motor torque to each joint
 */
bool webotsRobot::setMotorTorque(motorNameTypeDef motorName, double torque)
{
    // threshold
    if(torque >  TORQUE_LIMIT) torque =  TORQUE_LIMIT;
    if(torque < -TORQUE_LIMIT) torque = -TORQUE_LIMIT;

    switch (motorName){
    case FR_HIP:  {  FR_HIP_motor->setTorque(torque); break;  }
    case RR_HIP:  {  RR_HIP_motor->setTorque(torque); break;  }
    case FL_HIP:  {  FL_HIP_motor->setTorque(torque); break;  }
    case RL_HIP:  {  RL_HIP_motor->setTorque(torque); break;  }

    default:break;
    }

    return true;
}

/*
 * Description:   Set the motors position
 */
bool webotsRobot::setMotorPosition(motorNameTypeDef motorName, double position)
{
    // threshold
    if(position >  MOTOR_LIMIT) position =  MOTOR_LIMIT;
    if(position < -MOTOR_LIMIT) position = -MOTOR_LIMIT;

    switch (motorName){
    case FR_HIP:  {  FR_HIP_motor->setPosition(position); break;  }
    case RR_HIP:  {  RR_HIP_motor->setPosition(position); break;  }
    case FL_HIP:  {  FL_HIP_motor->setPosition(position); break;  }
    case RL_HIP:  {  RL_HIP_motor->setPosition(position); break;  }

    default:break;
    }

    return true;
}

