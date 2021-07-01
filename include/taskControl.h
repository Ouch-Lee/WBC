//
// Created by jun on 2020-8-19.
//

#ifndef TASK_CONTROL_H
#define TASK_CONTROL_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>       // std::chrono::system_clock::now()

#include "robotMessage.h"
#include "planToolkit.h"
#include "DiamondKinematics.h"

#include "wqpWbc.h"
#include "taskDefinition_Mario2D.h"
#include "constraintDefinition_Mario2D.h"
#include "robotDynamics_Mario2D.h"


class taskControl{
public:
    taskControl(RobotDynamics_Mario2D * robotRBDM, int qpFlag);
    ~taskControl();

    bool setParameters(const robotStateMachine & _rsm);

    bool walkCtrl(const robotStateMachine & _rsm, const robotState & _rs, const robotDesired & _rd, robotTaskRef & _rtr);

    bool walkCtrl_wqp(const robotStateMachine & _rsm, const robotState & _rs, const robotDesired & _rd, robotTaskRef & _rtr);
    bool walkCtrl_hqp(const robotStateMachine & _rsm, const robotState & _rs, const robotDesired & _rd, robotTaskRef & _rtr);


private:

    int qp_flag_{1};          // 0 : wqp_WBC; 1 : hqp_WBC

    RobotDynamics_Mario2D * robot_;
    Wbc * wbc_;
    Wbc * wbch_;

    // fk, ik, jac of leg
    DiaKine::DiamondLeg2D * dl2D;

    int nJg{11};            // dimension of Generalized Coordinates, 11
    int nJf{3};             // dimension of floating-base Joints, 3
    int nJa{4};             // dimension of Actuated Joints, 4
    int nJp{4};             // dimension of Passive (knee) Joints, 4
    int nFc{4};             // dimension of Contact Forces, 4

    int nV{nJg + nFc};      // dimension of Variables, nV = nJg + nFc, 15
    int nO{0};              // dimension of Objects/Costs
    int nC{0};              // dimension of Constraints

    double Iy_ub{0.26};
    double m_total{24.5};
    double m_leg{3.5};

    // the PD of pitch
    double kp_pitch{0.};  // 2400, 624
    double kd_pitch{0.};  // 70, 18.2
    // the PD of centroidal linear Moment
    std::vector<double> stdVec_kp_com{0., 0.};
    std::vector<double> stdVec_kd_com{0., 0.};
    // the PD of Cartesian Upper Body
    std::vector<double> stdVec_kp_U{0., 0.};
    std::vector<double> stdVec_kd_U{0., 0.};
    // the PD of c_point-feet
    std::vector<double> stdVec_kp_cPfDDot_R{0., 0.};
    std::vector<double> stdVec_kd_cPfDDot_R{0., 0.};
    std::vector<double> stdVec_ki_cPfDDot_R{0., 0.};
    std::vector<double> stdVec_kp_cPfDDot_L{0., 0.};
    std::vector<double> stdVec_kd_cPfDDot_L{0., 0.};
    std::vector<double> stdVec_ki_cPfDDot_L{0., 0.};

    // for Integrator of cartesian-point-foot
    Eigen::Vector2d integral_cPfError_R = Eigen::Vector2d::Zero(), integral_cPfError_L = Eigen::Vector2d::Zero();

    // tau_cmd = tau_opt + k_qDot_integral*(qDot_star - qDot_estimate)
    std::vector<double> stdVec_k_sw_qDot_integral{0., 0.};

    // Parameters
    double mu_static_{0.9};
    double jointTau_limit_{35.};
    double jointQdot_limit_{12.};                       // actuator Velocity limit (rad/s)
    double jointQddot_limit_{1e4};
    double myInfinity_{1e6};
    double delta_t_{0.05};	// 0.02 for VelTrk; 0.05 for PushRecovery

    // COP
    double rear_flag_{1.};
    double distance_feet_{0.2};
    double x_cop_lb_{0.};
    double x_cop_ub_{distance_feet_};

    // Knee Singularity, mechanical limits (rad)
    double knee_absMin_{DEG2RAD*5};         // 5deg ; DEG2RAD*5
    double knee_absMax_{PI - DEG2RAD*5};    // PI - DEG2RAD*5

    // TODO: for Bounds & Constraints
    Eigen::Vector4d fcLowerBounds;
    Eigen::Vector4d fcUpperBounds;

    // GRF weight transition
    double scale_ss{1.0};       // between (0., 1.], due to the response time of ANYdrive(SEA) is about 40ms for tracking a step signal. so 1.0 is OK
    double weight_fx_st{1.*1e-2}, weight_fz_st{0.01*1e-2};
    double weight_fx_sw{5.*1e-2}, weight_fz_sw{0.5*1e-2};

    // weight
    Eigen::VectorXd weightVec_CentroidalMoment;                     // 3*1
    Eigen::VectorXd weightVec_FloatingBasePose;                     // 3*1
    Eigen::VectorXd weightVec_PointFeetCartPosition;                // 4*1
    Eigen::VectorXd weightVec_PointFeetCartForce;                   // 4*1
    Eigen::VectorXd weightVec_InternalClosedLoop;                   // 4*1
    Eigen::VectorXd weightVec_TorqueChange;                         // 4*1
    Eigen::VectorXd weightVec_ForceChange;                          // 4*1

    // bounds
    Eigen::VectorXd boundsVec_lb;                                // nV*1
    Eigen::VectorXd boundsVec_ub;                                // nV*1

    // ----------------- Auxiliary Data --------------------
    // for wqp_WBC
    std::vector<int> nWSR_w_{100};
    std::vector<double> cpuTime_w_{10.};
    std::vector<int> intData;
    std::vector<double> doubleData;
    // for hqp_WBC
    std::vector<int> nWSR_h_{100,100,100,100};
    std::vector<double> cpuTime_h_{10.,10.,10.,10.};
    std::vector<int> dataOptInt;
    std::vector<double> dataOptDouble;
    std::vector<int> iterOpt;
    std::vector<int> successOpt;
    std::vector<double> costOpt;
    std::vector<double> timeOpt;
    // ----------------- Auxiliary Data --------------------

    // ------------------------ private Functions -----------------------------------
    // opt get
    bool getOptResult(Eigen::VectorXd & qDDot_opt, Eigen::VectorXd & fcPf_opt, Eigen::VectorXd & tauA_opt);
    bool testClosedLoopConsistent(const Eigen::VectorXd & qDDot_opt, Eigen::Vector4d & xRzRxLzL_ddot_ClosedLoop);

    // convert to Real-Robot PVT-PID
    bool calcJointDes(const robotStateMachine & _rsm, const robotState & _rs, const robotDesired & _rd, robotTaskRef & _rtr);         // calculate PVT in "frrl"
    bool calcPVTPID(const robotState & _rs, const robotDesired & _rd, robotTaskRef & _rtr);          // calculate PVT-PID in "rr"
};


#endif //TASK_CONTROL_H
