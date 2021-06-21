/**
 * @file constraintDefinition_Mario2D.h
 * @brief Declaration of the subclasses of the abstract class Constraint
 * @author Jiajun Wang
 * @date 2020-09-01
 * @version alpha
 * @copyright Copyright (c) 2015-2020 UBT-Beijing
 */

#ifndef WBC_CONSTRAINTDEFINITION_MARIO2D_H
#define WBC_CONSTRAINTDEFINITION_MARIO2D_H

#include "constraint.h"


class Mario2dDynamicConsistency : public Constraint{
public:
    /**
     * @brief Constructor
     * @param constrName The unique identification of the Constraint: name
     * @param constrDim The dimension of Constraint
     * @param varDim The DoF of variables in the WBC problem
     */
    Mario2dDynamicConsistency(const std::string & constrName, int constrDim, int varDim) : Constraint(constrName, constrDim, varDim){}
    ~Mario2dDynamicConsistency() = default;
    bool update(const RobotDynamics &robot) override;
};

class Mario2dFrictionCone : public Constraint{
public:
    /**
     * @brief Constructor
     * @param constrName The unique identification of the Constraint: name
     * @param constrDim The dimension of Constraint
     * @param varDim The DoF of variables in the WBC problem
     */
    Mario2dFrictionCone(const std::string & constrName, int constrDim, int varDim) : Constraint(constrName, constrDim, varDim){}
    ~Mario2dFrictionCone() = default;
    /**
     * @brief setParameter
     * @param params
     *  a std::vector, whose first item is the value of mu_static_ and the second item is the value of myInfinity_
     * @return
     */
    bool setParameter(const std::vector<double> & params) override;
    bool update(const RobotDynamics &robot) override;
private:
    double mu_static_{0.9};
    double myInfinity_{1e6};
};

class Mario2dJointTorqueSaturation : public Constraint{
public:
    /**
     * @brief Constructor
     * @param constrName The unique identification of the Constraint: name
     * @param constrDim The dimension of Constraint
     * @param varDim The DoF of variables in the WBC problem
     */
    Mario2dJointTorqueSaturation(const std::string & constrName, int constrDim, int varDim) : Constraint(constrName, constrDim, varDim){}
    ~Mario2dJointTorqueSaturation() = default;
    /**
     * @brief setParameter
     * @param params
     *  a std::vector, whose first item is the value of jointTau_limit_
     * @return
     */
    bool setParameter(const std::vector<double> & params) override;
    bool update(const RobotDynamics &robot) override;
private:
    double jointTau_limit_{35.};
};

class Mario2dKneeSingularity : public Constraint{
public:
    /**
     * @brief Constructor
     * @param constrName The unique identification of the Constraint: name
     * @param constrDim The dimension of Constraint
     * @param varDim The DoF of variables in the WBC problem
     */
    Mario2dKneeSingularity(const std::string & constrName, int constrDim, int varDim) : Constraint(constrName, constrDim, varDim){}
    ~Mario2dKneeSingularity() = default;
    /**
     * @brief setParameter
     * @param params
     *      a std::vector, whose first item is the value of delta_t_,
     *      the second item is the value of knee_absMin_, the third item is the value of knee_absMax_
     * @return
     */
    bool setParameter(const std::vector<double> & params) override;
    bool update(const RobotDynamics &robot) override;
private:
    double delta_t_{0.01};
    double knee_absMin_{0.0174532925 * 5};                  // 5deg
    double knee_absMax_{3.141592654 - 0.0174532925 * 5};    // 180deg - 5deg
};

class Mario2dCenterOfPressure : public Constraint{
public:
    /**
     * @brief Constructor
     * @param constrName The unique identification of the Constraint: name
     * @param constrDim The dimension of Constraint
     * @param varDim The DoF of variables in the WBC problem
     */
    Mario2dCenterOfPressure(const std::string & constrName, int constrDim, int varDim) : Constraint(constrName, constrDim, varDim){}
    ~Mario2dCenterOfPressure() = default;
    /**
     * @brief setParameter
     * @param params
     *  a std::vector, whose items are the value of rear_flag_, distance_feet_, x_cop_lb_, x_cop_ub_
     * @return
     */
    bool setParameter(const std::vector<double> & params) override;
    bool update(const RobotDynamics &robot) override;
private:
    double rear_flag_{1.};
    double distance_feet_{0.2};
    double x_cop_lb_{0.};
    double x_cop_ub_{distance_feet_};
    double myInfinity_{1e6};
};

#endif // WBC_CONSTRAINTDEFINITION_MARIO2D_H
