/**
 * @file taskDefinition_Mario2D.h
 * @brief Declaration of the subclasses of the abstract class Task
 * @author Jiajun Wang
 * @date 2020-09-01
 * @version alpha
 * @copyright Copyright (c) 2015-2020 UBT-Beijing
 */

#ifndef WBC_TASKDEFINITION_MARIO2D_H
#define WBC_TASKDEFINITION_MARIO2D_H

#include "task.h"

class Mario2dCentroidalMoment : public Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    Mario2dCentroidalMoment(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~Mario2dCentroidalMoment() = default;
    bool update(const RobotDynamics &robot) override;
};

class Mario2dFloatingBasePose : public Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    Mario2dFloatingBasePose(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~Mario2dFloatingBasePose() = default;
    bool update(const RobotDynamics &robot) override;
};

class Mario2dPointFeetCartPosition : public Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    Mario2dPointFeetCartPosition(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~Mario2dPointFeetCartPosition() = default;
    /**
     * @brief setParameter
     * @param params
     *  a std::vector, whose first item is the value of JcTruncation_
     * @return
     */
    bool setParameter(const std::vector<double> & params) override;
    bool update(const RobotDynamics &robot) override;
private:
    bool JcTruncation_{false};
};

class Mario2dPointFeetCartForce : public Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    Mario2dPointFeetCartForce(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~Mario2dPointFeetCartForce() = default;
    bool update(const RobotDynamics &robot) override;
};

class Mario2dInternalClosedLoop : public Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    Mario2dInternalClosedLoop(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~Mario2dInternalClosedLoop() = default;
    bool update(const RobotDynamics &robot) override;
};

class Mario2dJointTorqueChange : public Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    Mario2dJointTorqueChange(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~Mario2dJointTorqueChange() = default;
    bool update(const RobotDynamics &robot) override;
};

class Mario2dFeetForceChange : public Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    Mario2dFeetForceChange(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~Mario2dFeetForceChange() = default;
    bool update(const RobotDynamics &robot) override;
};

#endif // WBC_TASKDEFINITION_MARIO2D_H
