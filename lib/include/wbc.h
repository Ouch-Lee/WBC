/**
 * @file Wbc.h
 * @brief Declaration of the Wbc class
 * @author Jiajun Wang
 * @date 2020-08-27
 * @version alpha
 * @copyright Copyright (c) 2015-2020 UBT-Beijing
 */

#ifndef WBC_WBC_H
#define WBC_WBC_H

#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <algorithm>

#include <Eigen/Dense>

#include "task.h"
#include "constraint.h"
#include "robotDynamics.h"


/**
 * @class Wbc
 * @brief The Wbc class
 * @note this is an abstract class
 */
class Wbc{
public:

    /**
     * @brief Wbc Constructor
     * @param dimVar Dimension of Variables in the WBC problem
     * @param roDy Control object of WBC problem
     */
    Wbc(int dimVar, RobotDynamics * roDy);

    /**
     * @brief Copy constructor of Wbc
     * @param wbc_foo the wbc instance to be copied from
     */
    Wbc(const Wbc &wbc_foo);

    // ================================================== virtual funcions ====================================================

    virtual ~Wbc() = default;

    /**
     * @brief Implement some initial work
     * @details Generally, this function is called only once, immediately after its instantiation.
     * @return
     */
    virtual bool wbcInit();

    /**
     * @brief Add new element to tasks
     * @param taskPtr Pointer to Task class
     * @param priority The level of priority
     * @param mandatory Whether to replace, if the same key already exists. false by default
     * @return
     */
    virtual bool addTask(Task * const taskPtr, int priority, bool mandatory = false);

    /**
     * @brief Add new element to constraints
     * @param constrPtr Pointer to Constraint class
     * @param priority The level of priority
     * @param mandatory Whether to replace, if the same key already exists. false by default
     * @return
     */
    virtual bool addConstraint(Constraint * const constrPtr, int priority, bool mandatory = false);

    /**
     * @brief Remove an element from tasks
     * @param taskName The key of element to be removed
     * @return
     */
    virtual bool removeTask(const std::string & taskName);

    /**
     * @brief Remove an element from constraints
     * @param constrName The key of element to be removed
     * @return
     */
    virtual bool removeConstraint(const std::string & constrName);

    /**
     * @brief Adjust Task Priority
     * @param taskName The key of element to be adjusted
     * @param priority New level value
     * @return
     */
    virtual bool adjustTaskPriority(const std::string & taskName, int priority);

    /**
     * @brief Adjust Constraint Priority
     * @param constrName The key of element to be adjusted
     * @param priority New level value
     * @return
     */
    virtual bool adjustConstraintPriority(const std::string & constrName, int priority);

    /**
     * @brief Remove all elements from tasks
     * @return
     */
    virtual bool clearTask();

    /**
     * @brief Remove all elements from constraints
     * @return
     */
    virtual bool clearConstraint();

    /**
     * @brief Update the memeber value in the named Task
     * @param taskName The key of element
     * @param ref Reference Vector
     * @param w Weight Vector
     * @param params Parameters if necessary
     * @return
     */
    virtual bool updateTask(const std::string & taskName,
                            const Eigen::VectorXd & ref,
                            const Eigen::VectorXd & w,
                            const std::vector<double> * params = nullptr);

    /**
     * @brief Update the memeber value in the named Task
     * @param taskName The key of element
     * @param ref Reference Vector
     * @param params Parameters if necessary
     * @return
     */
    virtual bool updateTask(const std::string & taskName,
                            const Eigen::VectorXd & ref,
                            const std::vector<double> * params = nullptr);

    /**
     * @brief Update the memeber value in the named Constraint
     * @param constrName The key of element
     * @param params Parameters if necessary
     * @return
     */
    virtual bool updateConstraint(const std::string & constrName,
                                  const std::vector<double> * params = nullptr);

    /**
     * @brief get Dimension of some members
     * @param varDim dimension of Variables
     * @param objDim dimension of Tasks/Objects/Costs
     * @param conDim dimension of Constraints
     * @return
     */
    virtual bool getDimension(int & varDim, int & objDim, int & conDim) const;

    /**
     * @brief get the Pointer of RobotDynamics, i.e. the control object of WBC problem
     * @param robot_wbc the variable to be assigned
     * @return
     */
    virtual bool getRobotPointer(RobotDynamics * robot_wbc) const;

    /**
     * @brief get the main containers of class WBC
     * @param tasks_wbc dictionary of tasks
     * @param constraints_wbc dictionary of constraints
     * @param priorityTaskNames_wbc Two-dimensional table of task priority
     * @param priorityConstraintNames_wbc Two-dimensional table of constraints priority
     * @return
     */
    virtual bool getContainers(std::unordered_map<std::string, Task *> & tasks_wbc,
                               std::unordered_map<std::string, Constraint *> & constraints_wbc,
                               std::vector<std::vector<std::string>> & priorityTaskNames_wbc,
                               std::vector<std::vector<std::string>> & priorityConstraintNames_wbc) const;

    /**
     * @brief copy from another wbc
     * @param wbc_foo the wbc instance to be copied from
     * @return
     */
    virtual bool copyFromWbc(const Wbc & wbc_foo);

    /**
     * @brief Display WBC Information
     * @return
     */
    virtual bool displayWbcInformation() const;

    /**
     * @brief display Result Information
     * @return
     */
    virtual bool displayResultInformation() const;

    /**
     * @brief Set the parameters of the integer type
     * @param Parameters
     * @return
     */
    virtual bool setParametersInt(const std::vector<int> & Parameters);

    /**
     * @brief Set the parameters of the double type
     * @param Parameters
     * @return
     */
    virtual bool setParametersDouble(const std::vector<double> & Parameters);

    /**
     * @brief Get Auxiliary Data of the integer type
     * @param auxiliaryData
     * @return
     */
    virtual bool getAuxiliaryDataInt(std::vector<int> & auxiliaryData);

    /**
     * @brief Get Auxiliary Data of the double type
     * @param auxiliaryData
     * @return
     */
    virtual bool getAuxiliaryDataDouble(std::vector<double> & auxiliaryData);


    // ================================================== pure virtual funcions =================================================

    /**
     * @brief Update the bound of optimization variables
     * @param lb    Lower bound of optimization variables
     * @param ub    Upper bound of optimization variables
     * @return
     */
    virtual bool updateBound(const Eigen::VectorXd & lb, const Eigen::VectorXd & ub) = 0;

    /**
     * @brief update RobotDynamics
     * @details Update the robot member, perform some necessary dynamic calculations
     *          to prepare for the update of tasks and constraints.
     * @param q     Generalized position
     * @param qDot  Generalized velocity
     * @return Whether the function is executed successfully
     *  @retval true     execution succeed
     *  @retval false    execution failed
     * @note You must call this function before update Task and Constraint
     */
    virtual bool updateRobotDynamics(const Eigen::VectorXd & q, const Eigen::VectorXd & qDot) = 0;

    /**
     * @brief Solve the WBC problem using any kind of algorithm
     * @return
     */
    virtual bool wbcSolve() = 0;

    /**
     * @brief Get optimize variable results
     * @param resultOpt Output
     * @return
     */
    virtual bool getResultOpt(Eigen::VectorXd & resultOpt) = 0;


protected:

    RobotDynamics * robot = nullptr;                                ///< control object of WBC

    std::unordered_map<std::string, Task *> tasks;                  ///< dictionary of tasks
    std::unordered_map<std::string, Constraint *> constraints;      ///< dictionary of constraints

    std::vector<std::vector<std::string>> priorityTaskNames;        ///< Two-dimensional table of task priority
    std::vector<std::vector<std::string>> priorityConstraintNames;  ///< Two-dimensional table of constraints priority

    int nV{0};              ///< dimension of Variables
    int nO{0};              ///< dimension of Tasks/Objects/Costs
    int nC{0};              ///< dimension of Constraints

    bool nO_change{false};
    bool nV_change{false};
    bool nC_change{false};

    bool check(const Eigen::MatrixXd & M, int row, int col);
    bool check(const Eigen::VectorXd & v, int row);

};

#endif // WBC_WBC_H
