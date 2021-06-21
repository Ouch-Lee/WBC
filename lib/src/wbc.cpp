/**
 * @file Wbc.cpp
 * @brief Function implementation part of class Wbc
 * @author Jiajun Wang
 * @date 2020-08-27
 * @version alpha
 * @copyright Copyright (c) 2015-2020 UBT-Beijing
 */

#include "wbc.h"

// ======================================== public Functions ====================================================

Wbc::Wbc(int dimVar, RobotDynamics * roDy){
    nV = dimVar;
    robot = roDy;
    tasks.clear();
    constraints.clear();
    priorityTaskNames.clear();
    priorityConstraintNames.clear();
}

bool Wbc::addTask(Task * const taskPtr, int priority, bool mandatory){
    if (taskPtr == nullptr){
        std::cout << "the pointer of Task can not be NULL." << std::endl;
        return false;
    }
    if (tasks.find(taskPtr->name) != tasks.end()){
        if (mandatory){
            std::cout << "Note : the name of Task already exists. We will remove the former and add this new one!" << std::endl;
            removeTask(taskPtr->name);
        }else{
            std::cout << "Note : the name of Task already exists. We will maintain the former and ignore this new one!" << std::endl;
            return true;
        }
    }

    taskPtr->priority = priority;
    tasks.insert({taskPtr->name, taskPtr});
    nO += taskPtr->dim;
    nO_change = true;

    if (priority >= static_cast<int>(priorityTaskNames.size())){
        for(int i = static_cast<int>(priorityTaskNames.size()); i <= priority; ++i){
            std::vector<std::string> temp;
            priorityTaskNames.push_back(temp);
        }
    }
    priorityTaskNames.at(priority).emplace_back(taskPtr->name);

    return true;
}

bool Wbc::addConstraint(Constraint * const constrPtr, int priority, bool mandatory){
    if (constrPtr == nullptr){
        std::cout << "Error : the pointer of Constraint can not be NULL." << std::endl;
        return false;
    }
    if (tasks.find(constrPtr->name) != tasks.end()){
        if (mandatory){
            std::cout << "Note : the name of Constraint already exists. We will remove the former and add this new one!" << std::endl;
            removeConstraint(constrPtr->name);
        }else{
            std::cout << "Note : the name of Constraint already exists. We will maintain the former and ignore this new one!" << std::endl;
            return true;
        }
    }

    constrPtr->priority = priority;
    constraints.insert({constrPtr->name, constrPtr});
    nC += constrPtr->dim;
    nC_change = true;

    if (priority >= static_cast<int>(priorityConstraintNames.size())){
        for(int i = static_cast<int>(priorityConstraintNames.size()); i <= priority; ++i){
            std::vector<std::string> temp;
            priorityConstraintNames.push_back(temp);
        }
    }
    priorityConstraintNames.at(priority).emplace_back(constrPtr->name);

    return true;
}

bool Wbc::removeTask(const std::string &taskName){

    auto iter = tasks.find(taskName);

    if (iter == tasks.end()){
        std::cout << "Warning : the Task name : " << taskName << " NOT FOUND!" << std::endl;
    }else {
        std::cout << std::endl;
        auto name_iter = std::find(priorityTaskNames.at(iter->second->priority).begin(), priorityTaskNames.at(iter->second->priority).end(), taskName);
        if (name_iter != priorityTaskNames.at(iter->second->priority).end()){
            priorityTaskNames.at(iter->second->priority).erase(name_iter);
        }

        nO -= iter->second->dim;
        tasks.erase(taskName);

        nO_change = true;
    }

    return true;
}

bool Wbc::removeConstraint(const std::string &constrName){

    auto iter = constraints.find(constrName);

    if (iter == constraints.end()){
        std::cout << "Warning : the Constraint name : " << constrName << " NOT FOUND!" << std::endl;
    }else {
        auto name_iter = std::find(priorityConstraintNames.at(iter->second->priority).begin(), priorityConstraintNames.at(iter->second->priority).end(), constrName);
        if (name_iter != priorityConstraintNames.at(iter->second->priority).end()){
            priorityConstraintNames.at(iter->second->priority).erase(name_iter);
        }

        nC -= iter->second->dim;
        constraints.erase(constrName);

        nC_change = true;
    }

    return true;
}

bool Wbc::adjustTaskPriority(const std::string &taskName, int priority){
    auto iter = tasks.find(taskName);
    if (iter == tasks.end()){
        std::cout << "Warning : the Task name : " << taskName << " NOT FOUND!" << std::endl;
    }else {
        if(iter->second->priority != priority){
            auto ptrTemp = iter->second;
            removeTask(taskName);
            addTask(ptrTemp, priority);
        }
    }

    return true;
}

bool Wbc::adjustConstraintPriority(const std::string &constrName, int priority){
    auto iter = constraints.find(constrName);

    if (iter == constraints.end()){
        std::cout << "Warning : the Constraint name : " << constrName << " NOT FOUND!" << std::endl;
    }else {
        if(iter->second->priority != priority){
            auto ptrTemp = iter->second;
            removeConstraint(constrName);
            addConstraint(ptrTemp, priority);
        }
    }

    return true;
}

bool Wbc::clearTask(){
    tasks.clear();
    nO = 0;
    nO_change = true;
    priorityTaskNames.clear();
    return true;
}

bool Wbc::clearConstraint(){
    constraints.clear();
    nC = 0;
    nC_change = true;
    priorityConstraintNames.clear();
    return true;
}

bool Wbc::updateTask(const std::string & taskName,
                     const Eigen::VectorXd & ref,
                     const Eigen::VectorXd & w,
                     const std::vector<double> * params){

    auto iter = tasks.find(taskName);

    if (iter == tasks.end()){
        std::cout << "Error : the Task name : " << taskName << " NOT FOUND!" << std::endl;
        return false;
    }else {
        if (params == nullptr){
            if (iter->second->updateRefence(ref) && iter->second->updateWeight(w)){
                iter->second->update(* robot);
            }else{
                return false;
            }
        }else{
            if (iter->second->updateRefence(ref) && iter->second->updateWeight(w) && iter->second->setParameter(*params)){
                iter->second->update(* robot);
            }else{
                return false;
            }
        }
    }
    return true;
}

bool Wbc::updateTask(const std::string & taskName,
                     const Eigen::VectorXd & ref,
                     const std::vector<double> * params){

    auto iter = tasks.find(taskName);

    if (iter == tasks.end()){
        std::cout << "Error : the Task name : " << taskName << " NOT FOUND!" << std::endl;
        return false;
    }else {
        if (params == nullptr){
            if (iter->second->updateRefence(ref)){
                iter->second->update(* robot);
            }else{
                return false;
            }
        }else{
            if (iter->second->updateRefence(ref) && iter->second->setParameter(*params)){
                iter->second->update(* robot);
            }else{
                return false;
            }
        }
    }
    return true;
}

bool Wbc::updateConstraint(const std::string & constrName,
                           const std::vector<double> * params){

    auto iter = constraints.find(constrName);

    if (iter == constraints.end()){
        std::cout << "Error : the Constraint name : " << constrName << " NOT FOUND!" << std::endl;
        return false;
    }else {
        if (params == nullptr){
            iter->second->update(* robot);
        }else{
            if (iter->second->setParameter(*params)){
                iter->second->update(* robot);
            }else{
                return false;
            }
        }
    }
    return true;
}

bool Wbc::getDimension(int &varDim, int &objDim, int &conDim) const{
    varDim = nV;
    objDim = nO;
    conDim = nC;
    return true;
}

bool Wbc::wbcInit(){
    nO_change = false;
    nC_change = false;
    nV_change = false;

    displayWbcInformation();

    return true;
}

bool Wbc::displayWbcInformation() const{

    std::cout << "###################   WBC  --  Information   ##################" << std::endl
              << "-------------------------   Variable   ------------------------" << std::endl
              << "variables dimension : nV = " << nV << std::endl
              << "---------------------------   Task   --------------------------" << std::endl
              << "All tasks dimension : nO = " << nO << std::endl
              << "Level" << "\t" << " |   " << "Item Keys" << std::endl;
    for(int i = 0; i != static_cast<int>(priorityTaskNames.size()); ++i){
        std::cout << i << "\t" << " |   ";
        for(auto item : priorityTaskNames.at(i)){
            std::cout << item << "(" << tasks.find(item)->second->dim << ")" << " | ";
        }
        std::cout << std::endl;
    }
    std::cout << "-------------------------   Constraint   ----------------------" << std::endl
              << "All constraints dimension : nC = " << nC << std::endl
              << "Level" << "\t" << " |   " << "Item Keys" << std::endl;
    for(int i = 0; i != static_cast<int>(priorityConstraintNames.size()); ++i){
        std::cout << i << "\t" << " |   " ;
        for(auto item : priorityConstraintNames.at(i)){
            std::cout << item << "(" << constraints.find(item)->second->dim << ")" << " | ";
        }
        std::cout << std::endl;
    }
    std::cout << "###############################################################" << std::endl;

    return true;
}

bool Wbc::displayResultInformation() const{
    std::cout << "Nothing to be displayed, you should rewrite this function, if you want to use it."<< std::endl;
    return true;
}

bool Wbc::setParametersInt(const std::vector<int> &Parameters){
    std::cout << "Nothing to be done, you should rewrite this function, if you want to use it."<< std::endl;
    return true;
}

bool Wbc::setParametersDouble(const std::vector<double> &Parameters){
    std::cout << "Nothing to be done, you should rewrite this function, if you want to use it."<< std::endl;
    return true;
}

bool Wbc::getAuxiliaryDataInt(std::vector<int> &auxiliaryData){
    auxiliaryData.clear();
    std::cout << "Nothing to be output, you should rewrite this function, if you want to use it."<< std::endl;
    return true;
}

bool Wbc::getAuxiliaryDataDouble(std::vector<double> &auxiliaryData){
    auxiliaryData.clear();
    std::cout << "Nothing to be output, you should rewrite this function, if you want to use it."<< std::endl;
    return true;
}

// ======================================== protected Functions ====================================================

bool Wbc::check(const Eigen::MatrixXd & M, int row, int col){
    if (M.rows() != row || M.cols() != col){
        std::cout << "Error : Matrix dimensions do not match!" << std::endl;
        return false;
    }
    return true;
}

bool Wbc::check(const Eigen::VectorXd & v, int row){
    if (v.rows() != row){
        std::cout << "Error : Vector dimensions do not match!" << std::endl;
        return false;
    }
    return true;
}
