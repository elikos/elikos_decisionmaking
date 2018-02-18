/**
 * \file InformationManager.cpp
 * \brief InformationManager class implementation.
 * \author christophebedard
 */

#include "InformationManager.h"

InformationManager* InformationManager::instance_ = nullptr;

InformationManager InformationManager::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new InformationManager();
    }
    return instance_;
}

void InformationManager::freeInstance() {
    delete instance_;
    instance_ = nullptr;
}

InformationManager::InformationManager() {
    quad_ = new Quad();

    // get param
    ros::NodeHandle n_p("~");
    n_p.getParam("dimension_c", arenaDimension_);
    n_p.getParam("target_incertitude_count_threshold", targetIncertitudeCountThreshold_);
}

InformationManager::~InformationManager() {
    // \todo add if needed
}

void InformationManager::updateQuad(const geometry_msgs::Pose& pose) {
    quad_->updatePose(pose);
}

geometry_msgs::Pose InformationManager::getQuadPose() const {
    return quad_->getPose();
}

void InformationManager::updateTargets(const elikos_msgs::TargetRobotArray::ConstPtr& msg) {
    // \todo implement

    // compute distance between each current target and each new target
    // assign new target to current target corresponding to min distance
    // repeat until there are no current targets
    // if there are more new targets than current ones, create and add them to the list
    
    // DmMessageHandler::getInstance()->publishTargetPoses();
}

bool InformationManager::hasTarget() const {
    // \todo implement
    // count number of targets with counter value below threshold
    return false;
}

TargetRobot* InformationManager::getClosestTargetToQuad() const {
    // \todo implement
    // get target closest to quad with counter value below threshold
    return nullptr;
}

TargetRobot* InformationManager::getClosestTargetToGreenLine() const {
    // \todo implement
    // get target closest to green with counter value below threshold
    return nullptr;
}