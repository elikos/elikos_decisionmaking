/**
 * \file InformationManager.cpp
 * \brief InformationManager class implementation.
 * \author christophebedard
 */

#include "InformationManager.h"

InformationManager* InformationManager::instance_ = nullptr;

InformationManager* InformationManager::getInstance() {
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
    // delete targets
    for (auto it = targets_.begin(); it != targets_.end(); it++) {
        delete (*it);
    }
    // delete quad
    delete quad_;
}

void InformationManager::updateQuad(const geometry_msgs::Pose& pose) {
    quad_->updatePose(pose);
}

geometry_msgs::Pose InformationManager::getQuadPose() const {
    return quad_->getPose();
}

void InformationManager::updateTargets(const elikos_msgs::TargetRobotArray::ConstPtr& msg) {
    // \todo refactor/separate into multiple methods

    // convert TargetRobotArray to vector<Point>
    std::vector<geometry_msgs::Point> newTargetPoints;
    for (auto it = msg->targets.begin(); it != msg->targets.end(); it++) {
        newTargetPoints.push_back(it->poseOrigin.pose.position);
    }

    // compute distance between each current target and each new target
    // i : current target
    // j : new target
    // (i,j) : distance between current target and new target
    int numCurrentTargets = targets_.size();
    int numNewTargets = newTargetPoints.size();
    // \todo do somethign about this?
    std::vector<std::vector<double>> distances(numCurrentTargets, std::vector<double>(numNewTargets));
    for (int i = 0; i < numCurrentTargets; ++i) {
        for (int j = 0; j < numNewTargets; ++j) {
            distances[i][j] = distanceSquared(targets_.at(i)->getPosition(),
                                              newTargetPoints.at(j));
        }
    }

    // assign all current targets to new targets
    int assignedCurrentTargets = 0;
    while (assignedCurrentTargets < numCurrentTargets) {
        // find the (i,j) position of the minimum distance
        int iMinDist = 0;
        int jMinDist = 0;
        int minDist = distances[iMinDist][jMinDist];
        for (int i = 1; i < numCurrentTargets; ++i) {
            for (int j = 1; j < numNewTargets; ++j) {
                // if not already assigned and distance less than current minimum
                int curDist = distances[i][j];
                if ((distances[i][j] != -1.0) && (curDist < minDist)) {
                    iMinDist = i;
                    jMinDist = j;
                    minDist = curDist;
                }
            }
        }

        // set row and col corresponding to iMinDist and jMinDist to -1.0 to indicate that they've been assigned
        for (int i = 0; i < numCurrentTargets; ++i) {
            distances[i][jMinDist] = -1.0;
        }
        for (int j = 0; j < numNewTargets; ++j) {
            distances[iMinDist][j] = -1.0;
        }

        // assign (set current target's position to new target's position)
        targets_.at(iMinDist)->updatePosition(newTargetPoints.at(jMinDist));
        assignedCurrentTargets++;
    }

    // create new targets for unassigned new targets
    // find unassigned new target using the first row
    for (int j = 0; j < numNewTargets; ++j) {
        if (distances[0][j] != -1.0) {
            targets_.push_back(new TargetRobot(newTargetPoints.at(j)));
            distances[0][j] = -1.0; // unnecessary, but perhaps it's future-proof
        }
    }

    DmMessageHandler::getInstance()->publishTargetPoses(getTargetPoses());
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

double InformationManager::distanceSquaredQuadTarget(TargetRobot* target) const {
    return distanceSquared(quad_->getPose(), target->getPosition());
}

double InformationManager::distanceSquared(const geometry_msgs::Pose& pose, const geometry_msgs::Point& point) const {
    return distanceSquared(pose.position, point);
}

double InformationManager::distanceSquared(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) const {
    return std::pow(p2.x - p1.x, 2.0) + std::pow(p2.y - p1.y, 2.0);
}

std::vector<geometry_msgs::Pose>& InformationManager::getTargetPoses() const {
    std::vector<geometry_msgs::Pose> vec;

    for (auto it = targets_.begin(); it != targets_.end(); it++) {
        vec.push_back((*it)->getPose());
    }

    return vec;
}