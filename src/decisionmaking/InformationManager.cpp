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
    targets_ = new std::vector<TargetRobot*>();
    quad_ = new Quad();

    // get param
    ros::NodeHandle n_p("~");
    n_p.getParam("dimension_c", arenaDimension_);
    n_p.getParam("target_incertitude_count_threshold", targetIncertitudeCountThreshold_);
    n_p.getParam("target_incertitude_count_max", targetIncertitudeCountMax_);
}

InformationManager::~InformationManager() {
    // delete targets
    for (auto it = targets_->begin(); it != targets_->end(); it++) {
        delete (*it);
    }
    delete targets_;

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
    /// \todo refactor/separate into multiple methods

    // convert TargetRobotArray to vector<Point>
    std::vector<geometry_msgs::Point> newTargetPoints;
    for (auto it = msg->targets.begin(); it != msg->targets.end(); it++) {
        newTargetPoints.push_back(it->poseOrigin.pose.position);
    }

    // compute distance between each current target and each new target
    int numCurrentTargets = targets_->size();
    int numNewTargets = newTargetPoints.size();
    std::vector<bool> isCurrentTargetAssigned(numCurrentTargets);
    std::vector<bool> isNewTargetAssigned(numNewTargets);
    std::vector<std::vector<double>> distances(numCurrentTargets, std::vector<double>(numNewTargets));
    computeDistances(targets_, newTargetPoints, distances);

    // assign current targets to new targets (while there are enough targets to assign)
    int numAssignedTargets = 0;
    while (numAssignedTargets < std::min(numCurrentTargets, numNewTargets)) {
        // find the (i,j) position of the minimum distance that's not already taken
        int iMinDist;
        int jMinDist;
        findMinimumDistanceIndexes(distances, isCurrentTargetAssigned, isNewTargetAssigned, iMinDist, jMinDist);

        // indicate that the two targets have been assigned
        isCurrentTargetAssigned.at(iMinDist) = true;
        isNewTargetAssigned.at(jMinDist) = true;

        // assign (set current target's position to new target's position)
        targets_->at(iMinDist)->updatePosition(newTargetPoints.at(jMinDist));
        numAssignedTargets++;
    }
    
    // check unassigned targets (current or new)
    if (numAssignedTargets < numNewTargets) {
        // create new targets for unassigned new targets
        for (int j = 0; j < numNewTargets; ++j) {
            if (!isNewTargetAssigned.at(j)) {
                targets_->push_back(new TargetRobot(newTargetPoints.at(j), targetIncertitudeCountMax_));
                isNewTargetAssigned.at(j) = true; // unnecessary, but perhaps it's future-proof
            }
        }
    } else if (numAssignedTargets < numCurrentTargets) {
        // increment counter for unassigned targets
        for (int i = 0; i < numCurrentTargets; ++i) {
            if (!isCurrentTargetAssigned.at(i)) {
                targets_->at(i)->incrementIncertitudeCounter();
                isCurrentTargetAssigned.at(i) = true; // unnecessary, but perhaps it's future-proof
            }
        }
    }

    // debug
    DmMessageHandler::getInstance()->publishTargetPoses(getTargetPoses());
    DmMessageHandler::getInstance()->publishTargetMarkerArray(generateMarkerArray());
}

bool InformationManager::hasTarget() const {
    return getNumValidTargets() >= 1;
}

int InformationManager::getNumValidTargets() const {
    return std::count_if(targets_->begin(), targets_->end(), [&](TargetRobot* t)->bool{ return t->getIncertitudeCount() <= this->targetIncertitudeCountThreshold_; });
}

TargetRobot* InformationManager::getClosestTargetToQuad() const {
    if (hasTarget()) {
        auto minIt = std::min_element(targets_->begin(), targets_->end(), [&](TargetRobot* l, TargetRobot* r)->bool{ return this->distanceSquaredQuadTarget(l) < this->distanceSquaredQuadTarget(r); });
        return (*minIt);
    } else {
        return nullptr;
    }
}

TargetRobot* InformationManager::getClosestTargetToGreenLine() const {
    if (hasTarget()) {
        auto minIt = std::min_element(targets_->begin(), targets_->end(), [&](TargetRobot* l, TargetRobot* r)->bool{ return this->distanceTargetToGreenLine(l) < this->distanceTargetToGreenLine(r); });
        return (*minIt);
    } else {
        return nullptr;
    }
}

double InformationManager::distanceTargetToGreenLine(TargetRobot* target) const {
    // assuming the green line is at (y = arenaDimension_ / 2)
    return ((double)arenaDimension_ / 2.0) - target->getPosition().y;
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

std::vector<geometry_msgs::Pose> InformationManager::getTargetPoses() const {
    std::vector<geometry_msgs::Pose> vec;

    for (auto it = targets_->begin(); it != targets_->end(); it++) {
        vec.push_back((*it)->getPose());
    }

    return vec;
}

void InformationManager::computeDistances(std::vector<TargetRobot*>* targets, const std::vector<geometry_msgs::Point>& newTargetPoints, std::vector<std::vector<double>>& distances) {
    // i : current target
    // j : new target
    // (i,j) : squared distance between current target and new target
    for (int i = 0; i < distances.size(); ++i) {
        for (int j = 0; j < distances.at(0).size(); ++j) {
            distances[i][j] = distanceSquared(targets->at(i)->getPosition(),
                                              newTargetPoints.at(j));
        }
    }
}

void InformationManager::findMinimumDistanceIndexes(const std::vector<std::vector<double>>& distances, const std::vector<bool>& isCurrentTargetAssigned, const std::vector<bool>& isNewTargetAssigned, int& iMinDist, int& jMinDist) {
    iMinDist = 0;
    jMinDist = 0;
    int minDist = distances[iMinDist][jMinDist];

    for (int i = 1; i < distances.size(); ++i) {
        // if current target not already assigned
        if (!isCurrentTargetAssigned.at(i)) {
            for (int j = 1; j < distances.at(0).size(); ++j) {
                // if new target not already assigned
                if (!isNewTargetAssigned.at(j)) {
                    // if distance less than current minimum, update indexes and distance
                    int curDist = distances[i][j];
                    if (curDist < minDist) {
                        iMinDist = i;
                        jMinDist = j;
                        minDist = curDist;
                    }
                }
            }
        }
    }
}

visualization_msgs::MarkerArray InformationManager::generateMarkerArray() const {
    visualization_msgs::MarkerArray msgArray;

    for (auto it = targets_->begin(); it != targets_->end(); it++) {
        visualization_msgs::Marker msg;

        msg.header.frame_id = "/elikos_arena_origin";
        msg.id = it - targets_->begin();
        msg.type = visualization_msgs::Marker::MESH_RESOURCE;
        msg.action = visualization_msgs::Marker::ADD;
        msg.mesh_resource = "package://elikos_roomba/models/robot_green.dae";
        msg.pose = (*it)->getPose();
        double icnt = (double) ((*it)->getIncertitudeCount());
        double threshold = (double) targetIncertitudeCountThreshold_;
        msg.color.r = std::min(1.0, icnt/threshold);
        msg.color.g = 1.0;
        msg.color.b = 0.0;
        msg.color.a = 1.0;
        msg.scale.x = 1.1;
        msg.scale.y = 1.1;
        msg.scale.z = 1.1;
        msg.mesh_use_embedded_materials = true;
        msg.lifetime = ros::Duration();

        msgArray.markers.push_back(msg);
    }

    return msgArray;
}
