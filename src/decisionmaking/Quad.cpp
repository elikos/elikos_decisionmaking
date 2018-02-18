/**
 * \file Quad.cpp
 * \brief Quad class implementation.
 * \author christophebedard
 */

#include "Quad.h"

Quad::Quad(geometry_msgs::Pose pose)
    : pose_(pose)
{
}

Quad::Quad() {
    // initialize pose at 0
    pose_.position.x = 0.0;
    pose_.position.y = 0.0;
    pose_.position.z = 0.0;
    pose_.orientation.x = 0.0;
    pose_.orientation.y = 0.0;
    pose_.orientation.z = 0.0;
    pose_.orientation.w = 1.0;
}

geometry_msgs::Pose Quad::getPose() const {
    return pose_;
}

void Quad::updatePose(const geometry_msgs::Pose& pose) {
    pose_ = pose;
}