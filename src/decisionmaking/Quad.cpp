/**
 * \file Quad.cpp
 * \brief Quad class implementation.
 * \author christophebedard
 */

#include "Quad.h"

Quad::Quad(geometry_msgs::Pose pose)
    : pose_(pose)
{
    // \todo might need to initialize pose_ at 0
}

Quad::~Quad() {
}

geometry_msgs::Pose Quad::getPose() const {
    return pose_;
}

void Quad::updatePose(geometry_msgs::Pose pose) {
    pose_ = pose;
}