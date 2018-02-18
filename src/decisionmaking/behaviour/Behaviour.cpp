/**
 * \file Behaviour.cpp
 * \brief asbtract behaviour class implementation
 * \author christophebedard
 * \author Olivier
 */

#include "Behaviour.h"

Behaviour::Behaviour()
    : hasReachedDestinationThreshold_(0.2)
{
    // get param
    ros::NodeHandle n_p("~");
    n_p.getParam("has_reached_destination_threshold", hasReachedDestinationThreshold_)
}

Behaviour::~Behaviour() {
}

Command* Behaviour::getCurrentCommand() {
    return currentCmd_;
}

bool Behaviour::hasReachedDestination(const tf::Vector3& currentPos, const tf::Vector3& dest) {
    double distance;

    if (dest.getZ() < -0.5) {
        tf::Vector3 groundPos = destination;
        groundPos.setZ(0.0);
        distance = tf::tfDistance(currentPos, groundPos);
    } else {
        distance = tf::tfDistance(currentPos, dest);
    }

    return std::abs(distance) < hasReachedDestinationThreshold_;
}