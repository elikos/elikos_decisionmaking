/**
 * \file Command.cpp
 * \brief Command class implementation.
 * \author Olivier
 * \author christophebedard
 */

#include "Command.h"
#include "DmMessageHandler.h"

Command::Command()
    : hasReachedDestinationThreshold_(0.2)
{
    // get param
    ros::NodeHandle n_p("~");
    n_p.getParam("has_reached_destination_threshold", hasReachedDestinationThreshold_)
}

Command::~Command() {
}

bool Command::hasReachedDestination(const tf::Vector3& currentPos, const tf::Vector3& dest) {
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

void Command::sendDmCmd(const geometry_msgs::Pose& destPose, int cmdCode) {
    MessageHandler::getInstance()->publishDmCmd(destPose, cmdCode);
}