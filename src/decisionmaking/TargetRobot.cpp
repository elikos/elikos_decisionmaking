/**
 * \file TargetRobot.cpp
 * \brief TargetRobot class implementation.
 * \author christophebedard
 */

#include "TargetRobot.h"

TargetRobot::TargetRobot(geometry_msgs::Point pos)
    : pos_(pos),
      orientation_(0.0)
      incertitudeCount_(0)
{
}

TargetRobot::~TargetRobot() {
}

geometry_msgs::Point TargetRobot::getPosition() const {
    return pos_;
}

double TargetRobot::getOrientation() const {
    return orientation_;
}

void TargetRobot::updatePosition(geometry_msgs::Point pos) {
    // \todo calculate new orientation
    //orientation_ = 
    pos_ = pos;
}

void TargetRobot::incrementIncertitudeCounter() {
    incertitudeCount_++;
}