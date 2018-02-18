/**
 * \file TargetRobot.cpp
 * \brief TargetRobot class implementation.
 * \author christophebedard
 */

#include "TargetRobot.h"

TargetRobot::TargetRobot(tf::Vector3 pos)
    : pos_(pos),
      incertitudeCount_(0)
{
}

TargetRobot::~TargetRobot() {
}

tf::Vector3 TargetRobot::getPosition() const {
    return pos_;
}

void TargetRobot::updatePosition(tf::Vector3 pos) {
    pos_ = pos;
}

void TargetRobot::incrementIncertitudeCounter() {
    incertitudeCount_++;
}