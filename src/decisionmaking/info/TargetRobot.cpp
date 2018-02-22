/**
 * \file TargetRobot.cpp
 * \brief TargetRobot class implementation.
 * \author christophebedard
 */

#include "TargetRobot.h"

TargetRobot::TargetRobot(geometry_msgs::Point pos, int incertitudeCountMax)
    : pos_(pos),
      incertitudeCountMax_(incertitudeCountMax),
      orientation_(0.0), // orientation is only estimated after the first call to updatePosition()
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

int TargetRobot::getIncertitudeCount() const {
    return incertitudeCount_;
}

geometry_msgs::Pose TargetRobot::getPose() const {
    geometry_msgs::Pose pose;
    pose.position = pos_;
    pose.orientation = tf::createQuaternionMsgFromYaw(orientation_);
    return pose;
}

void TargetRobot::updatePosition(const geometry_msgs::Point& newPos) {
    updateOrientation(newPos);
    pos_ = newPos;
    // reset incertitude counter (since its position was just updated)
    incertitudeCount_ = 0;
}

void TargetRobot::incrementIncertitudeCounter() {
    incertitudeCount_ = (incertitudeCount_ < incertitudeCountMax_) ? incertitudeCount_+1 : incertitudeCountMax_;
}

void TargetRobot::updateOrientation(const geometry_msgs::Point& newPos) {
    /// \todo use better estimation method?
    orientation_ = atan2(newPos.y - pos_.y, newPos.x - pos_.x);
}