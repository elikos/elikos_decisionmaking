/**
 * \file Command.cpp
 * \brief Command class implementation.
 * \author Olivier
 * \author christophebedard
 */

#include "Command.h"
#include "DmMessageHandler.h"

Command::Command() {
    
}

Command::~Command() {
}

void Command::sendDmCmd(const geometry_msgs::Pose& destPose, int cmdCode) {
    MessageHandler::getInstance()->publishDmCmd(destPose, cmdCode);
}