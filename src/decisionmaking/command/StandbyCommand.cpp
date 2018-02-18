/**
 * \file StandbyCommand.cpp
 * \brief StandbyCommand class implementation.
 * \author christophebedard
 */

#include "StandbyCommand.h"

const std::string StandbyCommand::COMMMAND_NAME = "Standby";

StandbyCommand::StandbyCommand()
    : Command()
{
    
}

StandbyCommand::~StandbyCommand() {
}

void StandbyCommand::publish() {
    //sendDmCmd(, elikos_msgs::DMCmd::STANDBY);
}

std::string StandbyCommand::getName() {
    return COMMMAND_NAME;
}