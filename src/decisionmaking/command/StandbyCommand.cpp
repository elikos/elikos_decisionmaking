/**
 * \file StandbyCommand.cpp
 * \brief StandbyCommand class implementation.
 * \author christophebedard
 */

#include "StandbyCommand.h"

StandbyCommand::StandbyCommand()
    : Command()
{
    
}

StandbyCommand::~StandbyCommand() {
}

StandbyCommand::execute() {
    //sendDmCmd(, elikos_msgs::DMCmd::STANDBY);
}

StandbyCommand::isDone() {

}