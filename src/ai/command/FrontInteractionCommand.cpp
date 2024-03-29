//
// Created by olivier on 08/07/16.
//

#include "MessageHandler.h"
#include "TargetRobot.h"
#include "QuadRobot.h"

#include "FrontInteractionCommand.h"

namespace ai
{

const double FrontInteractionCommand::FORWARD_OFFSET = 1.5;
const double FrontInteractionCommand::WAIT_TIME = 5.0;

FrontInteractionCommand::FrontInteractionCommand(QuadRobot* quad, TargetRobot* target)
    : AbstractCommand(quad, target)
{
    destination_ = target_->getPose().getOrigin() + (target_->getDirection() * FORWARD_OFFSET);
    destination_.setZ(0.0);
    timer_.start();
}

FrontInteractionCommand::~FrontInteractionCommand()
{
}

void FrontInteractionCommand::execute()
{
    ai::MessageHandler::getInstance()->publishAiStateCommand("Front interaction command");
    MessageHandler::getInstance()->sendDestination(destination_, elikos_msgs::DMCmd::FRONT_INTERACTION);
}

bool FrontInteractionCommand::isCommmandDone()
{
    return (timer_.getElapsedS() > WAIT_TIME);
}

}
