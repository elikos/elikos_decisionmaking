/**
 * \file DecisionMaking.cpp
 * \brief entry point for elikos_decisionmaking2018 node
 * \author christophebedard
 */

#include <string>
//#include "stategy/Strategy.h"
#include "DmMessageHandler.h"
#include "InformationManager.h"

int main(int argc, char* argv[])
{
    // ROS init
    ros::init(argc, argv, "elikos_decisionmaking2018");
    ros::NodeHandle n;

    // get strategy selection parameter
    ros::NodeHandle n_p("~");
    std::string chosenStrategy;
    n_p.getParam("strategy_choice", chosenStrategy);
    ROS_INFO_STREAM("[DM] chosen strategy : " + chosenStrategy);

    // Choose corresponding strategy
    //Strategy* strat = new Whatever();
    //strat->launch();

    // init
    DmMessageHandler::getInstance();
    InformationManager::getInstance();

    ros::Rate rate(30);
    while (ros::ok())
    {
        DmMessageHandler::getInstance()->update();

        // update strategy, behaviour, and command

        DmMessageHandler::getInstance()->publishCurrentDmState("strategy/behaviour/command");

        ros::spinOnce();
        rate.sleep();
    }

    DmMessageHandler::freeInstance();
    InformationManager::freeInstance();
}
