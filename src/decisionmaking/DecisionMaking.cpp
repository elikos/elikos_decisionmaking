/**
 * \file DecisionMaking.cpp
 * \brief entry point for elikos_decisionmaking2018 node
 * \author christophebedard
 */

#include <string>
#include "stategy/StrategyFactory.h"
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
    std::unique_ptr<Strategy> strat = StrategyFactory::create(chosenStrategy);

    // init handler/manager
    DmMessageHandler::getInstance();
    InformationManager::getInstance();

    // launch strategy
    //strat->launch();

    ros::Rate rate(30);
    while (ros::ok())
    {
        DmMessageHandler::getInstance()->update();

        // update strategy: behaviour + command
        strat->update();

        DmMessageHandler::getInstance()->publishCurrentDmState(formatCurrentState(strat));

        ros::spinOnce();
        rate.sleep();
    }

    // shutdown, for safety?
    start->land();

    // free
    strat.reset();
    DmMessageHandler::freeInstance();
    InformationManager::freeInstance();
}
