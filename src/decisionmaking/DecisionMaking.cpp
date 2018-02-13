/**
 * \file DecisionMaking.cpp
 * \brief entry point for elikos_decisionmaking2018 node
 * \author christophebedard
 */

#include <string>
#include "Strategy.h"


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

    /*
    ros::Rate rate(1);
    while (ros::ok())
    {
        

        ros::spinOnce();
        rate.sleep();
    }
    */
}
