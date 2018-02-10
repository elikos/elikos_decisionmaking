#include "ros/ros.h"
#include <string>

int main(int argc, char* argv[])
{
    // ROS Init
    ros::init(argc, argv, "elikos_decisionmaking");
    ros::NodeHandle n;

    // get strategy selection parameter
    ros::NodeHandle n_p("~");
    std::string chosenStrategy;
    n_p.getParam("strategy_choice", chosenStrategy);
    ROS_INFO_STREAM("[DM] chosen strategy : " + chosenStrategy);

    /*
    ros::Rate rate(1);
    while (ros::ok())
    {
        

        ros::spinOnce();
        rate.sleep();
    }
    */
}
