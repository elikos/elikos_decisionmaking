#ifndef ELIKOS_DUNGEONMASTER_DMMESSAGEHANDLER_H
#define ELIKOS_DUNGEONMASTER_DMMESSAGEHANDLER_H

/**
 * \file DmMessageHandler.h
 * \brief DmMessageHandler class declaration
 * \author Olivier
 * \author christophebedard
 */

#include <string>
#include <ros/ros.h> // \todo remove?
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <std_msgs/String.h>
#include "elikos_msgs/TargetRobotArray.h"
#include "elikos_msgs/DMCmd.h"
#include "InformationManager.h"

/**
 * \class DmMessageHandler
 * \brief singleton class which contains and deals with all DM publishers and subscribers.
 */
class DmMessageHandler
{
public:
    /**
     * \brief Instance accessor.
     * 
     * \return the instance.
     */
    static DmMessageHandler* getInstance();

    /**
     * \brief Instance deletion.
     */
    static void freeInstance();

    /**
     * \brief DM command publisher method.
     * Stamps the pose and creates a DMCmd msg to publish.
     * 
     * \param destPose : destination/target pose.
     * \param cmdCode : command code (according to elikos_msgs::DMCmd).
     */
    void publishDmCmd(const geometry_msgs::Pose& destPose, int cmdCode);

    /**
     * \brief Publishes the current state string.
     * 
     * \param state : the current state formatted as a string.
     */
    void publishCurrentDmState(const std::string& state);

    /**
     * \brief Deleted copy constructor.
     */
    DmMessageHandler(const DmMessageHandler&) = delete;

    /**
     * \brief Deleted copy assignment.
     */
    void operator=(const DmMessageHandler&)  = delete;

private:
    static DmMessageHandler* instance_; /**< the instance itself */

    ros::NodeHandle nh_; /**< the node hanlde \todo get pointer from DecisionMaking.cpp? */

    bool isSimulation_; /**< the simulation state \todo needed? */

    std::string targetArrayTopic_; /**< the topic for target robot array */
    std::string cmdTopic_; /**< the command topic */
    std::string originTfName_; /**< the tf name for the arena origin */
    std::string quadTfName_; /**< the tf name for the quad's position */
    std::string stateDebugTopic_; /**< the name for the current state debug topic */

    ros::Subscriber targetRobotArraySub_; /**< the target robot array subscriber (INPUT from detection) */
    ros::Publisher dmCmdPub_; /**< the command publisher (OUTPUT to path planning (!sim) or elikos_sim (sim)) */
    ros::Publisher dmCurrentStatePub_; /**< the current state publisher (debug OUTPUT) */

    tf::TransformListener tfListener_; /**< the tf listener */

    /**
     * \brief Callback method for target robot array INPUT.
     * 
     * \param msg : the input message.
     */
    void targetRobotArrayCallback(const elikos_msgs::TargetRobotArray::ConstPtr& msg);

    /**
     * \brief Updates the current quad tf.
     */
    void updateQuadTf();

    /**
     * \brief Private constructor.
     */
    DmMessageHandler();

    /**
     * \brief Private destructor.
     */
    ~DmMessageHandler();

};

#endif // ELIKOS_DUNGEONMASTER_DMMESSAGEHANDLER_H