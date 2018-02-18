/**
 * \file DmMessageHandler.cpp
 * \brief DmMessageHandler class implementation.
 * \author christophebedard
 */

#include "DmMessageHandler.h"

DmMessageHandler* DmMessageHandler::instance_ = nullptr;

DmMessageHandler DmMessageHandler::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new DmMessageHandler();
    }
    return instance_;
}

void DmMessageHandler::freeInstance() {
    delete instance_;
    instance_ = nullptr;
}

DmMessageHandler::DmMessageHandler() {
    // get params
    // \todo limit scope of topic names
    ros::NodeHandle n_p("~");
    n_p.getParam("target_array_topic", targetArrayTopic_);
    n_p.getParam("cmd_topic", cmdTopic_);
    n_p.getParam("origin_tf_name", originTfName_);
    n_p.getParam("quad_tf_name", quadTfName_);
    n_p.getParam("simulation", isSimulation_);
    n_p.getParam("dm_state_debug_topic", stateDebugTopic_);

    // setup subscribers
    targetRobotArraySub_ = nh_.subscribe<elikos_msgs::TargetRobotArray>(targetArrayTopic_, 1, &DmMessageHandler::targetRobotArrayCallback, this);
    
    // setup publishers
    dmCmdPub_ = nh_.advertise<elikos_msgs::DMCmd>(cmdTopic_, 1);
    dmCurrentStatePub_ = nh_.advertise<std_msgs::String>(stateDebugTopic_, 1)
}

DmMessageHandler::~DmMessageHandler() {
    // \todo add if needed
}

void DmMessageHandler::update() {
    updateQuadTf();
}

void DmMessageHandler::updateQuadTf() {
    // lookup transform
    tf::Transform transform;
    try {
        tfListener_.lookupTransform(originTfName_, quadTfName_, ros::Time(0), transform);
    } catch(tf::TransformException e) {
        ROS_ERROR("[DM MESSAGE HANDLER] error looking up quad tf");
        return;
    }

    // create pose msg
    geometry_msgs::Transform geoTfMsg;
    tf::transformTFToMsg(transform, geoTfMsg);

    geometry_msgs::Pose poseMsg;
    poseMsg.position.x = geoTfMsg.translation.x;
    poseMsg.position.y = geoTfMsg.translation.y;
    poseMsg.position.z = geoTfMsg.translation.z;
    poseMsg.orientation = geoTfMsg.rotation;

    InformationManager::getInstance()->updateQuad(poseMsg);
}

void DmMessageHandler::targetRobotArrayCallback(const elikos_msgs::TargetRobotArray::ConstPtr& msg) {
    // \todo send target robot array to InformationManager
    //InformationManager::getInstance()->updateTargets(msg);
}

void DmMessageHandler::publishDmCmd(const geometry_msgs::Pose& destPose, int cmdCode) {
    geometry_msgs::PoseStamped poseStampedMsg;
    poseStampedMsg.pose = destPose;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = originTfName_;

    elikos_msgs::DMCmd msg;
    msg.cmdCode = cmdCode;
    msg.pose = poseStampedMsg;

    dmCmdPub_.publish(msg);
}

void DmMessageHandler::publishCurrentDmState(const std::string& state) {
    std_msgs::String msg;
    msg.data = state;
    dmCurrentStatePub_.publish(msg);
}