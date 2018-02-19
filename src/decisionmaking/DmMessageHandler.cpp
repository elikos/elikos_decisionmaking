/**
 * \file DmMessageHandler.cpp
 * \brief DmMessageHandler class implementation.
 * \author christophebedard
 */

#include "DmMessageHandler.h"

DmMessageHandler* DmMessageHandler::instance_ = nullptr;

DmMessageHandler* DmMessageHandler::getInstance() {
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
    /// \todo limit scope of topic names
    ros::NodeHandle n_p("~");
    n_p.getParam("target_array_topic", targetArrayTopic_);
    n_p.getParam("cmd_topic", cmdTopic_);
    n_p.getParam("origin_tf_name", originTfName_);
    n_p.getParam("quad_tf_name", quadTfName_);
    n_p.getParam("simulation", isSimulation_);
    n_p.getParam("debug", isDebug_);
    n_p.getParam("dm_state_debug_topic", stateDebugTopic_);
    n_p.getParam("dm_target_poses_topic", targetPosesDebugTopic_);

    // setup subscribers
    targetRobotArraySub_ = nh_.subscribe<elikos_msgs::TargetRobotArray>(targetArrayTopic_, 1, &DmMessageHandler::targetRobotArrayCallback, this);
    
    // setup publishers
    dmCmdPub_ = nh_.advertise<elikos_msgs::DMCmd>(cmdTopic_, 1);
    dmCurrentStateDebugPub_ = nh_.advertise<std_msgs::String>(stateDebugTopic_, 1);
    targetPosesDebugPub_ = nh_.advertise<geometry_msgs::PoseArray>(targetPosesDebugTopic_, 1);

    // wait for tf
    tfListener_.waitForTransform(originTfName_, quadTfName_, ros::Time(0), ros::Duration(5.0));
}

DmMessageHandler::~DmMessageHandler() {
    /// \todo add if needed
}

void DmMessageHandler::update() {
    updateQuadTf();
}

void DmMessageHandler::updateQuadTf() {
    // lookup transform
    tf::StampedTransform stampedTf;
    try {
        tfListener_.lookupTransform(originTfName_, quadTfName_, ros::Time(0), stampedTf);
    } catch(tf::TransformException e) {
        ROS_ERROR("[DM MESSAGE HANDLER] error looking up quad tf");
        return;
    }

    // create pose msg
    geometry_msgs::TransformStamped geoTfStMsg;
    tf::transformStampedTFToMsg(stampedTf, geoTfStMsg);

    geometry_msgs::Pose poseMsg;
    poseMsg.position.x = geoTfStMsg.transform.translation.x;
    poseMsg.position.y = geoTfStMsg.transform.translation.y;
    poseMsg.position.z = geoTfStMsg.transform.translation.z;
    poseMsg.orientation = geoTfStMsg.transform.rotation;

    InformationManager::getInstance()->updateQuad(poseMsg);
}

void DmMessageHandler::targetRobotArrayCallback(const elikos_msgs::TargetRobotArray::ConstPtr& msg) {
    InformationManager::getInstance()->updateTargets(msg);
}

void DmMessageHandler::publishDmCmd(const geometry_msgs::Pose& destPose, int cmdCode) const {
    geometry_msgs::PoseStamped poseStampedMsg;
    poseStampedMsg.pose = destPose;
    poseStampedMsg.header.stamp = ros::Time::now();
    poseStampedMsg.header.frame_id = originTfName_;

    elikos_msgs::DMCmd msg;
    msg.cmdCode = cmdCode;
    msg.pose = poseStampedMsg;

    dmCmdPub_.publish(msg);
}

void DmMessageHandler::publishCurrentDmState(const std::string& state) const {
    /// \todo less ghetto way to do this?
    if (isDebug_) {
        std_msgs::String msg;
        msg.data = state;

        dmCurrentStateDebugPub_.publish(msg);
    }
}

void DmMessageHandler::publishTargetPoses(const std::vector<geometry_msgs::Pose>& poses) const {
    /// \todo less ghetto way to do this?
    if (isDebug_) {
        geometry_msgs::PoseArray msg;
        msg.poses = poses;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = originTfName_;

        targetPosesDebugPub_.publish(msg);
    }
}