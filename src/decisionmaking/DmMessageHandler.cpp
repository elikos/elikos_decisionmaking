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
    ros::NodeHandle n_p("~");
    n_p.getParam("target_array_topic", targetArrayTopic_);
    n_p.getParam("cmd_topic", cmdTopic_);
    n_p.getParam("origin_tf_name", originTfName_);
    n_p.getParam("quad_tf_name", quadTfName_);
    n_p.getParam("simulation", isSimulation_);

    // setup subscribers
    targetRobotArraySub_ = nh_.subscribe<elikos_msgs::TargetRobotArray>(targetArrayTopic_, 1, &DmMessageHandler::targetRobotArrayCallback, this);
    
    // setup publishers
    dmCmdPub_ = nh_.advertise<elikos_msgs::DMCmd>(cmdTopic_, 1);
}

DmMessageHandler::~DmMessageHandler() {
    // \todo add if needed
}

void DmMessageHandler::update() {
    updateQuadTf();
}

void DmMessageHandler::updateQuadTf() {
    // lookup stamped transform
    tf::StampedTransform stampedTf;
    try {
        tfListener_.lookupTransform(originTfName_, quadTfName_, ros::Time(0), stampedTf);
    } catch(tf::TransformException e) {
        ROS_ERROR("[DM MESSAGE HANDLER] error looking up quad tf");
        return;
    }

    // create stamped tf pose
    tf::Stamped<tf::Pose> poseTf;
    poseTf.setOrigin(stampedTf.getOrigin());
    poseTf.setRotation(stampedTf.getRotation());
    // \todo updateQuad(stampedTf);
}

void DmMessageHandler::targetRobotArrayCallback(const elikos_msgs::TargetRobotArray::ConstPtr& msg) {
    // \todo handle target robot array
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
