#ifndef ELIKOS_DUNGEONMASTER_INFORMATIONMANAGER_H
#define ELIKOS_DUNGEONMASTER_INFORMATIONMANAGER_H

/**
 * \file InformationManager.h
 * \brief InformationManager class declaration
 * \author christophebedard
 */

#include <list>
#include <geometry_msgs/Pose.h>
#include "Behaviour.h"
#include "TargetRobot.h"

/**
 * \class InformationManager
 * \brief class which manages decision-making information.
 *
 * Manages and processes input information so that the decision-making architecture can use it.
 */
class InformationManager
{
public:
    /**
     * \brief Instance accessor.
     * 
     * \return the instance.
     */
    static InformationManager* getInstance();

    /**
     * \brief Instance deletion.
     */
    static void freeInstance();

    /**
     * \brief Update quad pose.
     * 
     * \param pose : the pose of the quad.
     */
    void updateQuad(const geometry_msgs::Pose& pose);

    /**
     * \brief Quad pose accessor.
     * 
     * \param pose : the pose of the quad.
     */
    geometry_msgs::Pose getQuadPose() const;

    /**
     * \brief Update target robot array with newest array.
     * 
     * \param msg : the target robot array message.
     */
    void updateTargets(const elikos_msgs::TargetRobotArray::ConstPtr& msg);

    /**
     * \brief Checks if target list is non-empty.
     * 
     * \return true if >= 1 target.
     */
	bool hasTarget() const;

    /**
     * \brief Find and get closest target to the quad.
     * 
     * \return pointer to closest target to quad.
     */
	TargetRobot* getClosestTargetToQuad() const;

    /**
     * \brief Find and get closest target to the green line.
     * 
     * \return pointer to closest target to the green line.
     */
	TargetRobot* getClosestTargetToGreenLine() const;

private:
    static InformationManager* instance_; /**< the instance itself */
    
    std::list<TargetRobot*> targets_; /**< the list of target robots */
    Quad* quad_; /**< the quad */

    /**
     * \brief Private constructor.
     */
    InformationManager();

    /**
     * \brief Private destructor.
     */
    ~InformationManager();

    /**
     * \brief Deleted copy constructor.
     */
    InformationManager(const InformationManager&) = delete;

    /**
     * \brief Deleted copy assignment.
     */
    void operator=(const InformationManager&)  = delete;

};

#endif // ELIKOS_DUNGEONMASTER_INFORMATIONMANAGER_H