#ifndef ELIKOS_DUNGEONMASTER_INFORMATIONMANAGER_H
#define ELIKOS_DUNGEONMASTER_INFORMATIONMANAGER_H

/**
 * \file InformationManager.h
 * \brief InformationManager class declaration
 * \author christophebedard
 */

#include <string>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include "elikos_msgs/TargetRobotArray.h"
//#include "behaviour/Behaviour.h"
#include "DmMessageHandler.h"
#include "info/Quad.h"
#include "info/TargetRobot.h"

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
     * \brief Check if there is at least one valid target.
     * 
     * \return true if >= 1 valid target.
     */
	bool hasTarget() const;

    /**
     * \brief Get the number of valid targets (incertitude count below the threshold).
     * 
     * \return number of valid targets.
     */
    int getNumValidTargets() const;

    /**
     * \brief Find and get closest target to the quad.
     * 
     * \return the pointer to closest target to quad if hasTarget(), otherwise nullptr.
     */
	TargetRobot* getClosestTargetToQuad() const;

    /**
     * \brief Find and get closest target to the green line.
     * 
     * \return the pointer to closest target to the green line if hasTarget(), otherwise nullptr.
     */
	TargetRobot* getClosestTargetToGreenLine() const;

private:
    static InformationManager* instance_;   ///< the instance itself
    
    std::vector<TargetRobot*>* targets_;    ///< the vector of target robots
    Quad* quad_;                            ///< the quad

    int arenaDimension_;                    ///< the dimension of the arena (side)
    int targetIncertitudeCountThreshold_;   ///< the threshold defining valid targets
    int targetIncertitudeCountMax_;         ///< the maximum value for target incertitude

    /**
     * \brief Get iterator of target closest to quad.
     * 
     * \return the iterator.
     */
    std::vector<TargetRobot*>::iterator getItClosestTargetToQuad() const;

    /**
     * \brief Get iterator of target closest to green line.
     * 
     * \return the iterator.
     */
    std::vector<TargetRobot*>::iterator getItClosestTargetToGreenLine() const;

    /**
     * \brief Compute squared distance between quad and given target.
     * 
     * \param target : the target robot.
     * 
     * \return the squared distance.
     */
    double distanceSquaredQuadTarget(TargetRobot* target) const;

    /**
     * \brief Helper method to compute squared distance between pose and point.
     * 
     * \param pose : the pose.
     * \param point : the position.
     * 
     * \return the squared distance.
     */
    double distanceSquared(const geometry_msgs::Pose& pose, const geometry_msgs::Point& point) const;

    /**
     * \brief Helper method to compute squared distance between two points.
     * 
     * \param p1 : the first point.
     * \param p2 : the second point.
     * 
     * \return the squared distance.
     */
    double distanceSquared(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) const;

    /**
     * \brief Helper method to compute distance between the given target and the green line.
     * Assuming the green line is at (y = arenaDimension_ / 2).
     * 
     * \param target : the target.
     * 
     * \return the distance.
     */
    double distanceTargetToGreenLine(TargetRobot* target) const;

    /**
     * \brief Retrieve a vector of target poses.
     * 
     * \return the vector of poses.
     */
    std::vector<geometry_msgs::Pose> getTargetPoses() const;

    /**
     * \brief Compute squared distance between each current target and each new target.
     * 
     * \param targets : the vector of targets.
     * \param newTargetPoints : the vector of new target points.
     * 
     * \param distances : the reference to the vector of computed squared distances.
     */
    void computeDistances(std::vector<TargetRobot*>* targets, const std::vector<geometry_msgs::Point>& newTargetPoints, std::vector<std::vector<double>>& distances);

    /**
     * \brief Find the (i,j) indexes of the minimum distance between two targets that are not already taken.
     * 
     * \param distances : the distances.
     * \param isCurrentTargetAssigned : the bool vector indicating which current targets have been assigned.
     * \param isNewTargetAssigned : the bool vector indicating which new targets have been assigned.
     * 
     * \param iMinDist : the reference to the i index of the minimum distance.
     * \param jMinDist : the reference to the j index of the minimum distance.
     */
    void findMinimumDistanceIndexes(const std::vector<std::vector<double>>& distances, const std::vector<bool>& isCurrentTargetAssigned, const std::vector<bool>& isNewTargetAssigned, int& iMinDist, int& jMinDist);

    /**
     * \brief Generate a MarkerArray using target poses.
     * 
     * \return the MarkerArray message.
     */
    visualization_msgs::MarkerArray generateMarkerArray() const;

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
    void operator=(const InformationManager&) = delete;

};

#endif // ELIKOS_DUNGEONMASTER_INFORMATIONMANAGER_H