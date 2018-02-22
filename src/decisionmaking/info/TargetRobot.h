#ifndef ELIKOS_DUNGEONMASTER_TARGETROBOT_H
#define ELIKOS_DUNGEONMASTER_TARGETROBOT_H

/**
 * \file TargetRobot.h
 * \brief TargetRobot class declaration
 * \author christophebedard
 */

#include <cmath>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>

/**
 * \class TargetRobot
 * \brief class which encapsulates relevant info for a target robot.
 */
class TargetRobot
{
public:
    /**
     * \brief Constructor.
     * 
     * \param pos : the position of the target robot.
     * \param incertitudeCountMax : maximum value for incertitude count.
     */
    TargetRobot(geometry_msgs::Point pos, int incertitudeCountMax);

    /**
     * \brief Destructor.
     */
    ~TargetRobot();

    /**
     * \brief Position accessor.
     * 
     * \return the latest position.
     */
    geometry_msgs::Point getPosition() const;

    /**
     * \brief Orientation accessor.
     * 
     * \return the latest orientation.
     */
    double getOrientation() const;

    /**
     * \brief Incertitude count accessor.
     * 
     * \return the incertitude count.
     */
    int getIncertitudeCount() const;

    /**
     * \brief Create and return pose.
     * 
     * \return the pose.
     */
    geometry_msgs::Pose getPose() const;

    /**
     * \brief Update target robot position+orientation and reset counter.
     * 
     * \param pos : the latest position.
     */
    void updatePosition(const geometry_msgs::Point& pos);

    /**
     * \brief Increment incertitude counter (while keeping it below the maximum value).
     * When target robot was not seen after an update.
     */
    void incrementIncertitudeCounter();

private:
    geometry_msgs::Point pos_; /**< the position of the target robot */
    double orientation_; /**< the orientation of the target robot */
    int incertitudeCount_; /**< the counter of updates since it was last seen */

    int incertitudeCountMax_; /**< maximum value for incertitude count */

    /**
     * \brief Update orientation by estimating it from last positions.
     * 
     * \param newPos : the latest position.
     */
    void updateOrientation(const geometry_msgs::Point& newPos);


    /**
     * \brief Deleted default constructor.
     * If a new target robot is created, its instant position has to be known.
     */
    TargetRobot() = delete;
};

#endif // ELIKOS_DUNGEONMASTER_TARGETROBOT_H