#ifndef ELIKOS_DUNGEONMASTER_TARGETROBOT_H
#define ELIKOS_DUNGEONMASTER_TARGETROBOT_H

/**
 * \file TargetRobot.h
 * \brief TargetRobot class declaration
 * \author christophebedard
 */

#include <geometry_msgs/Point.h>

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
     */
    TargetRobot(geometry_msgs::Point pos);

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
     * \brief Update target robot position+orientation and reset counter.
     * 
     * \param pos : the latest position.
     */
    void updatePose(geometry_msgs::Point pos);

    /**
     * \brief Increment incertitude counter.
     * When target robot was not seen after an update.
     */
    void incrementIncertitudeCounter();

private:
    geometry_msgs::Point pos_; /**< the position of the target robot */
    double orientation_; /**< the orientation of the target robot */
    int incertitudeCount_; /**< the counter of updates since it was last seen */

    /**
     * \brief Deleted default constructor.
     * If a new target robot is created, its instant position is known.
     */
    TargetRobot() = delete;
};

#endif // ELIKOS_DUNGEONMASTER_TARGETROBOT_H