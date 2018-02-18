#ifndef ELIKOS_DUNGEONMASTER_TARGETROBOT_H
#define ELIKOS_DUNGEONMASTER_TARGETROBOT_H

/**
 * \file TargetRobot.h
 * \brief TargetRobot class declaration
 * \author christophebedard
 */

#include <tf/Vector3.h>

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
    TargetRobot(tf::Vector3 pos);

    /**
     * \brief Destructor.
     */
    ~TargetRobot();

    /**
     * \brief Position accessor.
     * 
     * \return the latest position.
     */
    tf::Vector3 getPosition() const;

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
    void updatePose(tf::Vector3 pos);

    /**
     * \brief Increment incertitude counter.
     * When target robot was not seen after an update.
     */
    void incrementIncertitudeCounter();

private:
    tf::Vector3 pos_; /**< the position of the target robot */
    double orientation_; /**< the orientation of the target robot */
    int incertitudeCount_; /**< the counter of updates since it was last seen */

    /**
     * \brief Deleted default constructor.
     */
    TargetRobot() = delete;
};

#endif // ELIKOS_DUNGEONMASTER_TARGETROBOT_H