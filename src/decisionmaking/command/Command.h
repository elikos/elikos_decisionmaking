#ifndef ELIKOS_DUNGEONMASTER_COMMAND_H
#define ELIKOS_DUNGEONMASTER_COMMAND_H

/**
 * \file Command.h
 * \brief asbtract command class declaration
 * \author christophebedard
 * \author Olivier
 */

#include <ros/ros.h> // \todo remove?
#include <tf/tf.h>

/**
 * \class Command
 * \brief asbtract class which defines a command.
 *
 * Gets executed and checks when it's done.
 */
class Command
{
public:
    Command();
    virtual ~Command();

    /**
     * \brief Command execution.
     */
    virtual void execute() =0;

    /**
     * \brief Command completion check.
     * 
     * \return true when command is done.
     */
    virtual bool isDone() =0;

protected:
    // \todo add info about quad position (checks if it has reached destination)
    // quad
    // target

    double hasReachedDestinationThreshold_; /**< threshold for checking if destination is reached (value from param server) */

    /**
     * \brief Checks if the destination has been reached; 
     * useful for position-dependant commands.
     * 
     * \param currentPos : current position.
     * \param dest : destination position.
     * 
     * \return true when destination has been reached.
     */
    bool hasReachedDestination(const tf::Vector3& currentPos, const tf::Vector3& dest);

    /**
     * \brief Helper method to send DM commands.
     * \todo implement following message handler implementation
     * \param destPose : destination/target pose.
     * \param cmdCode : command code (according to elikos_msgs::DMCmd).
     */
    void sendDmCmd(const geometry_msgs::Pose& destPose, int cmdCode);

private:
    
};

#endif // ELIKOS_DUNGEONMASTER_COMMAND_H