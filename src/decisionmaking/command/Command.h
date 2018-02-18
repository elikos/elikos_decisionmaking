#ifndef ELIKOS_DUNGEONMASTER_COMMAND_H
#define ELIKOS_DUNGEONMASTER_COMMAND_H

/**
 * \file Command.h
 * \brief asbtract command class declaration
 * \author christophebedard
 * \author Olivier
 */

#include <string>
#include <ros/ros.h> // \todo remove?
#include <tf/tf.h>

/**
 * \class Command
 * \brief asbtract class which defines a command.
 *
 * Gets called and publishes itself.
 */
class Command
{
public:
    Command();
    virtual ~Command();

    /**
     * \brief Command publishing.
     */
    virtual void publish() =0;

    /**
     * \brief Get command name.
     * 
     * \return the command name.
     */
    virtual std::string getName() =0;

protected:

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