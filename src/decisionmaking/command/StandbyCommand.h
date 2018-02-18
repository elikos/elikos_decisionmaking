#ifndef ELIKOS_DUNGEONMASTER_STANDBYCOMMAND_H
#define ELIKOS_DUNGEONMASTER_STANDBYCOMMAND_H

/**
 * \file StandbyCommand.h
 * \brief StandbyCommand class declaration
 * \author christophebedard
 */

#include <ros/ros.h> // \todo remove?
#include <tf/tf.h>

/**
 * \class StandbyCommand
 * \brief class which defines a standby command.
 *
 * 
 */
class StandbyCommand
{
public:
    StandbyCommand();
    virtual ~StandbyCommand();

    /**
     * \brief Command publishing.
     */
    virtual void publish();

    /**
     * \brief Get command name.
     * 
     * \return the command name.
     */
    virtual std::string getName();

protected:


private:
    static const std::string COMMMAND_NAME; /**< the name of the command */
};

#endif // ELIKOS_DUNGEONMASTER_STANDBYCOMMAND_H