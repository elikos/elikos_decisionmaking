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
     * \brief Command execution.
     */
    virtual void execute();

    /**
     * \brief Command completion check.
     * \todo when is this command done?
     * \return true when command is done.
     */
    virtual bool isDone();

protected:


private:
    
};

#endif // ELIKOS_DUNGEONMASTER_STANDBYCOMMAND_H