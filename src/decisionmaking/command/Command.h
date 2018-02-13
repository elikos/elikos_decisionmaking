#ifndef ELIKOS_DUNGEONMASTER_COMMAND_H
#define ELIKOS_DUNGEONMASTER_COMMAND_H

/**
 * \file Command.h
 * \brief asbtract command class declaration
 * \author christophebedard
 * \author Olivier
 */

#include <ros/ros.h> // \todo remove?

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

    double hasReachedDestinationThreshold_; /**< threshold for checking is destination is reached */

    /**
     * \brief Checks if the destination was reached; useful for position-dependant commands.
     * 
     * \todo add necessary params
     * 
     * \return true when destination was reached.
     */
    bool hasReachedDestination();


private:
    
};

#endif // ELIKOS_DUNGEONMASTER_COMMAND_H