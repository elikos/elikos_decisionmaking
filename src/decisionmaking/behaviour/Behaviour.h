#ifndef ELIKOS_DUNGEONMASTER_BEHAVIOUR_H
#define ELIKOS_DUNGEONMASTER_BEHAVIOUR_H

/**
 * \file Behaviour.h
 * \brief asbtract behaviour class declaration
 * \author christophebedard
 * \author Olivier
 */

#include "Command.h"

/**
 * \class Behaviour
 * \brief asbtract class which defines a behaviour.
 *
 * Gets updated, then executes and changes command accordingly.
 */
class Behaviour
{
public:
    Behaviour();
    virtual ~Behaviour();

    /**
     * \brief Update behaviour. Changes current command if needed.
     */
    virtual void update() =0;

    /**
     * \brief Get behaviour name.
     * 
     * \return the behaviour name.
     */
    virtual std::string getName() =0;

    /**
     * \brief Get current command.
     * 
     * \return the command.
     */
    Command* getCurrentCommand();

protected:
    Command* currentCmd_; /**< the current command */

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

private:

};

#endif // ELIKOS_DUNGEONMASTER_BEHAVIOUR_H