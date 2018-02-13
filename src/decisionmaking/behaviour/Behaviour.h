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
 * Gets started, then executes a list of commands until it's stopped.
 */
class Behaviour
{
public:
    Behaviour();
    virtual ~Behaviour();

    /**
     * \brief Start behaviour.
     */
    virtual void start() =0;

    /**
     * \brief Stop behaviour.
     * \todo needed?
     */
    virtual void stop() =0;

private:
    // \todo command queue

};

#endif // ELIKOS_DUNGEONMASTER_BEHAVIOUR_H