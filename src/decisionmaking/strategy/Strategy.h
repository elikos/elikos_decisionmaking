#ifndef ELIKOS_DUNGEONMASTER_STRATEGY_H
#define ELIKOS_DUNGEONMASTER_STRATEGY_H

/**
 * \file Strategy.h
 * \brief asbtract strategy class declaration
 * \author christophebedard
 */

#include "Behaviour.h"

/**
 * \class Strategy
 * \brief asbtract class which defines a strategy.
 *
 * Gets launched, then switches between behaviours accordingly. One-time launch.
 */
class Strategy
{
public:
    Strategy();
    virtual ~Strategy();

    /**
     * \brief Launch strategy.
     */
    virtual void launch() =0;

protected:
    

private:
    

};

#endif // ELIKOS_DUNGEONMASTER_STRATEGY_H