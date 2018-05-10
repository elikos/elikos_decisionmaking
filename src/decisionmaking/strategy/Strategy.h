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
 * Gets updated, then switches between behaviours accordingly.
 */
class Strategy
{
public:
    Strategy();
    virtual ~Strategy();

    /**
     * \brief Update strategy. Changes behaviour if needed.
     */
    virtual void update() =0;

    /**
     * \brief One-time takeoff/init.
     */
    void takeoff();

    /**
     * \brief Stop and land.
     * Should
     */
    void land();

    /**
     * \brief Get strategy name.
     * 
     * \return the strategy name.
     */
    virtual std::string getName() =0;

    /**
     * \brief Get current behaviour.
     * 
     * \return the behaviour.
     */
    Behaviour* getCurrentBehaviour();

protected:
    Behaviour* currentBehaviour_; /**< the current behaviour

private:
    

};

#endif // ELIKOS_DUNGEONMASTER_STRATEGY_H