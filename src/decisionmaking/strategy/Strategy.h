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
 * Gets launched, then updated and switches between behaviours accordingly.
 *      * launch() is to be called first every time. However, 
 *          the concrete class may or may not call takeoff().
 *      * stop() is to be called last every time. However,
 *          the concrete class may or may not call land().
 *      * (?) land() may be called at any time to safely land the drone.
 */
class Strategy
{
public:
    Strategy();
    virtual ~Strategy();

    /**
     * \brief Strategy launch. Might call takeoff(), depending on concrete strategy.
     */
    virtual void launch() =0;

    /**
     * \brief Strategy stop. Might call land(), depending on concrete strategy.
     */
    virtual void stop() =0;

    /**
     * \brief Update strategy. Changes behaviour if needed.
     */
    virtual void update() =0;

    /**
     * \brief One-time takeoff/init.
     * \todo should this be protected?
     */
    void takeoff();

    /**
     * \brief Land (forced or not).
     * \todo should this be protected, or public for safety?
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
    Behaviour* currentBehaviour_; ///< the current behaviour

private:
    

};

#endif // ELIKOS_DUNGEONMASTER_STRATEGY_H