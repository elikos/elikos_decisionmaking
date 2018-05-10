#ifndef ELIKOS_DUNGEONMASTER_TESTFOLLOWROBOTSTRATEGY_H
#define ELIKOS_DUNGEONMASTER_TESTFOLLOWROBOTSTRATEGY_H

/**
 * \file TestFollowRobotStrategy.h
 * \brief TestFollowRobotStrategy class declaration
 * \author christophebedard
 */

#include "Strategy.h"

/**
 * \class TestFollowRobotStrategy
 * \brief class which defines a Strategy to test robot tracking and following.
 *
 * 
 */
class TestFollowRobotStrategy : public Strategy
{
public:
    TestFollowRobotStrategy();
    virtual ~TestFollowRobotStrategy();

    /**
     * \brief Update strategy. Changes behaviour if needed.
     */
    virtual void update();

    /**
     * \brief Get strategy name.
     * 
     * \return the strategy name.
     */
    virtual std::string getName();

protected:


private:
    static const std::string STRATEGY_NAME; ///< the name of the behaviour

};

#endif // ELIKOS_DUNGEONMASTER_TESTFOLLOWROBOTSTRATEGY_H