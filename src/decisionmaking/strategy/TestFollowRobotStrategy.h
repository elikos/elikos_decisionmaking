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

    virtual void launch();

protected:


private:
    

};

#endif // ELIKOS_DUNGEONMASTER_TESTFOLLOWROBOTSTRATEGY_H