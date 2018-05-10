#ifndef ELIKOS_DUNGEONMASTER_TESTSHAPESTRATEGY_H
#define ELIKOS_DUNGEONMASTER_TESTSHAPESTRATEGY_H

/**
 * \file TestShapeStrategy.h
 * \brief TestShapeStrategy class declaration
 * \author christophebedard
 */

#include "Strategy.h"

/**
 * \class TestShapeStrategy
 * \brief test
 *
 * 
 */
class TestShapeStrategy : public Strategy
{
public:
    TestShapeStrategy();
    virtual ~TestShapeStrategy();

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
    static const std::string STRATEGY_NAME; /**< the name of the behaviour */

};

#endif // ELIKOS_DUNGEONMASTER_TESTSHAPESTRATEGY_H