#ifndef ELIKOS_DUNGEONMASTER_STRATEGYFACTORY_H
#define ELIKOS_DUNGEONMASTER_STRATEGYFACTORY_H

/**
 * \file StrategyFactory.h
 * \brief StrategyFactory class declaration
 * \author christophebedard
 */

#include <memory>
#include <string>
#include "TestFollowRobotStrategy.h"
#include "TestShapeStrategy.h"


/**
 * \class StrategyFactory
 * \brief class for Strategy creation.
 */
class StrategyFactory
{
public:
    /**
     * \brief Create a new Strategy instance.
     *
     * \param strategyName : the name of the strategy.
     *
     * \return the new instance of corresponding type.
     */
    static std::unique_ptr<Strategy> create(std::string strategyName);

private:
    static const std::string STRATEGY_NAME_TESTFOLLOWROBOT; ///< the name for TestFollowRobotStrategy
    //static const std::string STRATEGY_NAME_; ///< 
    //static const std::string STRATEGY_NAME_; ///< 
    //static const std::string STRATEGY_NAME_; ///< 

};

#endif // ELIKOS_DUNGEONMASTER_STRATEGYFACTORY_H