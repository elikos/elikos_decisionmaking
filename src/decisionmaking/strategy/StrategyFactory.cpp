/**
 * \file StrategyFactory.cpp
 * \brief StrategyFactory class implementation.
 * \author christophebedard
 */

#include "StrategyFactory.h"

std::string StrategyFactory::STRATEGY_NAME_TESTFOLLOWROBOT = "TestFollowRobotStrategy";

std::unique_ptr<Strategy> createStrategy(std::string strategyName) {
    switch (strategyName)
    {
        case default:
        case STRATEGY_NAME_TESTFOLLOWROBOT:
            return std::unique_ptr<TestFollowRobotStrategy>(new TestFollowRobotStrategy());
            break;
        // case "":
        //     strat = new Whatever();
        //     break;
        // case "":
        //     strat = new Whatever();
        //     break;
    }
}

