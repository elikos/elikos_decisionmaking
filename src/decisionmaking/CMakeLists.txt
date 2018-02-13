include_directories(
  	${catkin_INCLUDE_DIRS}
)

add_executable(elikos_decisionmaking2018
    DecisionMaking.cpp
    command/Command.h
    behaviour/Behaviour.h
    strategy/Strategy.h
    strategy/TestFollowRobotStrategy.h
)

target_link_libraries(elikos_decisionmaking2018
    ${catkin_LIBRARIES}
)
