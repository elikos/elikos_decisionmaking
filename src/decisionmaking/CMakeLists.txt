include_directories(
  	${catkin_INCLUDE_DIRS}
)

add_executable(elikos_decisionmaking2018
    DecisionMaking.cpp
    DmMessageHandler.cpp
    DmMessageHandler.h
    InformationManager.cpp
    InformationManager.h
    Quad.cpp
    Quad.h
    TargetRobot.cpp
    TargetRobot.h
    #command/Command.h
    #behaviour/Behaviour.h
    #strategy/Strategy.h
    #trategy/TestFollowRobotStrategy.h
)

target_link_libraries(elikos_decisionmaking2018
    ${catkin_LIBRARIES}
)
