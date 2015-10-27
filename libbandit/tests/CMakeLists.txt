add_executable(testbandit testbandit.cpp)
target_link_libraries(testbandit bandit ${Boost_LIBRARIES})

add_executable(servohold servohold.cpp)
target_link_libraries(servohold bandit ${Boost_LIBRARIES})
