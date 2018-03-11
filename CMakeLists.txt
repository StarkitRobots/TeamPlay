cmake_minimum_required(VERSION 2.8)
project(rhoban_team_play)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

find_package(catkin REQUIRED COMPONENTS
  rhoban_utils)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES rhoban_team_play
    CATKIN_DEPENDS rhoban_utils
    )

include_directories(include ${catkin_INCLUDE_DIRS})

add_library (rhoban_team_play src/rhoban_team_play/team_play.cpp)
target_link_libraries(rhoban_team_play ${catkin_LIBRARIES})
