cmake_minimum_required(VERSION 2.8)
project(starkit_team_play)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra")

find_package(catkin REQUIRED COMPONENTS
  starkit_utils)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES starkit_team_play
    CATKIN_DEPENDS starkit_utils
    )

include_directories(include ${catkin_INCLUDE_DIRS})

add_library (starkit_team_play src/starkit_team_play/team_play.cpp)
target_link_libraries(starkit_team_play ${catkin_LIBRARIES})
