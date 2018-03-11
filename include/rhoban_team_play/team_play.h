#pragma once

#include <cstdint>

namespace rhoban_team_play
{

/**
 * Robot playing state in teamplay
 */
enum TeamPlayState : int {
    Inactive = 0,
    Playing = 1,
    BallHandling = 2,
    PlacingA = 3,
    PlacingB = 4,
    PlacingC = 5,
    PlacingD = 6
};

/**
 * Robot priority in teamplay
 */
enum TeamPlayPriority : int {
    LowPriority = 0,
    NormalPriority = 1,
    HighPriority = 2,
};

/**
 * TeamPlay mMessage structure
 */
struct TeamPlayInfo {
    //ID of the player
    int id;
    //State of the player
    TeamPlayState state;
    //Player priority
    TeamPlayPriority priority;
    //Ball position in self frame
    float ballX, ballY, ballQ;
    bool ballOk;
    /// Ball speed along x-axis in self referential
    float ballVelX;
    /// Ball speed along y-axis in self referential
    float ballVelY;
    //Robot pose in field
    float fieldX, fieldY, fieldYaw, fieldQ, fieldConsistency;
    bool fieldOk;
    //Distance to placing
    double scoreA, scoreB, scoreC, scoreD;
    // Placing target
    bool placing;
    double targetX, targetY;
    double localTargetX, localTargetY;
    // Ball target
    double ballTargetX, ballTargetY;
    /// Time elapsed since last kick was performed
    float timeSinceLastKick;
    //Referee textual state
    char stateReferee[15];
    //Robocup textual state
    char stateRobocup[10];
    //Playing textual state
    char statePlaying[10];
    //Approach textual state
    char stateSearch[10];
    //Hardware warnings
    char hardwareWarnings[30];
    //Robot clock
    uint8_t hour, min, sec;
    //Timestamp of data reception
    //in milliseconds
    float timestamp;

    /**
     * Return the time in milliseconds
     * since data reception
     */
    float getAge() const;

    /**
     * Return true if data have been
     * received for too long
     */
    bool isOutdated() const;

    /**
     * Score for a given role
     */
    double scoreFor(TeamPlayState role) const;

    /**
     * Return the distance and the
     * azimuth between the robot 
     * and the ball
     */
    float getBallDistance() const;
    float getBallAzimuth() const;
};        

}
