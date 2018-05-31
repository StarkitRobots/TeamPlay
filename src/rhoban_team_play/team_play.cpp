#include "rhoban_team_play/team_play.h"

#include <rhoban_utils/timing/time_stamp.h>

#include <cmath>

using namespace rhoban_utils;

namespace rhoban_team_play
{

float TeamPlayInfo::getAge() const
{
    return TimeStamp::now().getTimeMS() - timestamp;
}

bool TeamPlayInfo::isOutdated() const
{
    //Outdated after 3 seconds
    return (getAge() > 3000);
}

float TeamPlayInfo::scoreFor(TeamPlayState role) const
{
    if (role == PlacingA) return scoreA;
    if (role == PlacingB) return scoreB;
    if (role == PlacingC) return scoreC;
    if (role == PlacingD) return scoreD;
    return -1;
}
float TeamPlayInfo::getBallDistance() const
{
    return sqrt(ballX*ballX + ballY*ballY);
}

float TeamPlayInfo::getBallAzimuth() const
{
    return atan2(ballY, ballX);
}

float CaptainInfo::getAge() const
{
    return TimeStamp::now().getTimeMS() - timestamp;
}

}
