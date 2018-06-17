#include <iostream>
#include "rhoban_team_play/team_play.h"

#include <rhoban_utils/timing/time_stamp.h>

#include <cmath>

using namespace rhoban_utils;

namespace rhoban_team_play
{

void teamPlayfromJson(TeamPlayInfo &info, const Json::Value & json_value)
{
    if (json_value.size() >= 33) {
        int k = 0;
        
        // Robot id and state
        info.id = json_value[k++].asInt();
        info.state = (TeamPlayState)json_value[k++].asInt();
        
        // Ball
        info.ballX = json_value[k++].asFloat();
        info.ballY = json_value[k++].asFloat();
        info.ballQ = json_value[k++].asFloat();
        info.ballOk = json_value[k++].asBool();
        info.ballVelX = json_value[k++].asFloat();
        info.ballVelY = json_value[k++].asFloat();
        
        // Field
        info.fieldX = json_value[k++].asFloat();
        info.fieldY = json_value[k++].asFloat();
        info.fieldYaw = json_value[k++].asFloat();
        info.fieldQ = json_value[k++].asFloat();
        info.fieldConsistency = json_value[k++].asFloat();
        info.fieldOk = json_value[k++].asBool();
        
        // Placing data
        info.placing = json_value[k++].asBool();
        info.targetX = json_value[k++].asFloat();
        info.targetY = json_value[k++].asFloat();
        info.localTargetX = json_value[k++].asFloat();
        info.localTargetY = json_value[k++].asFloat();
        
        // Kick
        info.ballTargetX = json_value[k++].asFloat();
        info.ballTargetY = json_value[k++].asFloat();
        info.timeSinceLastKick = json_value[k++].asFloat();
        
        // States
        strncpy(info.stateReferee, json_value[k++].asString().c_str(), sizeof(info.stateReferee));
        strncpy(info.stateRobocup, json_value[k++].asString().c_str(), sizeof(info.stateRobocup));
        strncpy(info.statePlaying, json_value[k++].asString().c_str(), sizeof(info.statePlaying));
        strncpy(info.stateSearch, json_value[k++].asString().c_str(), sizeof(info.stateSearch));
        strncpy(info.hardwareWarnings, json_value[k++].asString().c_str(), sizeof(info.hardwareWarnings));
        
        // Hour
        info.hour = json_value[k++].asInt();
        info.min = json_value[k++].asInt();
        info.sec = json_value[k++].asInt();
        
        // Obstacles
        info.obstaclesRadius = json_value[k++].asFloat();
        info.nbObstacles = 0;
        for (auto obstacle : json_value[k++]) {
            info.obstacles[info.nbObstacles][0] = obstacle[0].asFloat();
            info.obstacles[info.nbObstacles][1] = obstacle[1].asFloat();
            info.nbObstacles++;
        }
        
        info.timestamp = json_value[k++].asFloat();
        
        info.goalKeeper = json_value[k++].asBool();
    } else {
        std::cerr << "TeamPlayInfo::fromJson bad array size!" << std::endl;    
    }
}
                    
Json::Value teamPlayToJson(const TeamPlayInfo &info)
{
    Json::Value json(Json::arrayValue);
    
    // Robot id and state
    json.append(info.id);
    json.append(info.state);
    
    // Ball
    json.append(info.ballX);
    json.append(info.ballY);
    json.append(info.ballQ);
    json.append(info.ballOk);
    json.append(info.ballVelX);
    json.append(info.ballVelY);
    
    // Field
    json.append(info.fieldX);
    json.append(info.fieldY);
    json.append(info.fieldYaw);
    json.append(info.fieldQ);
    json.append(info.fieldConsistency);
    json.append(info.fieldOk);
    
    // Placing data
    json.append(info.placing);
    json.append(info.targetX);
    json.append(info.targetY);
    json.append(info.localTargetX);
    json.append(info.localTargetY);
    
    // Kick info
    json.append(info.ballTargetX);
    json.append(info.ballTargetY);
    json.append(info.timeSinceLastKick);
    
    // States
    json.append(info.stateReferee);
    json.append(info.stateRobocup);
    json.append(info.statePlaying);
    json.append(info.stateSearch);
    json.append(info.hardwareWarnings);
    
    // Timing
    json.append(info.hour);
    json.append(info.min);
    json.append(info.sec);
    
    // Obstacles
    json.append(info.obstaclesRadius);
    Json::Value obstacles = Json::Value(Json::arrayValue);
    for (int k=0; k<info.nbObstacles; k++) {
        Json::Value obstacle(Json::arrayValue);
        obstacle[0] = info.obstacles[k][0];
        obstacle[1] = info.obstacles[k][1];
        obstacles.append(obstacle);
    }
    json.append(obstacles);
    
    // Timestamp
    json.append(info.timestamp);
    
    // Is the robot a goal keeper ?
    json.append(info.goalKeeper);
    
    return json;
}

float TeamPlayInfo::getAge() const
{
    return TimeStamp::now().getTimeMS() - timestamp;
}

bool TeamPlayInfo::isPenalized() const
{
  return strcmp(stateRobocup, "penalized") == 0;
}

bool TeamPlayInfo::isOutdated() const
{
    //Outdated after 3 seconds
    return (getAge() > 3000);
}

float TeamPlayInfo::getBallDistance() const
{
    return sqrt(ballX*ballX + ballY*ballY);
}

float TeamPlayInfo::getBallAzimuth() const
{
    return atan2(ballY, ballX);
}

Eigen::Vector2d TeamPlayInfo::getBallInField() const
{
  Eigen::Vector2d ball_in_self;
  ball_in_self(0) = fieldX + ballX * cos(fieldYaw) - ballY * sin(fieldYaw);
  ball_in_self(1) = fieldY + ballX * sin(fieldYaw) + ballY * cos(fieldYaw);
  return ball_in_self;
}

CaptainInfo::CaptainInfo()
: id(-1)
{
    for (int k=0; k<CAPTAIN_MAX_ID; k++) {
        order[k] = SearchBall;
        robotTarget[k][0] = 0;
        robotTarget[k][1] = 0;
        robotTarget[k][2] = 0;
    }
    common_ball.nbRobots = 0;
    common_ball.x = 0;
    common_ball.y = 0;
    nb_opponents = 0;
    for (int k=0; k<MAX_OPPONENTS; k++) {
      common_opponents[k].consensusStrength = 0;
      common_opponents[k].x = 0;
      common_opponents[k].y= 0;
    }
}

float CaptainInfo::getAge() const
{
    return TimeStamp::now().getTimeMS() - timestamp;
}

int CaptainInfo::getHandler() const
{
  for (int robot_id = 0; robot_id < CAPTAIN_MAX_ID; robot_id++) {
    if (order[robot_id] == CaptainOrder::HandleBall) {
      return robot_id + 1;
    }
  }
  return -1;
}

void captainFromJson(CaptainInfo &info, const Json::Value & json_value)
{
    if (json_value.size() == 8) {
        int k = 0;
        
        info.id = json_value[k++].asInt();
        
        auto targets = json_value[k++];
        for (int n=0; n<CAPTAIN_MAX_ID; n++) {
            info.robotTarget[n][0] = targets[n][0].asFloat();
            info.robotTarget[n][1] = targets[n][1].asFloat();
            info.robotTarget[n][2] = targets[n][2].asFloat();
        }
        
        auto orders = json_value[k++];
        for (int n=0; n<CAPTAIN_MAX_ID; n++) {
            info.order[n] = (CaptainOrder)orders[n].asInt();
        }
        
        info.timestamp = json_value[k++].asFloat();

        info.common_ball.nbRobots = json_value[k++].asInt();
        info.common_ball.x = json_value[k++].asFloat();
        info.common_ball.y = json_value[k++].asFloat();

        const Json::Value & common_opponents_json = json_value[k++];
        if (!common_opponents_json.isArray()) {
          throw std::logic_error("Common opponents is not an array");
          info.nb_opponents = common_opponents_json.size();
          for (int opp_id = 0; opp_id < MAX_OPPONENTS; opp_id++) {
            if (opp_id < info.nb_opponents) {
              const Json::Value & opp_json = common_opponents_json[opp_id];
              info.common_opponents[opp_id].consensusStrength = opp_json[opp_id].asInt();
              info.common_opponents[opp_id].x = opp_json[opp_id].asFloat();
              info.common_opponents[opp_id].y = opp_json[opp_id].asFloat();
            } else {
              info.common_opponents[opp_id].consensusStrength = 0;
              info.common_opponents[opp_id].x = 0;
              info.common_opponents[opp_id].y = 0;
            }
          }
        }
    } else {
        std::cerr << "CaptainInfo bad json size" << std::endl;    
    }
}
                    
Json::Value captainToJson(const CaptainInfo &info)
{
    Json::Value json;
    
    json.append(info.id);
    
    Json::Value targets(Json::arrayValue);
    for (int k=0; k<CAPTAIN_MAX_ID; k++) {
        Json::Value target(Json::arrayValue);
        target.append(info.robotTarget[k][0]);
        target.append(info.robotTarget[k][1]);
        target.append(info.robotTarget[k][2]);
        targets.append(target);
    }
    json.append(targets);
    
    Json::Value orders(Json::arrayValue);
    for (int k=0; k<CAPTAIN_MAX_ID; k++) {
        orders.append(info.order[k]);
    }
    json.append(orders);
    
    json.append(info.timestamp);

    json.append(info.common_ball.nbRobots);
    json.append(info.common_ball.x);
    json.append(info.common_ball.y);

    Json::Value opponents(Json::arrayValue);
    for (int k = 0; k < info.nb_opponents; k++) {
      Json::Value opp(Json::arrayValue);
      opp.append(info.common_opponents[k].consensusStrength);
      opp.append(info.common_opponents[k].x);
      opp.append(info.common_opponents[k].y);
      opponents.append(opp);
    }
    json.append(opponents);
    
    return json;
}

}
