/**
 * @file Kinematic.h
 * @author  Nikos Zikos
 * @version 1.0
 *
 * @section LICENSE
 * Whatever
 *
 * @section DESCRIPTION
 *
 * Under Const.
 */

#ifndef _KINEMATIC_H
#define	_KINEMATIC_H


#include <stdint.h>
#include <cmath>
#include <ros/ros.h>

namespace Kinematic{
class Position{
public:
    Position();
    ~Position();
    Position(const float& X, const float& Y, const float& TH);
    void setPosition(const float& X, const float& Y, const float& TH);
    float x;				/*!< x coordinate of the robot measured in meters. */
    float y;				/*!< y coordinate of the robot measured in meters. */
    float th;				/*!< orientation of the robot measured in rads from the global X axis counterclockwise. */
};

class Velocity{
public:
    Velocity();
    ~Velocity();
    Velocity(const float& vx, const float& vy, const float& w);
    void setVelocity(const float& velx, const float& vely, const float& radsp);
    float vx;				/*!< Linear speed of the robot in m/sec of the robot's x-axis. */
    float vy;				/*!< Detailed description after the member. */
    float w;				/*!< Angular speed of the robot in rad/sec. */
};

class RPM{
public:
    RPM();
    ~RPM();
    RPM(const float& left, const float& right);
    void setRPM(const float& left, const float& right);
    float left;				/*!< Rotational speed of the left motor in rpm. */
    float right;				/*!< Rotational speed of the right motor in rpm. */
};

/*class InterventionTrigger{
    Velocity oldVelocity;
    ros::Time timestamp;
    ros::Duration sameVelocityTimer;
    void updateTimer();
public:
    void checkVelocity(Velocity newVelocity);
    bool timeIsUp();
    InterventionTrigger();
    ~InterventionTrigger();
    void kickInTheAss(RPM& controlword);
};


static InterventionTrigger desperation;
*/
Velocity calculateVelocity(const RPM& controlInput);
RPM calculateRPM(const Velocity& velocity);
uint32_t encodeToControlWord(RPM rpm);
RPM adjustRPM(RPM& rpm);
bool isValidRPM(const RPM& rpm);
bool isOnlyOneTrackSpinning(const Velocity& u);
static const float A_ = 28.28e-6*0.6595;		/*!< Parameter converting the angular motor speed into linear robot speed */
static const float B_ = 103.0e-6*0.6595;	/*!< Parameter converting the angular motor speed into angular robot speed */
static const float MAX_RPM_=5100;


}
#endif	/* _KINEMATIC_H */

