/**
 * @file Kinematic.cpp
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

#include "epos_handler/kinematic.h"
#include "math.h"

Kinematic::Velocity::Velocity() {}
Kinematic::Velocity::~Velocity() {}
Kinematic::Velocity::Velocity(const float &vx, const float &vy, const float &w) {
  this->vx = vx;
  this->vy = vy;
  this->w = w;
}
void Kinematic::Velocity::setVelocity(const float &vx, const float &vy, const float &w) {
  this->vx = vx;
  this->vy = vy;
  this->w = w;
}
Kinematic::RPM::RPM() {}
Kinematic::RPM::~RPM() {}
Kinematic::RPM::RPM(const float &rpml, const float &rpmr) {
  left = rpml;
  right = rpmr;
}
void Kinematic::RPM::setRPM(const float &rpml, const float &rpmr) {
  left = rpml;
  right = rpmr;
}


Kinematic::RPM  Kinematic::calculateRPM(const Velocity& velocity) {
  RPM controlInput;
//   Kinematic::desperation.checkVelocity(velocity);
  controlInput.setRPM( velocity.vx /  Kinematic::A_ - velocity.w /  Kinematic::B_,
                       velocity.vx /  Kinematic::A_ + velocity.w /  Kinematic::B_);
  while(!isValidRPM(controlInput))adjustRPM(controlInput);
//   if (Kinematic::desperation.timeIsUp()) Kinematic::desperation.kickInTheAss(controlInput);
  return controlInput;
}

Kinematic::Velocity Kinematic::calculateVelocity(const RPM& controlInput) {
  Velocity newVelocity(  (controlInput.right + controlInput.left) / 2 *  Kinematic::A_,
                         0,
                         (controlInput.right - controlInput.left) / 2 *  Kinematic::B_);

  return newVelocity;
}


Kinematic::RPM  Kinematic::adjustRPM(RPM& rpm) {
  float c = 0.1;
  Velocity tempVel = calculateVelocity(rpm);
  if(!isValidRPM(rpm)) {
    ROS_DEBUG("got in new code");
    c = abs(rpm.left) > abs(rpm.right) ? (MAX_RPM_ / abs(rpm.left)) : (MAX_RPM_ / abs(rpm.right));
    tempVel.setVelocity(c * tempVel.vx,
                        c * tempVel.vy,
                        c * tempVel.w
                       );
    rpm = calculateRPM(tempVel);
  }
  while(!isValidRPM(rpm)) {
    ROS_DEBUG("Got in loop");
    tempVel.setVelocity(0.9 * tempVel.vx,
                        0.9 * tempVel.vy,
                        0.9 * tempVel.w
                       );
    rpm = calculateRPM(tempVel);
  }
  ROS_DEBUG("Kinematic: New robot speed : x = %f \ny= %f \nw = %f  \n'", tempVel.vx, tempVel.vy, tempVel.w);
  return rpm;
}

bool  Kinematic::isValidRPM(const RPM& rpm) {
  bool flag = true;

  if(abs(rpm.left) > MAX_RPM_) {
    ROS_DEBUG("mainMotorControl: robot speed to high: '%f rpm'", rpm.left);
    flag = false;
  }
  if(abs(rpm.right) > MAX_RPM_) {

    ROS_DEBUG("mainMotorControl: robot speed to high: '%f rpm'", rpm.right);
    flag = false;
  }
  return flag;
}

bool Kinematic::isOnlyOneTrackSpinning(const Velocity &u) {
  bool retVal;
  if ( (u.w != 0) && ( ( u.w < 1.05 * (B_ / A_) *u.vx ) && ( u.w > 0.95 * (B_ / A_) ) ) ) {
    ROS_ERROR("CHECKING VELOCITIES RECEIVED:POSSIBLY ONLY ONE TRACK MOVING");
    retVal = true;
  }
  else retVal = false;
  return retVal;
}

/*PANIKOS!!! An doulepsei to navigation sta lofakia na diagrafei!

Kinematic::InterventionTrigger::InterventionTrigger():sameVelocityTimer(0.0001){
    oldVelocity.setVelocity(0,0,0);
}



Kinematic::InterventionTrigger::~InterventionTrigger(){}
void Kinematic::InterventionTrigger::checkVelocity(Kinematic::Velocity newVelocity){
    float diff_x=abs((oldVelocity.vx-newVelocity.vx)/oldVelocity.vx);
    float diff_w=abs((oldVelocity.w-newVelocity.w)/oldVelocity.w);
    if ((diff_w<0.05) && (diff_x<0.05) && isOnlyOneTrackSpinning(newVelocity) ){
        sameVelocityTimer=ros::Time::now()-timestamp;
    }
    else
        sameVelocityTimer=(ros::Duration)0;
        oldVelocity.vx=oldVelocity.w;
        newVelocity.vx=oldVelocity.w;
        timestamp=ros::Time::now();
}

bool Kinematic::InterventionTrigger::timeIsUp(){
    bool retVal;
    if (sameVelocityTimer.sec>=60) retVal=true;
    else retVal=false;
    return retVal;
}

void Kinematic::InterventionTrigger::kickInTheAss(RPM& controlword){
    if (controlword.left>controlword.right){
        controlword.right=(controlword.right/controlword.right)*500;
    }
}
*/
