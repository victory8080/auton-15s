#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor intake(9);
inline pros::Motor outtake(-2);
inline pros::adi::DigitalOut outtake_raiser('A');
inline pros::adi::DigitalOut descore('B');
inline pros::adi::DigitalOut loader('H');
inline pros::adi::DigitalOut odomraiser('G');
inline pros::Optical OpticalSensor(20); //optical
inline pros::Distance FrontDis(18);
inline pros::Distance RightDis(17);
// inline pros::GPS GPSsensor(1);