#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::MotorGroup intake({-4, -15});
inline pros::MotorGroup intakeTop({14});

inline ez::Piston scraper('C');
inline ez::Piston horns('D');
inline ez::Piston outtake('A');
inline ez::Piston intakePiston('B');
// A - outtake; C - scraper; D - horn / arm; 


// inline ez::Piston doublePark('G');
