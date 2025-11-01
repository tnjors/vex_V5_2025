#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::MotorGroup intake({-4, -15});
inline pros::MotorGroup intakeTop({14});

inline ez::Piston scraper('C');
inline ez::Piston horns('A');
inline ez::Piston intakePiston('B');