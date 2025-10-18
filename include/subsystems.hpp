#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::MotorGroup intake({-6, -5});
inline pros::MotorGroup intakeTop({4});

inline ez::Piston scraper('A');
inline ez::Piston horns('E');
inline ez::Piston intakePiston('B');