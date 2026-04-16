#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::MotorGroup intakeTop({-15});


inline pros::MotorGroup intake({16, -17});

inline ez::Piston scraper('C');
inline ez::Piston horns('F');
inline ez::Piston outtake('D');
inline ez::Piston intakePiston('E');
inline ez::Piston hoarder('B');
inline ez::Piston midGoalDeScore('A');
// A - outtake; C - scraper; D - horn / arm;

// inline ez::Piston doublePark('G');
