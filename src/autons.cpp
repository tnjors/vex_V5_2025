#include "main.h"
#define OPTICAL_PORT 19

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 120;
const int SWING_SPEED = 110;

// Additional speed constants for autonomous routines
const int SLOW_DRIVE_SPEED = 30;
const int MEDIUM_DRIVE_SPEED = 40;
const int VERY_SLOW_DRIVE_SPEED = 25;
const int INTAKE_SPEED = 120;
const int INTAKE_MEDIUM_SPEED = 100;
const int INTAKE_SORT_SPEED = 90;

// Timing constants (in milliseconds)

const int CENTER_REJECT = 1500;  // 1400
const int PICKUP_AUTOS = 400;    // 800
const int RING_EJECT_DELAY = 1000;
const int RING_INTAKE_DELAY = 900;
const int SETTLING_DELAY = 50;

// ----------------------------------------------------------------------------
// Program Constants
// ----------------------------------------------------------------------------

// 1. Generic Drive

enum OrientationEnum { LEFT = -1,
                       RIGHT = 1 };

const int GENERIC_INITIAL_ANGLE = 29;
const int GENERIC_DRIVE_DISTANCE_CENTER = 15.5;
const int GENERIC_CENTER_PICKUP_DRIVE = 12;
const int GENERIC_TURN_TO_LOADER = 179;
const int GENERIC_DRIVE_TO_LOADER = 11;
const int GENERIC_DRIVE_DEPOSIT = -30;
const int GENERIC_DRIVE_AWAY_LONG = 12;
const int GENERIC_TURN_PAR1 = 135;
const int GENERIC_DRIVE_PAR = -16;
const int GENERIC_TURN_PAR2 = -180;

// 2. Skills

const int DRIVE_DISTANCE_CYCLE2 = 31.5;
const int DEPOSIT_DELAY_SKILLS = 2100;

// ----------------------------------------------------------------------------
// PID Constants
// ----------------------------------------------------------------------------
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();

  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .

// ------------------------ Helper Functions ------------------------

/**
 * Drive the robot forward or backward.
 *
 * \param n
 *        Distance to drive in inches. Positive = forward, negative = backward.
 */
void drive(float n) {
  chassis.pid_drive_set(n, 95);
  chassis.pid_wait();
}

/**
 * Turn the robot relative to the current heading.
 *
 * \param n
 *        Angle to turn in degrees. Positive = right, negative = left.
 */
void turnRel(float n) {
  chassis.pid_turn_set(n, TURN_SPEED);
  chassis.pid_wait();
}

/**
 * Generic autonomous routine that works for both left and right sides.
 * Picks up rings, scores in the alliance goal, and positions for endgame.
 *
 * \param orientation
 *        LEFT or RIGHT to mirror the routine for the appropriate starting position
 */
void genericDrive(OrientationEnum orientation) {
  // Move to center and collect first ring
  turnRel(GENERIC_INITIAL_ANGLE * orientation);
  intake.move(INTAKE_SPEED);
  horns.set(true);

  drive(GENERIC_DRIVE_DISTANCE_CENTER);
  chassis.pid_drive_set(GENERIC_CENTER_PICKUP_DRIVE, SLOW_DRIVE_SPEED);
  chassis.pid_wait();
  intake.move(0);

  if (orientation == LEFT) {
    turnRel(135 * orientation);
    drive(-15);  //-13
    outtake.set(true);
    intakeTop.move(INTAKE_SORT_SPEED);
    intake.move(INTAKE_SORT_SPEED);
    pros::delay(RING_EJECT_DELAY);
    intakeTop.move(0);
    intake.move(0);
    outtake.set(false);
    drive(52);  // 50

  } else {
    turnRel(-45);
    // chassis.pid_turn_set(-45, TURN_SPEED);
    // chassis.pid_wait();

    drive(13);

    intake.move(-INTAKE_SORT_SPEED);
    pros::delay(CENTER_REJECT);  // RING_EJECT_DELAY
    intake.move(0);

    //______________________________________________

    drive(-6);
    turnRel(133 * orientation);  // 135 * orient

    drive(43);  // 44.5// & 43
  }

  // master.rumble(".");

  // Move to center goal and score
  scraper.set(true);
  // pros::delay(SETTLING_DELAY);
  turnRel(GENERIC_TURN_TO_LOADER);  // 180

  // intake.move(INTAKE_MEDIUM_SPEED);

  intake.move(INTAKE_SPEED);

  chassis.pid_drive_set(GENERIC_DRIVE_TO_LOADER, 110);
  chassis.pid_wait_quick_chain();

  pros::delay(PICKUP_AUTOS - 300);  // pickup delay for loader !!!! 300

  // Move to far goal

  // drive(-30); //-30

  chassis.pid_drive_set(GENERIC_DRIVE_DEPOSIT, 110);  //-31
  chassis.pid_wait_quick_chain();                     // quick chain

  intakeTop.move(INTAKE_SPEED);

  pros::delay(RING_EJECT_DELAY + 600);  //+300
  scraper.set(false);
  horns.set(true);

  intakeTop.move(0);
  intake.move(0);

  drive(GENERIC_DRIVE_AWAY_LONG);

  // Position for endgame for right
  // turnRel(135);
  chassis.pid_turn_set(GENERIC_TURN_PAR1, 110);
  chassis.pid_wait_quick_chain();

  // chassis.pid_turn_set(135, 120);
  // chassis.pid_wait_quick();

  // drive(-16);  //-14
  chassis.pid_drive_set(GENERIC_DRIVE_PAR, 110);  //-31
  chassis.pid_wait();                             // quick chain

  // turnRel(-180);
  chassis.pid_turn_set(-GENERIC_TURN_PAR2, 110);
  chassis.pid_wait_quick_chain();

  horns.set(false);

  chassis.pid_drive_set(-15, DRIVE_SPEED);
  chassis.pid_wait();
}

/**
 * Solo win point autonomous routine.
 * Optimized path for scoring the win point goal on the right side.
 */
void soloWinPoint_old() {
  turnRel(26);
  intake.move(INTAKE_SPEED);
  drive(16);
  chassis.pid_drive_set(12, MEDIUM_DRIVE_SPEED);
  chassis.pid_wait();
  intake.move(0);

  turnRel(-45);
  drive(13);

  intake.move(-INTAKE_SPEED);
  pros::delay(RING_INTAKE_DELAY);
  intake.move(0);

  drive(-6);
  turnRel(-97);
  drive(28);
  intake.move(INTAKE_MEDIUM_SPEED);
  chassis.pid_drive_set(10, MEDIUM_DRIVE_SPEED);
  chassis.pid_wait();

  turnRel(-135);
  drive(-14);
  intake.move(INTAKE_SPEED);
  intakeTop.move(-INTAKE_SPEED);
  pros::delay(RING_INTAKE_DELAY);
  intakeTop.move(0);
  intake.move(0);

  drive(53);

  scraper.set(true);
  horns.set(true);
  turnRel(180);
  intake.move(INTAKE_MEDIUM_SPEED);
  drive(14);
  pros::delay(RING_EJECT_DELAY);
  intake.move(0);

  horns.set(false);
  drive(-29);
  intake.move(INTAKE_SPEED);
  intakeTop.move(INTAKE_SPEED);

  // Ensure all motors are stopped
  intake.move(0);
  intakeTop.move(0);
}

void soloWinPoint() {
  // drive(32);

  // chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  // 2. Set "Soft" PID constants
  // Lower the first number (kP).
  // If your default is 10, try 4.0 or 5.0 for a long, smooth ramp down.
  // Format: (kP, kI, kD)
  chassis.pid_drive_constants_set(8.0, 0, 5.0);

  // 3. Run the drive command
  // The robot will now start slowing down much earlier
  chassis.pid_drive_set(33, 120);
  chassis.pid_wait();

  // 4. Reset to your normal "Sharp" constants
  // (Replace these numbers with whatever you have in your drive.cpp setup)
  // chassis.pid_drive_constants_forward_set(11.0, 0, 45.0);
  // chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

  // chassis.pid_drive_set(32, 85, true);  // 13
  // chassis.pid_wait();

  scraper.set(true);

  turnRel(90);

  intake.move(127);

  // chassis.pid_drive_set(14, 100);  // 13
  // chassis.pid_wait_quick_chain();// not quick chain

  drive(13);

  intakeTop.move(127);

  // pros::delay(PICKUP_AUTOS - 300);  // pickup delay for loader !!!! - 300
  // pros::delay(100);  // pickup delay for loader !!!! - 300

  intakeTop.move(0);

  scraper.set(false);

  chassis.pid_drive_set(-30, 110);
  chassis.pid_wait_quick_chain();

  intakeTop.move(127);

  pros::delay(RING_EJECT_DELAY + 600);  //+300 // 600

  intake.move(0);
  intakeTop.move(0);

  chassis.pid_turn_set(190, 100);
  chassis.pid_wait_quick_chain();

  intakeTop.move(120);
  intake.move(120);

  drive(19);  // 16

  intakeTop.move(80);

  turnRel(180);

  chassis.pid_drive_constants_forward_set(11.0, 0, 45.0);

  intakeTop.move(0);

  chassis.pid_drive_set(24, 110);  // 25
  chassis.pid_wait_quick_chain();

  turnRel(-190);
  chassis.pid_drive_set(11, 80);
  chassis.pid_wait_quick_chain();

  scraper.set(true);
  outtake.set(true);

  turnRel(135);
  intake.move(0);

  drive(-17.5);  //-16
  intakeTop.move(INTAKE_SORT_SPEED);
  intake.move(INTAKE_SORT_SPEED);
  pros::delay(RING_EJECT_DELAY + 800);
  intakeTop.move(0);
  intake.move(0);

  drive(51);  // 48
  outtake.set(false);

  chassis.pid_turn_constants_set(11.0, 0, 65.0);

  // turnRel(90);
  chassis.pid_turn_set(90, 110);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-17.5, 110);  //-18
  chassis.pid_wait_quick_chain();

  intakeTop.move(120);
  intake.move(120);

  // pros::delay(RING_EJECT_DELAY + 600);  //+300 // 600

  // drive(12);

  // // Position for endgame for right
  // turnRel(-135);
  // drive(-16);  //-14
  // turnRel(-90);
  // horns.set(false);

  // chassis.pid_drive_set(-13, DRIVE_SPEED);
  // chassis.pid_wait_quick_chain();
}

/**
 * Skills autonomous routine.
 * Maximizes score by collecting and scoring multiple rings across the field.
 */
void skills_old() {
  turnRel(29);
  intake.move(INTAKE_SPEED);
  horns.set(true);

  drive(16);
  chassis.pid_drive_set(12, VERY_SLOW_DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(500);
  intake.move(0);

  turnRel(-45);
  drive(13);

  intake.move(-INTAKE_SPEED);
  pros::delay(RING_EJECT_DELAY);
  intake.move(0);

  drive(-6);
  turnRel(-97);
  drive(33);
  intake.move(INTAKE_MEDIUM_SPEED);
  chassis.pid_drive_set(10, VERY_SLOW_DRIVE_SPEED);
  chassis.pid_wait();

  turnRel(-135);
  drive(-16);
  intake.move(INTAKE_SPEED);
  intakeTop.move(-INTAKE_SPEED);
  pros::delay(RING_INTAKE_DELAY);
  intakeTop.move(0);
  intake.move(0);

  drive(52);

  scraper.set(true);
  horns.set(true);
  turnRel(180);

  intake.move(INTAKE_MEDIUM_SPEED);

  drive(15);
  drive(-0.5);

  // Settling movements to ensure all rings score
  drive(-2);
  drive(2);
  drive(-2);
  drive(2);
  drive(-2);
  drive(2);

  pros::delay(500);

  horns.set(false);

  drive(-33);

  intakeTop.move(INTAKE_SPEED);
  pros::delay(500);
  intake.move(INTAKE_SPEED);

  pros::delay(2000);

  intakeTop.move(0);
  pros::delay(500);
  intake.move(0);

  drive(12);
  horns.set(true);
  drive(-14);

  scraper.set(false);

  drive(14);
  turnRel(90);
  drive(48);
  turnRel(180);

  intake.move(INTAKE_MEDIUM_SPEED);

  drive(30);

  // Ensure all motors are stopped
  intake.move(0);
  intakeTop.move(0);
}

void skills_old2() {
  // scraper.set(true);
  // outtake.set(true);
  drive(38);  // 39
  scraper.set(true);

  turnRel(-90);

  // scraper.set(true);
  intake.move(127);
  intakeTop.move(120);
  // -------------

  // drive(12);
  chassis.pid_drive_set(12, 90);
  chassis.pid_wait_quick_chain();

  pros::delay(1800);
  intakeTop.move(0);
  intake.move(0);

  chassis.pid_drive_set(-30, 110);
  chassis.pid_wait_quick_chain();

  scraper.set(false);

  // start moving long

  turnRel(0);
  drive(12);    // 13
  turnRel(88);  // compensating for drift along the lng goal 89
  drive(81);    // 83

  // Move across field

  turnRel(0);
  drive(-14);
  // outtake.set(true);

  // turn and align

  turnRel(90);
  // drive(-14);  //-14
  chassis.pid_drive_set(-15.5, 110);
  chassis.pid_wait_quick_chain();

  // move to long goal

  scraper.set(true);
  // outtake.set(true);  //!!!!

  intake.move(127);
  intakeTop.move(127);
  pros::delay(2000);
  // intake.move(0);
  intakeTop.move(0);

  // deposit

  // intake.move(120);

  //  outtake.set(false);

  chassis.pid_drive_set(30, 110);
  chassis.pid_wait_quick_chain();

  intakeTop.move(110);

  pros::delay(1800);
  intake.move(0);
  intakeTop.move(0);

  // scraper.set(false);

  // pick up loader 2

  chassis.pid_drive_set(-30, 110);
  chassis.pid_wait_quick_chain();

  intake.move(127);
  intakeTop.move(120);
  pros::delay(2000);
  intake.move(0);
  intakeTop.move(0);

  // deposit (2) long goal

  scraper.set(false);

  drive(12);

  turnRel(176);

  // CHANGE SIDES

  drive(93);  // 83

  turnRel(90);
  scraper.set(true);

  // ---------

  // scraper.set(true);
  // intake.move(127);

  // -------------

  intake.move(127);
  intakeTop.move(120);

  // drive(12);
  chassis.pid_drive_set(16, 90);  // 15, 60
  chassis.pid_wait_quick_chain();

  pros::delay(1800);
  intake.move(0);
  intakeTop.move(0);

  //
  chassis.pid_drive_set(-30, 110);
  chassis.pid_wait_quick_chain();
  scraper.set(false);
  // turnRel(90);

  // start moving long

  turnRel(180);
  drive(14);     // 13
  turnRel(264);  // 266
  drive(81);     // 83

  // Move across field

  turnRel(180);
  drive(-16);  //-14
  // outtake.set(true);

  // turn and align

  turnRel(268);  // 270
  chassis.pid_drive_set(-16, 110);
  chassis.pid_wait_quick_chain();

  // move to long goal

  scraper.set(true);
  // outtake.set(true);  //!!!!

  intake.move(127);
  intakeTop.move(120);
  pros::delay(2000);
  // intake.move(0);
  intakeTop.move(0);
  // turnRel(268);

  // deposit

  // intake.move(120);

  //  outtake.set(false);

  chassis.pid_drive_set(30, 110);
  chassis.pid_wait_quick_chain();

  pros::delay(1800);
  intake.move(0);
  // scraper.set(false);

  // pick up loader 2

  drive(-29);  // 29
  intake.move(127);
  intakeTop.move(120);
  pros::delay(1800);
  // intake.move(0);
  // intakeTop.move(0);

  // park

  scraper.set(false);

  drive(10);
  turnRel(-5);
  drive(45);  // 48

  turnRel(-95);

  drive(40);
}

void skills() {
  chassis.pid_drive_constants_set(8, 0, 18.0);

  drive(48);

  scraper.set(true);

  turnRel(-90);

  intake.move(127);
  intakeTop.move(127);

  drive(14);

  chassis.drive_set(20, 20);

  pros::delay(550);  // pickup delay for loader !!!! - 300

  chassis.drive_set(0, 0);

  intakeTop.move(0);

  scraper.set(false);
  horns.set(true);

  drive(-12);
  turnRel(45);
  drive(17);
  turnRel(88);  // compensating for drift along the lng goal 89
  drive(76);    // 83

  // Move across field

  turnRel(0);
  drive(-13);

  turnRel(90);

  horns.set(false);
  scraper.set(true);

  drive(-15.5);

  intake.move(127);
  intakeTop.move(127);
  pros::delay(DEPOSIT_DELAY_SKILLS);

  drive(3);
  horns.set(true);
  drive(-3.2);

  // drive(DRIVE_DISTANCE_CYCLE2);
  chassis.pid_drive_set(DRIVE_DISTANCE_CYCLE2 - 5, 95);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(5, 65);
  chassis.pid_wait();

  chassis.drive_set(20, 20);

  pros::delay(600);  // pickup delay for loader !!!! - 300

  chassis.drive_set(0, 0);

  drive(-1 * DRIVE_DISTANCE_CYCLE2);
  horns.set(false);
  scraper.set(false);
  pros::delay(DEPOSIT_DELAY_SKILLS);

  drive(12);

  turnRel(180);

  drive(96);

  turnRel(90);

  // goal2

  horns.set(true);

  chassis.pid_drive_set(DRIVE_DISTANCE_CYCLE2 - 12 - 5, 95);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(5, 65);
  chassis.pid_wait();

  chassis.drive_set(20, 20);

  pros::delay(600);  // pickup delay for loader !!!! - 300

  chassis.drive_set(0, 0);
}
/**
 * Main autonomous selector functions.
 * These are called from the autonomous selector in main.cpp
 */

void drive_left() {
  genericDrive(LEFT);
  master.rumble("..-");
}

void drive_right() {
  genericDrive(RIGHT);
  master.rumble("..-");

  // intake.move(-INTAKE_SORT_SPEED);
  // pros::delay(CENTER_REJECT);  // RING_EJECT_DELAY
  // intake.move(0);
}

void drive_swp() {
  soloWinPoint();
  // master.rumble("..-");
}

void drive_skills() {
  skills();
  master.rumble("-.-");
}

void driveInch() {
  drive(2);
  master.rumble(".");
}

void driveRight2() {
  turnRel(29);
  intake.move(INTAKE_SPEED);
  horns.set(true);
  outtake.set(false);

  drive(16);
  chassis.pid_drive_set(12, SLOW_DRIVE_SPEED);
  chassis.pid_wait();
  // pros::delay(COLOR_SORT_DELAY);
  intake.move(0);

  turnRel(133);  // 135 * orient
  drive(36);     // 44.5//

  master.rumble(".");

  // Move to center goal and score
  scraper.set(true);
  // pros::delay(SETTLING_DELAY);
  turnRel(179);  // 180

  // intake.move(INTAKE_MEDIUM_SPEED);

  intake.move(120);

  chassis.pid_drive_set(11, 110);
  chassis.pid_wait();

  intakeTop.move(120);

  pros::delay(PICKUP_AUTOS - 200);  // pickup delay for loader !!!! 300

  intakeTop.move(0);

  chassis.pid_drive_set(-30, 110);
  chassis.pid_wait_quick_chain();

  intakeTop.move(120);

  pros::delay(RING_EJECT_DELAY + 1200);  //+300// ring+600
  scraper.set(false);
  horns.set(true);

  intakeTop.move(0);
  intake.move(0);

  drive(12);

  // Position for endgame for right
  turnRel(135);
  drive(-16);  //-14
  turnRel(-180);
  horns.set(false);

  chassis.pid_drive_set(-15, DRIVE_SPEED);
  chassis.pid_wait();
  // drive(-8);
}

// chassis.pid_turn_set(90, 100);
// chassis.pid_wait_quick_chain();
