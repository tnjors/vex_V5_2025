#include "main.h"
#define OPTICAL_PORT 19

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 110;
const int SWING_SPEED = 110;

// Additional speed constants for autonomous routines
const int SLOW_DRIVE_SPEED = 30;
const int MEDIUM_DRIVE_SPEED = 40;
const int VERY_SLOW_DRIVE_SPEED = 25;
const int INTAKE_SPEED = 127;
const int INTAKE_MEDIUM_SPEED = 100;
const int INTAKE_SORT_SPEED = 90;

// _______________________
const int CENTER_REJECT = 1500;  // 1400
const int PICKUP_AUTOS = 400;    // 800
// _______________________

// Color sensor constants
const int HUE_THRESHOLD_BLUE_RED = 200;  // Hue values > 200 are blue, <= 200 are red
const int OPTICAL_LED_PWM = 50;

// Timing constants (in milliseconds)
const int COLOR_SORT_DELAY = 500;
const int RING_EJECT_DELAY = 1000;
const int RING_INTAKE_DELAY = 900;
const int SETTLING_DELAY = 50;

enum OrientationEnum { LEFT = -1,
                       RIGHT = 1 };

///
// Constants
///
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
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
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

// ------------------------ Color Sort Function ------------------------

pros::Optical optical_sensor(OPTICAL_PORT);

/**
 * Sorts rings by color using the optical sensor.
 * Blue rings (hue > 200) are ejected, red rings (hue <= 200) are kept.
 *
 * \param n
 *        Intake motor speed (0-127).
 * \param x
 *        Total duration in milliseconds before cleaning up.
 */
void intakeSort(int n, int x) {
  optical_sensor.set_led_pwm(OPTICAL_LED_PWM);
  intake.move(n);

  // Blue rings (hue > 200) are ejected, red rings (hue <= 200) are kept
  if (optical_sensor.get_hue() > HUE_THRESHOLD_BLUE_RED) {
    intakeTop.move(-n);
    pros::delay(COLOR_SORT_DELAY);
    intakeTop.move(0);
  } else if (optical_sensor.get_hue() <= HUE_THRESHOLD_BLUE_RED) {
    intakeTop.move(n);
  }

  pros::delay(x);

  // Reset LED when done
  optical_sensor.set_led_pwm(0);
}

// ------------------------------------------------------------------------

/**
 * Generic autonomous routine that works for both left and right sides.
 * Picks up rings, scores in the alliance goal, and positions for endgame.
 *
 * \param orientation
 *        LEFT or RIGHT to mirror the routine for the appropriate starting position
 */
void genericDrive(OrientationEnum orientation) {
  // Move to center and collect first ring
  turnRel(29 * orientation);
  intake.move(INTAKE_SPEED);
  horns.set(true);

  drive(16);
  chassis.pid_drive_set(12, SLOW_DRIVE_SPEED);
  chassis.pid_wait();
  // pros::delay(COLOR_SORT_DELAY);
  intake.move(0);

  if (orientation == LEFT) {
    turnRel(135 * orientation);
    drive(-13);
    intakeTop.move(-INTAKE_SORT_SPEED);
    intake.move(INTAKE_SORT_SPEED);
    pros::delay(RING_EJECT_DELAY);
    intakeTop.move(0);
    intake.move(0);

    drive(50);

  } else {
    turnRel(-45);
    drive(13);

    intake.move(-INTAKE_SORT_SPEED);
    pros::delay(CENTER_REJECT);  // RING_EJECT_DELAY
    intake.move(0);

    //______________________________________________

    drive(-6);
    turnRel(133 * orientation);  // 135*orient
    drive(44.5);                 // 4
  }

  master.rumble(".");

  // Move to center goal and score
  scraper.set(true);
  // pros::delay(SETTLING_DELAY);
  turnRel(179);  // 180

  // intake.move(INTAKE_MEDIUM_SPEED);

  intake.move(120);

  chassis.pid_drive_set(11, 110);
  chassis.pid_wait();

  // Settling movements to ensure rings drop
  // drive(-2);

  // drive(4);
  pros::delay(PICKUP_AUTOS);
  // intake.move(0);

  master.rumble(".");

  // Move to far goal
  hood.set(true);

  drive(-30);
  // intake.move(INTAKE_MEDIUM_SPEED);

  // chassis.pid_drive_set(-3, 60);
  // chassis.pid_wait();

  intakeTop.move(120);
  // intake.move(120);

  pros::delay(RING_EJECT_DELAY + 300);
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

  chassis.pid_drive_set(-5, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  drive(-5);

  // Ensure all motors are stopped
  intake.move(0);
  intakeTop.move(0);
}

/**
 * Solo win point autonomous routine.
 * Optimized path for scoring the win point goal on the right side.
 */
void soloWinPoint() {
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
  pros::delay(COLOR_SORT_DELAY);
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

  pros::delay(COLOR_SORT_DELAY);

  horns.set(false);

  drive(-33);

  intakeTop.move(INTAKE_SPEED);
  pros::delay(COLOR_SORT_DELAY);
  intake.move(INTAKE_SPEED);

  pros::delay(2000);

  intakeTop.move(0);
  pros::delay(COLOR_SORT_DELAY);
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

void skills() {
  // scraper.set(true);
  hood.set(true);
  drive(38);  // 39
  scraper.set(true);

  turnRel(-90);

  // scraper.set(true);
  intake.move(127);

  // -------------

  // drive(12);
  chassis.pid_drive_set(12, 60);
  chassis.pid_wait();

  pros::delay(1800);
  intake.move(0);
  drive(-21);
  scraper.set(false);

  // start moving long

  turnRel(0);
  drive(12);    // 13
  turnRel(88);  // compensating for drift along the lng goal 89
  drive(81);    // 83

  // Move across field

  turnRel(0);
  drive(-14);
  // hood.set(true);

  // turn and align

  turnRel(90);
  drive(-14);  //-14

  // move to long goal

  scraper.set(true);
  // hood.set(true);  //!!!!

  intake.move(127);
  intakeTop.move(120);
  pros::delay(2000);
  // intake.move(0);
  intakeTop.move(0);

  // deposit

  // intake.move(120);

  //  hood.set(false);

  drive(26);
  drive(3);

  pros::delay(1800);
  intake.move(0);
  // scraper.set(false);

  // pick up loader 2

  drive(-29);
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

  // drive(12);
  chassis.pid_drive_set(15, 60);
  chassis.pid_wait();

  intake.move(127);

  pros::delay(1800);
  intake.move(0);
  drive(-21);
  scraper.set(false);
  // turnRel(90);

  // start moving long

  turnRel(180);
  drive(14);  // 13
  turnRel(264);//266
  drive(81);  // 83

  // Move across field

  turnRel(180);
  drive(-13.5);
  // hood.set(true);

  // turn and align

  turnRel(268);//270
  drive(-14);  //-14

  // move to long goal

  scraper.set(true);
  // hood.set(true);  //!!!!

  intake.move(127);
  intakeTop.move(120);
  pros::delay(2000);
  // intake.move(0);
  intakeTop.move(0);
  turnRel(268);

  // deposit

  // intake.move(120);

  //  hood.set(false);

  drive(26);
  drive(3);

  pros::delay(1800);
  intake.move(0);
  // scraper.set(false);

  // pick up loader 2

  drive(-29);
  intake.move(127);
  intakeTop.move(120);
  pros::delay(1800);
  intake.move(0);
  intakeTop.move(0);
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
  master.rumble("..-");
}

void drive_skills() {
  skills();
  master.rumble("-.-");
}

void driveInch() {
  drive(2);
  master.rumble(".");
}
