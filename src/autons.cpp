#include "main.h"

// These are out of 127
const int DRIVE_SPEED = POWER/2;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

/*
 * Constants
 */
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Keeps robot straight
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control (odom)
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control (odom)

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

  // The amount that turns are prioritized over driving w/ odom 
  // W/ tracking wheels, values can be set higher (1.0 max)
  chassis.odom_turn_bias_set(0.9);
  chassis.odom_look_ahead_set(7_in);           // How far ahead in path bot looks at
  chassis.odom_boomerang_distance_set(16_in);  // Max carrot dist away from target
  chassis.odom_boomerang_dlead_set(0.625);     // Aggressive value of end of boomerang motions
  chassis.pid_angle_behavior_set(ez::shortest);  // Changes default behavior for turning
}

/*
 * Helper wrapper functions to combine set + wait
 */
template<typename T>
inline void drive(T distance, int speed = DRIVE_SPEED, bool slew = false) {
  chassis.pid_drive_set(distance, speed, slew);
  chassis.pid_wait();
}

template<typename T>
inline void turn(T angle, int speed = TURN_SPEED) {
  chassis.pid_turn_set(angle, speed);
  chassis.pid_wait();
}

template<typename T>
inline void swing(ez::e_swing type, T angle, int speed = SWING_SPEED, int still_speed = 45) {
  chassis.pid_swing_set(type, angle, speed, still_speed);
  chassis.pid_wait();
}

template<typename T>
inline void odom_drive(T distance, int speed = DRIVE_SPEED, bool slew = false) {
  chassis.pid_odom_set(distance, speed, slew);
  chassis.pid_wait();
}

// Example functions
void drive_example() {
  // First param - target in inches
  // Second param - max bot speed 
  // Third param - toggle slew
  // For slew, only enable it when drive dist > slew dist + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}
void turn_example() {
  // First param - target in degrees
  // Second param - max bot speed

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}
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
void wait_until_change_speed() {
  // pid_wait_until will wait until bot gets to desired position
  // When bot gets to 6 inches slowly, bot will travel remaining dist at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, bot will go remaining dist at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When bot gets to -6 inches slowly, bot will travel remaining dist at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, bot will go remaining dist at DRIVE_SPEED
  chassis.pid_wait();
}
void motion_chaining() {
  // Motion chaining - blend motions instead of individual movements
  // Works by exiting while bot is still moving
  // To use this, replace pid_wait with pid_wait_quick_chain
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Final motion should still be normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}
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

// Own autonomous functions
void head_auton(int s) {
  // U turn to loader
  drive(0.6_tile);
  turn(s * 90_deg);
  drive(1.13_tile, TURN_SPEED, true);
  turn(s * 180_deg);

  // Scraper extend
  scraperPneu.extend();
  pros::delay(500);

  // Collecting
  turn(s * 180_deg);
  inMotor.move(POWER * 0.8);
  outMotor.move(-POWER);
  drive(0.7_tile, DRIVE_SPEED * 1.5);
  drive(-0.02_tile);
  drive(0.2_tile, DRIVE_SPEED * 1.5);
  turn(s * 183_deg);
  pros::delay(200);
  
  // Back up and lift
  drive(-0.4_tile);
  loaderPneu.extend();
  scraperPneu.retract();
  turn(s * 180_deg, TURN_SPEED * 0.5);
  inMotor.move(-POWER);
  pros::delay(100);
  inMotor.move(0);

  // Score
  drive(-1_tile, DRIVE_SPEED, true);
  turn(s * 180_deg, TURN_SPEED * 0.5);
  scraperPneu.extend();
  pros::delay(500);
  outMotor.move(POWER);
  inMotor.move(POWER);

  // Straighten up
  turn(s * 180_deg, TURN_SPEED * 0.5);
  drive(0.05_tile, DRIVE_SPEED);
  turn(s * 175_deg, TURN_SPEED * 0.5);
  drive(-0.3_tile, DRIVE_SPEED * 1.5);
  turn(s * 180_deg, TURN_SPEED * 0.5);

  pros::delay(5'000);

  // Push
  loaderPneu.retract();
  drive(0.06_tile, DRIVE_SPEED);
  drive(-0.1_tile, DRIVE_SPEED);

}

void right_side_auton() {
  head_auton(1);
}

void left_side_auton() {
  head_auton(-1);
}
