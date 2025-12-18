#include "main.h"
// Credit to: https://ez-robotics.github.io/EZ-Template/ for initial template code

/**********************************************************
 * Defines for power and controller buttons
**********************************************************/
 // Intake power
#define INT_POWER 127
// Triggers
#define L1 master.get_digital(DIGITAL_L1)
#define L2 master.get_digital(DIGITAL_L2)
#define R1 master.get_digital(DIGITAL_R1)
#define R2 master.get_digital(DIGITAL_R2)
// Buttons
#define X master.get_digital(DIGITAL_X)
#define B master.get_digital(DIGITAL_B)
#define A master.get_digital(DIGITAL_A)
#define Y master.get_digital(DIGITAL_Y)
// Dpad
#define DOWN master.get_digital(DIGITAL_DOWN)
#define UP master.get_digital(DIGITAL_UP)
#define LEFT master.get_digital(DIGITAL_LEFT)
#define RIGHT master.get_digital(DIGITAL_RIGHT)
// Joysticks
#define LEFT_X master.get_analog(ANALOG_LEFT_X)
#define LEFT_Y master.get_analog(ANALOG_LEFT_Y)
#define RIGHT_X master.get_analog(ANALOG_RIGHT_X)
#define RIGHT_Y master.get_analog(ANALOG_RIGHT_Y)



/**********************************************************
 * Initialization code is run as soon as program is started
**********************************************************/
void initialize() {
  ez::ez_template_print();
  pros::delay(500);

  // Chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);
  chassis.opcontrol_curve_default_set(0.0, 0.0);
  default_constants();

  // Auton selector
  ez::as::auton_selector.autons_add({
      {"Right Side", right_side_auton},
      {"Left Side", left_side_auton},
      {"Drive\n\nDrive forward and come back", drive_example},
      {"Turn\n\nTurn 3 times.", turn_example},
      {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn}
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}



/**********************************************************
 * Runs while the robot is in the disabled state
**********************************************************/
void disabled() {
  // Do nothing
}



/**********************************************************
 * Runs after initialize() for competition-specific initialization routines
**********************************************************/
void competition_initialize() {
  // Do nothing
}



/**********************************************************
 * Runs the user autonomous code
**********************************************************/
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}



/**********************************************************
 * Simplifies printing tracker values to the brain screen
**********************************************************/
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}



/**********************************************************
 * Ez screen task
**********************************************************/
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else if (ez::as::page_blank_amount() > 0)
      ez::as::page_blank_remove_all();

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);



/**********************************************************
 * OPControl extras 
 * - Auton: DOWN + B
 * - PID Tuner: Y
**********************************************************/
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // Set values in auton.cpp after finding values that are happy with
    // Increment / Decrement: A and Y
    // Navigation: Arrow keys
    if (Y)
      chassis.pid_tuner_toggle();

    // Trigger the selected auton
    if (B && DOWN) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else if (chassis.pid_tuner_enabled())
    chassis.pid_tuner_disable();
}



/**********************************************************
 * Operator code
**********************************************************/
void opcontrol() {
  // Set chassis brake mode
  chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);

  // Scraper extended
  scraperPneu.extend();
  bool scraperState = true;

  // Main loop
  while (true) {
    ez_template_extras();

    // Standard split arcade
    chassis.opcontrol_arcade_standard(ez::SPLIT);   

    // Intake power
    int intakePwr = 0;
    if (L2) intakePwr = INT_POWER;
    else if (L1) intakePwr = -INT_POWER;
    intakeMotors.move(intakePwr);

    // Paddle extend
    if (R1) paddlePneu.extend();
    else paddlePneu.retract();

    // Intake extend
    if (R2) intakePneu.extend();
    else intakePneu.retract();

    // Scraper toggle
    if (X && !scraperState) scraperPneu.toggle();
    scraperState = X;

    // Delay
    pros::delay(ez::util::DELAY_TIME);
  }
}
