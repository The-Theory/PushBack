#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

/**********************************************************
 * Robot systems
**********************************************************/
// Chassis
inline ez::Drive chassis(
    // First motor for sensing, negative are reverse
    {-18, -19, -20},  // Left Chassis Ports
    {1, 2, 3},        // Right Chassis Ports
    13,   // IMU Port
    3.0,  // Wheel Diameter
    600 * 60/36); // Wheel RPM = cartridge * (motor gear / wheel gear)

// Subsystems
inline pros::MotorGroup intakeMotors({6, -11});
inline pros::adi::Pneumatics scraperPneu('A', false);
inline pros::adi::Pneumatics intakePneu('H', false);
inline pros::adi::Pneumatics paddlePneu('G', false);