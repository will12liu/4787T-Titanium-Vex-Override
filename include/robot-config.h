#pragma once
#include "main.h"
#include "pros/distance.hpp"
#include "pros/rotation.hpp"
#include "lemlib/api.hpp"

extern pros::MotorGroup right_motors;
extern pros::MotorGroup left_motors;
extern pros::MotorGroup cascade;
// Set to 1 once the IMU, tracking wheels and distance sensor are physically wired
// and ports/offsets are confirmed. Off = these objects are not created at startup.
#define USE_LEMLIB_ODOM 0
// Pneumatic claw. Off while bisecting a startup problem; set to 1 once air is plumbed.
#define USE_CLAW_PISTON 1

extern pros::Motor claw_turn;
#if USE_CLAW_PISTON
extern pros::adi::Pneumatics claw_piston;
#endif
extern pros::Motor claw_spin;

// PLACEHOLDER PORT — update once build team confirms IMU mounting/wiring
extern pros::Imu imu;

#if USE_LEMLIB_ODOM
extern lemlib::Drivetrain drivetrain;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::OdomSensors sensors;
extern lemlib::Chassis chassis;
#endif
extern pros::Motor intake_motor;
// Add drivetrain motors, IMU, other sensors here as the robot gets built
#if USE_LEMLIB_ODOM
extern pros::Distance distance_sensor;
extern pros::Rotation vertical_rotation;
extern pros::Rotation horizontal_rotation;
extern pros::Rotation claw_rotation;
extern lemlib::TrackingWheel vertical_tracking_wheel;
extern lemlib::TrackingWheel horizontal_tracking_wheel;
#endif
extern bool claw_turn_horizontal;
