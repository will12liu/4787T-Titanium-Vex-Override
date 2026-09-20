#pragma once
#include "main.h"
#include "pros/distance.hpp"
#include "pros/rotation.hpp"
#include "lemlib/api.hpp"

extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::Motor cascade_motor;
extern pros::Motor claw_turn;
extern pros::adi::Pneumatics claw_piston;
extern pros::Motor claw_spin;

// PLACEHOLDER PORT — update once build team confirms IMU mounting/wiring
extern pros::Imu imu;

extern lemlib::Drivetrain drivetrain;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::OdomSensors sensors;
extern lemlib::Chassis chassis;
extern pros::Motor intake_motor;
// Add drivetrain motors, IMU, other sensors here as the robot gets built
extern pros::Distance distance_sensor;
extern pros::Rotation vertical_rotation;
extern pros::Rotation horizontal_rotation;
extern pros::Rotation claw_rotation;
extern lemlib::TrackingWheel vertical_tracking_wheel;
extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern bool claw_turn_horizontal;
