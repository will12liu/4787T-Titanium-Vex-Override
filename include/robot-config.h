#pragma once
#include "main.h"
#include "pros/rotation.hpp"
#include "lemlib/api.hpp"

extern pros::MotorGroup right_motors;
extern pros::MotorGroup left_motors;
extern pros::MotorGroup cascade;

// Current odometry uses drive motor encoders + IMU. Enable mounted pods here.
#define USE_TRACKING_WHEELS 0
#define USE_VERTICAL_POD 0
#define USE_HORIZONTAL_POD 1

extern pros::Motor claw_spin;
extern pros::Rotation horiz_odom;
extern pros::Rotation lift_rot;   // on the middle cascade axis: the real lift height

extern pros::Imu imu;

// LemLib chassis + odometry. Needs the IMU; without it the LemLib auton
// commands are skipped.
extern lemlib::Drivetrain drivetrain;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::OdomSensors sensors;
extern lemlib::Chassis chassis;
#if USE_TRACKING_WHEELS
#if USE_VERTICAL_POD
extern pros::Rotation vertical_rotation;
extern lemlib::TrackingWheel vertical_tracking_wheel;
#endif
#if USE_HORIZONTAL_POD
extern lemlib::TrackingWheel horizontal_tracking_wheel;
#endif
#endif
