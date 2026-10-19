#pragma once
#include "main.h"
#include "pros/distance.hpp"
#include "pros/rotation.hpp"
#include "lemlib/api.hpp"

extern pros::MotorGroup right_motors;
extern pros::MotorGroup left_motors;
extern pros::MotorGroup cascade;

// Dedicated tracking wheel pods + distance sensor + claw rotation sensor.
// Off until they are physically mounted and ports/offsets are confirmed.
// Turn this on once the horizontal pod is mounted and plugged in; with it on and
// no pod, odom reads garbage.
#define USE_TRACKING_WHEELS 1

// Which pods exist (only matter when USE_TRACKING_WHEELS is 1). Forward/back
// distance comes from the drive motor encoders unless the vertical pod is on.
// Current plan: horizontal pod only, to catch sideways drift. Heading is the IMU.
#define USE_VERTICAL_POD 0
#define USE_HORIZONTAL_POD 1

extern pros::Motor claw_spin;
extern pros::adi::Pneumatics aux_piston_a;
extern pros::adi::Pneumatics aux_piston_b;
extern pros::Rotation horiz_odom;
extern pros::Rotation lift_rot;   // on the middle cascade axis: the real lift height

// PLACEHOLDER PORT — update once build team confirms IMU mounting/wiring
extern pros::Imu imu;

// LemLib chassis + odometry. Needs the IMU; without it the LemLib auton
// commands are skipped.
extern lemlib::Drivetrain drivetrain;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::OdomSensors sensors;
extern lemlib::Chassis chassis;
// Add drivetrain motors, IMU, other sensors here as the robot gets built
#if USE_TRACKING_WHEELS
#if USE_VERTICAL_POD
extern pros::Rotation vertical_rotation;
#endif
#if USE_VERTICAL_POD
extern lemlib::TrackingWheel vertical_tracking_wheel;
#endif
#if USE_HORIZONTAL_POD
extern lemlib::TrackingWheel horizontal_tracking_wheel;
#endif
#endif
