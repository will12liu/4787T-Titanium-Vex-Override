#pragma once
#include "main.h"
#include "pros/distance.hpp"
#include "pros/rotation.hpp"
#include "lemlib/api.hpp"

extern pros::MotorGroup right_motors;
extern pros::MotorGroup left_motors;
extern pros::MotorGroup cascade;

// Cascade lift position sensor. lift_pos() reads this instead of a motor count.
// PORT IS A GUESS (unused port): set it to the real port. Negative = reversed.
constexpr int CASCADE_ROTATION_PORT = 9;
// Sensor degrees per cascade-motor degree (1.0 if the sensor turns 1:1 with the
// lift gearing the old motor-degree heights were measured on). Heights in
// mechanisms.h stay valid if this is right.
constexpr double CASCADE_ROTATION_RATIO = 1.0;
extern pros::Rotation cascade_rotation;

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

extern pros::Motor claw_turn;
extern pros::Motor claw_spin;
extern pros::adi::Pneumatics aux_piston_a;
extern pros::adi::Pneumatics aux_piston_b;
extern pros::Rotation horiz_odom;

// PLACEHOLDER PORT — update once build team confirms IMU mounting/wiring
extern pros::Imu imu;

// LemLib chassis + odometry. Needs the IMU; without it the LemLib auton
// commands are skipped.
extern lemlib::Drivetrain drivetrain;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::OdomSensors sensors;
extern lemlib::Chassis chassis;
extern pros::Motor intake_motor;
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
extern bool claw_turn_horizontal;
