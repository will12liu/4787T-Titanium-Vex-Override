#pragma once
#include "main.h"
#include "pros/distance.hpp"
#include "pros/rotation.hpp"
#include "lemlib/api.hpp"

extern pros::MotorGroup right_motors;
extern pros::MotorGroup left_motors;
extern pros::MotorGroup cascade;

// LemLib chassis + odometry. Drivetrain-only odom: IMU for heading, drive motor
// encoders for distance. No tracking wheels are needed for this to work.
// Requires the IMU to be plugged in; autonomous() falls back to time-based
// driving if it is not found at startup.
#define USE_LEMLIB_ODOM 1

// Dedicated tracking wheel pods + distance sensor + claw rotation sensor.
// Off until they are physically mounted and ports/offsets are confirmed.
// When on, odom switches from drive encoders to the pods automatically.
#define USE_TRACKING_WHEELS 0

// Pneumatic claw. Off: the claw piston was replaced by aux_piston_a/b (D-pad UP).
// Auton claw_set_open() does nothing while this is 0.
#define USE_CLAW_PISTON 0

extern pros::Motor claw_turn;
extern pros::Motor claw_spin;
extern pros::adi::Pneumatics aux_piston_a;
extern pros::adi::Pneumatics aux_piston_b;

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
#if USE_TRACKING_WHEELS
extern pros::Distance distance_sensor;
extern pros::Rotation vertical_rotation;
extern pros::Rotation horizontal_rotation;
extern pros::Rotation claw_rotation;
extern lemlib::TrackingWheel vertical_tracking_wheel;
extern lemlib::TrackingWheel horizontal_tracking_wheel;
#endif
extern bool claw_turn_horizontal;
