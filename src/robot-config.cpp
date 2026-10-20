#include "robot-config.h"

// Motor ports: a negative port reverses that motor.
pros::MotorGroup left_motors({1, 4}, pros::MotorGears::blue);
pros::MotorGroup right_motors({-2, -3}, pros::MotorGears::blue);
pros::MotorGroup cascade({-20, 19, -18}, pros::MotorGears::green);

pros::Motor claw_spin(17, pros::MotorGears::green);
// Lift rotation sensor on the middle cascade axis (port 14).
// Readings decrease as this lift rises.
pros::Rotation lift_rot(14);
pros::Rotation horiz_odom(-6);   // Reversed horizontal pod; offset is 2 inches behind the turn center.
pros::Imu imu(8);

// Measured on the robot.
lemlib::Drivetrain drivetrain(
    &left_motors,
    &right_motors,
    11.02f,                      // track width (inches), measured 28 cm
    lemlib::Omniwheel::NEW_325,  // 3.25" omni wheels
    384,                         // measured wheel rpm
    2                            // horizontal drift compensation
);

// Driving PID. Re-tune using DRV+24 after drivetrain changes.
lemlib::ControllerSettings lateral_controller(
    13,   // kP
    0,    // kI
    3,    // kD
    3,    // anti-windup range
    1,    // small error range (inches)
    0,    // small error timeout (ms): 0 = ends the instant it is inside the small range, no settle wait
    3,    // large error range (inches)
    500,  // large error timeout (ms)
    0     // max acceleration slew
);

// Turning PID. Re-tune using TURN+90 after drivetrain changes.
lemlib::ControllerSettings angular_controller(
    2,    // kP
    0,    // kI
    7,    // kD
    3,    // anti-windup range
    1,    // small error range (degrees)
    0,    // small error timeout (ms): 0 = ends the instant it is inside the small range, no settle wait
    3,    // large error range (degrees)
    500,  // large error timeout (ms)
    0     // max acceleration slew
);

#if USE_TRACKING_WHEELS
#if USE_VERTICAL_POD
pros::Rotation vertical_rotation(12);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotation, lemlib::Omniwheel::NEW_2, -1);
#endif
#if USE_HORIZONTAL_POD
lemlib::TrackingWheel horizontal_tracking_wheel(&horiz_odom, lemlib::Omniwheel::NEW_2, -2);
#endif
#endif

lemlib::OdomSensors sensors(
#if USE_TRACKING_WHEELS && USE_VERTICAL_POD
    &vertical_tracking_wheel,
#else
    nullptr,
#endif
    nullptr,
#if USE_TRACKING_WHEELS && USE_HORIZONTAL_POD
    &horizontal_tracking_wheel,
#else
    nullptr,
#endif
    nullptr,
    &imu
);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);
