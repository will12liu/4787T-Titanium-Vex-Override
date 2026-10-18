#include "robot-config.h"

#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "lemlib/api.hpp"




// PLACEHOLDER PORTS — update once build team finalizes wiring
pros::MotorGroup left_motors({1,4}, pros::MotorGears::blue);
pros::MotorGroup right_motors({-2,-3}, pros::MotorGears::blue);
pros::MotorGroup cascade ({-20,19,-18}, pros::MotorGears::green);

pros::Vision vision_sensor(17);
pros::Motor claw_spin(15, pros::MotorGears::green);
// Lift rotation sensor on the middle cascade axis (port 14).
// Negative port = reversed; flip the sign if the number goes DOWN when the lift goes up.
pros::Rotation lift_rot(14);
pros::Rotation horiz_odom(-6);   // horizontal odom pod: port 6 (Brain port check), reversed (x went down sliding right). Offset measured: 2" behind the turning center (-2)
pros::Imu imu (8);
pros::Motor intake_motor(18, pros::MotorGears::green);



// Measured on the robot.
lemlib::Drivetrain drivetrain(
    &left_motors,
    &right_motors,
    11.02f,                      // track width (inches), measured 28 cm
    lemlib::Omniwheel::NEW_325,  // 3.25" omni wheels
    384,                         // wheel rpm. Was 600 (direct drive), but pushing 24" read 37.5", so 600 * 24 / 37.5
    2                            // horizontal drift compensation
);

// Driving PID. kP / kD tuned with DRV+24 on the Brain's TUNE screen 10/13.
// kP was 28: it braked so hard at the end the robot tipped forward. Raising kP
// makes it brake later and harder, so watch for tipping if you change it.
lemlib::ControllerSettings lateral_controller(
    13,   // kP
    0,    // kI
    3,    // kD
    3,    // anti-windup range
    1,    // small error range (inches)
    100,  // small error timeout (ms)
    3,    // large error range (inches)
    500,  // large error timeout (ms)
    0     // max acceleration slew
);

// Turning PID. kP / kD tuned with TURN+90 on the Brain's TUNE screen.
// Re-tune if the robot's weight or drivetrain changes.
lemlib::ControllerSettings angular_controller(
    2,    // kP (tuned on the TUNE screen 10/13)
    0,    // kI
    7,    // kD (tuned on the TUNE screen 10/13, was 10)
    3,    // anti-windup range
    1,    // small error range (degrees)
    100,  // small error timeout (ms)
    3,    // large error range (degrees)
    500,  // large error timeout (ms)
    0     // max acceleration slew
);

#if USE_TRACKING_WHEELS
// PLACEHOLDER PORT — update once build team confirms wiring.
#if USE_VERTICAL_POD
pros::Rotation vertical_rotation(12);
#endif
#if USE_VERTICAL_POD
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotation, lemlib::Omniwheel::NEW_2, -1);
#endif
#if USE_HORIZONTAL_POD
lemlib::TrackingWheel horizontal_tracking_wheel(&horiz_odom, lemlib::Omniwheel::NEW_2, -2);
#endif

lemlib::OdomSensors sensors(
#if USE_VERTICAL_POD
    &vertical_tracking_wheel,
#else
    nullptr,
#endif
    nullptr,
#if USE_HORIZONTAL_POD
    &horizontal_tracking_wheel,
#else
    nullptr,
#endif
    nullptr,
    &imu
);
#else
lemlib::OdomSensors sensors(
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    &imu
);
#endif

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

bool claw_turn_horizontal = false;
