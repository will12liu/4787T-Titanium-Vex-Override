#include "robot-config.h"

#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "lemlib/api.hpp"




// PLACEHOLDER PORTS — update once build team finalizes wiring
pros::MotorGroup left_motors({1,4}, pros::MotorGears::blue);
pros::MotorGroup right_motors({-2,-3}, pros::MotorGears::blue);
pros::MotorGroup cascade ({-20,19}, pros::MotorGears::green);

pros::Motor claw_turn(-17, pros::MotorGears::green);
pros::Motor claw_spin(15, pros::MotorGears::green);
pros::Rotation horiz_odom(-6);   // horizontal odom pod: port 6 (Brain port check), reversed (x went down sliding right). Offset measured: 2" behind the turning center (-2)


// PLACEHOLDER PORT — update once build team confirms ADI wiring
// PLACEHOLDER PORTS, update once wiring is confirmed
pros::adi::Pneumatics aux_piston_a('A', false);
pros::adi::Pneumatics aux_piston_b('B', false);
pros::Imu imu (8);
pros::Motor intake_motor(18, pros::MotorGears::green);


#if USE_LEMLIB_ODOM
// Measured on the robot.
lemlib::Drivetrain drivetrain(
    &left_motors,
    &right_motors,
    11.02f,                      // track width (inches), measured 28 cm
    lemlib::Omniwheel::NEW_325,  // 3.25" omni wheels
    600,                         // drivetrain rpm, direct drive blue
    2                            // horizontal drift compensation
);

// PLACEHOLDER PID GAINS — untuned. Requires empirical tuning on real
// hardware once chassis is finalized. Do not trust these values as-is.
lemlib::ControllerSettings lateral_controller(
    28,   // kP (was 10; higher = faster, more power, can overshoot)
    0,    // kI
    3,    // kD
    3,    // anti-windup range
    1,    // small error range (inches)
    100,  // small error timeout (ms)
    3,    // large error range (inches)
    500,  // large error timeout (ms)
    0     // max acceleration slew
);

// PLACEHOLDER PID GAINS — untuned. Requires empirical tuning on real
// hardware once chassis is finalized. Do not trust these values as-is.
lemlib::ControllerSettings angular_controller(
    2,    // kP
    0,    // kI
    10,   // kD
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

// PLACEHOLDER OFFSETS — update once tracking wheel mounting is finalized (offset is distance from tracking center, in inches)
// Vertical pod (optional, USE_VERTICAL_POD): offset is how far it sits left (-) or right (+)
// of the robot's turning center. If pose y goes DOWN when you push the robot forward,
// reverse the port (-12).
#if USE_VERTICAL_POD
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotation, lemlib::Omniwheel::NEW_2, -1);
#endif
// Horizontal pod: offset is how far it sits behind (-) or in front of (+) the robot's turning
// center. It MUST be measured; a wrong offset adds fake sideways movement whenever the
// robot turns. If pose x goes DOWN when you slide the robot to the right, reverse the port.
#if USE_HORIZONTAL_POD
lemlib::TrackingWheel horizontal_tracking_wheel(&horiz_odom, lemlib::Omniwheel::NEW_2, -2);
#endif

// Odometry: forward/back distance from the vertical pod if there is one, otherwise from
// the drive motor encoders (LemLib falls back to them on nullptr). Sideways from the
// horizontal pod if there is one. Heading from the IMU.
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
// Drivetrain-only odometry: no tracking wheels, LemLib uses the drive motor
// encoders for distance and the IMU for heading.
lemlib::OdomSensors sensors(
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    &imu
);
#endif

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);
#endif

bool claw_turn_horizontal = false;
