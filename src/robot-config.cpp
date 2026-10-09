#include "robot-config.h"

#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "lemlib/api.hpp"




// PLACEHOLDER PORTS — update once build team finalizes wiring
pros::MotorGroup left_motors({1,2}, pros::MotorGears::blue);
pros::MotorGroup right_motors({-3,-4}, pros::MotorGears::blue);
pros::MotorGroup cascade ({-20,19}, pros::MotorGears::green);

pros::Motor claw_turn(-13, pros::MotorGears::green);
pros::Motor claw_spin(8, pros::MotorGears::green);


// PLACEHOLDER PORT — update once build team confirms ADI wiring
// PLACEHOLDER PORTS, update once wiring is confirmed
pros::adi::Pneumatics aux_piston_a('A', false);
pros::adi::Pneumatics aux_piston_b('B', false);
pros::Imu imu (11);
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
    10,   // kP
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
// PLACEHOLDER PORTS — update once build team confirms wiring.
// Chosen to avoid the IMU (11) and claw_turn (13).
pros::Distance distance_sensor(15);
pros::Rotation vertical_rotation(12);
pros::Rotation horizontal_rotation(14);
pros::Rotation claw_rotation(16);

// PLACEHOLDER OFFSETS — update once tracking wheel mounting is finalized (offset is distance from tracking center, in inches)
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotation, lemlib::Omniwheel::NEW_2, -1);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rotation, lemlib::Omniwheel::NEW_2, -1);

// Odometry: vertical + horizontal pods for distance, IMU for heading
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel,
    nullptr,
    &horizontal_tracking_wheel,
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
