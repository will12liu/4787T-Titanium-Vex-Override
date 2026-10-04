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
#if USE_CLAW_PISTON
pros::adi::Pneumatics claw_piston('A', false);
#endif
pros::Imu imu (11);
pros::Motor intake_motor(18, pros::MotorGears::green);


#if USE_LEMLIB_ODOM
// PLACEHOLDER VALUES — track width and wheel diameter are guesses until
// chassis dimensions are locked. Update once build team finalizes chassis.
lemlib::Drivetrain drivetrain(
    &left_motors,
    &right_motors,
    10.748f,                          // track width (inches), PLACEHOLDER
    3.25f,    // wheel diameter, PLACEHOLDER
    600,                         // drivetrain rpm
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

// Drivetrain-only odometry: no tracking wheels yet, IMU + motor encoders only
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel,
    nullptr,
    &horizontal_tracking_wheel,
    nullptr,
    &imu
);

pros::Distance distance_sensor(11);

// PLACEHOLDER PORTS — update once build team confirms wiring
pros::Rotation vertical_rotation(12);
pros::Rotation horizontal_rotation(13);
pros::Rotation claw_rotation(14);

// PLACEHOLDER OFFSETS — update once tracking wheel mounting is finalized (offset is distance from tracking center, in inches)
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotation, lemlib::Omniwheel::NEW_2, -1);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rotation, lemlib::Omniwheel::NEW_2, -1);
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);
#endif

bool claw_turn_horizontal = false;
