#include "robot-config.h"

pros::Optical optical_sensor(1);

// PLACEHOLDER PORTS — update once build team finalizes wiring
pros::MotorGroup left_motors({1, 2}, pros::MotorGears::blue);
pros::MotorGroup right_motors({-3, -4}, pros::MotorGears::blue);

// PLACEHOLDER PORT — update once build team confirms ADI wiring
pros::adi::Pneumatics claw_piston('A', false);
