#include "main.h"
#include "robot-config.h"
constexpr double CLAW_TURN_VERTICAL_DEG = 60.0;
constexpr double CLAW_TURN_HORIZONTAL_DEG = 1000.0;
constexpr int CLAW_TURN_VELOCITY = 200.0;
constexpr int CLAW_TURN_DOWN_VELOCITY = 100.0;
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */


void opcontrol() {
	printf("DBG opcontrol start\n");
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	bool claw_open = false;
	bool intake_running = false;
	bool claw_spin_in = false;
	bool claw_spin_out = false;	
    claw_turn.tare_position();

	while (true) {
		// 1. Drivetrain (tank/arcade, omni wheels)
			int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
			int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

			// Deadband: kill stick noise near center
			if (std::abs(dir) < 10) dir = 0;
			if (std::abs(turn) < 10) turn = 0;

			int left = dir + turn;
			int right = dir - turn;

			// Clamp: keep summed values in valid motor range
			left = std::clamp(left, -127, 127);
			right = std::clamp(right, -127, 127);

			left_motors.move(left);
			right_motors.move(right);

		// Intake: R1 in, R2 out
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
    		intake_running = !intake_running;
		}
		if (intake_running) {
			intake_motor.move(127);
		} else {
			intake_motor.move(0);
		}

		/** 
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
				intake_motor.move(-127);
			} else if (intake_running) {
				intake_motor.move(127);
			} else {
				intake_motor.move(0);
		}
				*/

		// Cascade: L1 up, L2 down
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			cascade.move(127);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			cascade.move(-127);
		} else {
			cascade.move(0);
		}

		// Banshee claw pneumatic: X toggles
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			claw_open = !claw_open;
#if USE_CLAW_PISTON
			claw_piston.set_value(claw_open);
#endif
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
			claw_spin_in = !claw_spin_in;
			claw_spin_out = false;
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			claw_spin_out = !claw_spin_out;
			claw_spin_in = false;
		}
			if (claw_spin_in) {
				claw_spin.move(127);
			} else if (claw_spin_out) {
				claw_spin.move(-60);
			} else {
				claw_spin.move(0);
			}

        //Turn Banshee Claw up: press B, press again to return
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
			claw_turn_horizontal = !claw_turn_horizontal;
			double target = claw_turn_horizontal ? CLAW_TURN_HORIZONTAL_DEG : CLAW_TURN_VERTICAL_DEG;
			int velocity = claw_turn_horizontal ? CLAW_TURN_VELOCITY : CLAW_TURN_DOWN_VELOCITY;
			claw_turn.move_absolute(target, velocity);
}
       


		pros::delay(20);
	}
}