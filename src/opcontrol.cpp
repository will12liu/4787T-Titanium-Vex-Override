#include "main.h"
#include "robot-config.h"
constexpr double CLAW_TURN_VERTICAL_DEG = 0.0;
constexpr double CLAW_TURN_HORIZONTAL_DEG = 100.0;
constexpr int CLAW_TURN_VELOCITY = 60;
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	bool claw_open = false;
    claw_turn.tare_position();

	while (true) {
		int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		left_motors.move(dir + turn);
		right_motors.move(dir - turn);

		// Intake: R1 in, R2 out
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intake_motor.move(127);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intake_motor.move(-127);
		} else {
			intake_motor.move(0);
		}

		// Cascade: L1 up, L2 down
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			cascade_motor.move(127);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			cascade_motor.move(-127);
		} else {
			cascade_motor.move(0);
		}

		// Banshee claw pneumatic: X toggles
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			claw_open = !claw_open;
			claw_piston.set_value(claw_open);
		}

		// Banshee claw spin motor: Y spins in, A spins out
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
			claw_spin.move(127);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			claw_spin.move(-127);
		} else {
			claw_spin.move(0);
		}
        //Turn Banshee Claw up: press B, press again to return
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            claw_turn_horizontal = !claw_turn_horizontal;
            double target = claw_turn_horizontal ? CLAW_TURN_HORIZONTAL_DEG : CLAW_TURN_VERTICAL_DEG;
            claw_turn.move_absolute(target, CLAW_TURN_VELOCITY);
}
       


		pros::delay(20);
	}
}