#include "main.h"
#include "robot-config.h"
#include "mechanisms.h"   // CLAW_TURN_* constants live here now, shared with autonomous
#include "auton.h"

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
	bool intake_running = false;
	bool claw_spin_in = false;
	bool claw_spin_out = false;
	bool aux_open = false;
	// claw_turn is tared once at boot in mech_init(); re-taring here would
	// shift the flip targets after autonomous.
	bool cascade_holding = false;
	double cascade_hold_pos = 0.0;
	cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// Claw turn states: 0 = down/vertical, 1 = horizontal, 2 = lifted off the cone
	int claw_turn_state = 0;
	// Lift is a fraction of the vertical-to-horizontal travel past horizontal.
	// 0.5 = about 45 degrees if horizontal is 90 degrees from vertical. Use 0.33 to 0.67 for 30 to 60.
	// The sign follows the vertical-to-horizontal direction. If it lifts the wrong way, use -0.5.
	const double CLAW_LIFT_FRACTION = 0.5;
	const double claw_turn_lift_deg = CLAW_TURN_HORIZONTAL_DEG +
		(CLAW_TURN_HORIZONTAL_DEG - CLAW_TURN_VERTICAL_DEG) * CLAW_LIFT_FRACTION;
	double claw_turn_target = CLAW_TURN_VERTICAL_DEG;

	while (true) {
		// Auton test: D-pad DOWN runs autonomous() once. Only works with no
		// competition switch / field control plugged in, so it can't fire in a match.
		if (!pros::competition::is_connected() &&
		    master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			autonomous();
			claw_turn_state = 0;
			claw_turn_target = CLAW_TURN_VERTICAL_DEG;
			claw_turn_horizontal = false;
			cascade_holding = false;     // re-capture the lift position after auton moved it
			intake_running = false;
			claw_spin_in = false;
			claw_spin_out = false;
			aux_open = false;
			aux_piston_a.set_value(aux_open);
			aux_piston_b.set_value(aux_open);
		}
		// Drive-only test: R2 runs the selected auton's driving with no lift or claw.
		if (!pros::competition::is_connected() &&
		    master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
			auton_drive_test();
		}

		// 1. Drivetrain
		int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		if (std::abs(dir) < 10) dir = 0;
		if (std::abs(turn) < 10) turn = 0;

		int left = std::clamp(dir + turn, -127, 127);
		int right = std::clamp(dir - turn, -127, 127);

		left_motors.move(left);
		right_motors.move(right);

		// Intake: R1 toggles
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
			intake_running = !intake_running;
		}
		intake_motor.move(intake_running ? 127 : 0);

		// Cascade: L1 up, L2 down
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			cascade.move(127);
			cascade_holding = false;
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			cascade.move(-127);
			cascade_holding = false;
		} else {
			if (!cascade_holding) {
				cascade_hold_pos = cascade.get_position();
				cascade_holding = true;
			}
			cascade.move_absolute(cascade_hold_pos, 100);
		}

		// Claw spin: Y in, A out (toggles)
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

				// Claw turn: B cycles vertical -> horizontal -> lifted -> vertical
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
			claw_turn_state = (claw_turn_state + 1) % 3;
			if (claw_turn_state == 1) {
				claw_turn_target = CLAW_TURN_HORIZONTAL_DEG;
			} else if (claw_turn_state == 2) {
				claw_turn_target = claw_turn_lift_deg;
			} else {
				claw_turn_target = CLAW_TURN_VERTICAL_DEG;
			}
			claw_turn_horizontal = (claw_turn_state != 0);  // keep shared flag in sync with auton
		}

		// Two aux pistons: D-pad UP toggles open/close together
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
			aux_open = !aux_open;
			aux_piston_a.set_value(aux_open);
			aux_piston_b.set_value(aux_open);
		}

		// Re-issue every loop so the motor's position PID keeps fighting gravity
		int velocity = claw_turn_horizontal ? CLAW_TURN_VELOCITY : CLAW_TURN_DOWN_VELOCITY;
		claw_turn.move_absolute(claw_turn_target, velocity);
		pros::delay(20);
	}
}