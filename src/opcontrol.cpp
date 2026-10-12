#include "main.h"
#include "robot-config.h"
#include "mechanisms.h"   // CLAW_TURN_* constants live here now, shared with autonomous
#include "auton.h"
#include "tuning.h"
#include "estop.h"

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
	// Cascade brake mode is CASCADE_BRAKE_MODE in mechanisms.h (no active hold).
	cascade.set_brake_mode_all(CASCADE_BRAKE_MODE);   // set in mechanisms.h
	claw_turn.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);   // set here, not at boot: it made the claw twitch
	// Claw turn states: 0 = down/vertical, 1 = horizontal, 2 = lifted off the cone
	int claw_turn_state = 0;
	// The 'slightly up' position (2nd B press) is CLAW_TURN_LIFT_DEG in mechanisms.h.
	const double claw_turn_lift_deg = CLAW_TURN_LIFT_DEG;
	double claw_turn_target = CLAW_TURN_VERTICAL_DEG;

	while (true) {
		// Auton test: D-pad DOWN runs autonomous() once. Only works with no
		// competition switch / field control plugged in, so it can't fire in a match.
		if (!pros::competition::is_connected() && !tuning_active() &&
		    master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			autonomous();
			claw_turn_state = 0;
			claw_turn_target = CLAW_TURN_VERTICAL_DEG;
			claw_turn_horizontal = false;
			// E-stopped mid-run: leave the claw flip where it stopped instead of
			// snapping it back to vertical as soon as driver control resumes.
			if (estop_triggered()) claw_turn_target = claw_turn.get_position();
			intake_running = false;
			claw_spin_in = false;
			claw_spin_out = false;
			aux_open = false;
			aux_piston_a.set_value(aux_open);
			aux_piston_b.set_value(aux_open);
		}
		// 1. Drivetrain
		int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		if (std::abs(dir) < 10) dir = 0;
		if (std::abs(turn) < 10) turn = 0;

		int left = std::clamp(dir + turn, -127, 127);
		int right = std::clamp(dir - turn, -127, 127);

		// Tuning screen open: leave the drivetrain to its tests.
		if (!tuning_active()) {
			left_motors.move(left);
			right_motors.move(right);
		}

		// L1 + R1 together is the E-STOP combo: ignore both for the lift and intake
		// so stopping a test (or a tuning run) doesn't raise the lift or start the intake.
		bool estop_combo = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
		                   master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);

		// Intake: R1 toggles the intake and the claw spin (A's function) together
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) && !estop_combo) {
			intake_running = !intake_running;
			claw_spin_out = intake_running;
			claw_spin_in = false;
		}
		intake_motor.move(intake_running ? 127 : 0);

		// Cascade: L1 up, L2 down
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !estop_combo) {
			cascade.move(127);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			cascade.move(-85);
		} else {
			cascade.move(0);
		}

		// Claw spin: press A or Y once to spin IN, press again to stop.
		// (R1 above also starts claw spin out together with the intake.)
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			claw_spin_in = !claw_spin_in;
			claw_spin_out = false;
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
			claw_spin_out = !claw_spin_out;
			claw_spin_in = false;
		}
		if (claw_spin_in) {
			claw_spin.move(45);
		} else if (claw_spin_out) {
			claw_spin.move(-127);
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