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
	int inverse = 1;
	// The drive direction ramps between +1 and -1 instead of snapping, so hitting D-pad Up
	// at full speed doesn't slam the motors from +127 to -127 in one loop (wheel spin,
	// current spike, the robot stalls for a moment).
	double inverse_scale = 1.0;
	const double INVERSE_RAMP_STEP = 0.25;   // per 20 ms loop: full flip takes ~160 ms
	master.print(2, 0, "DRIVE: FWD");
	bool intake_running = false;
	bool intake_reversed = false;
	bool claw_spin_in = false;
	bool claw_spin_out = false;
	int r1_state = 0;
	bool cascade_intake_active = false;
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
			cascade_intake_active = false;
			claw_turn_state = 0;
			claw_turn_target = CLAW_TURN_VERTICAL_DEG;
			claw_turn_horizontal = false;
			// E-stopped mid-run: leave the claw flip where it stopped instead of
			// snapping it back to vertical as soon as driver control resumes.
			if (estop_triggered()) claw_turn_target = claw_turn.get_position();
			intake_running = false;
			intake_reversed = false;
			claw_spin_in = false;
			claw_spin_out = false;
		}
		// 1. Drivetrain
		// D-pad Up toggles inverted driving: the claw side becomes the front, so stick up
		// drives claw first. Turning stays the same. Rumble: long = inverted, short = normal.
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
			inverse = -inverse;
			master.rumble(inverse < 0 ? "-" : ".");
			master.set_text(1, 0, inverse < 0 ? "INVERTED  " : "normal    ");
		}
		if (inverse_scale < inverse) {
			inverse_scale = std::min(inverse_scale + INVERSE_RAMP_STEP, double(inverse));
		} else if (inverse_scale > inverse) {
			inverse_scale = std::max(inverse_scale - INVERSE_RAMP_STEP, double(inverse));
		}
		int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		if (std::abs(dir) < 10) dir = 0;
		if (std::abs(turn) < 10) turn = 0;
		dir = (int)std::lround(dir * inverse_scale);

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
			r1_state = (r1_state + 1) % 3;
			intake_running = (r1_state == 1);
			intake_reversed = false;
			claw_spin_out = (r1_state != 0);
			claw_spin_in = false;
			cascade_intake_active = (r1_state == 1);
		}
		// R2 toggles both motors opposite their R1 directions; press again to stop.
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) && !estop_combo) {
			intake_reversed = !intake_reversed;
			intake_running = false;
			claw_spin_in = false;
			claw_spin_out = false;
		}
		intake_motor.move(intake_reversed ? -127 : (intake_running ? 127 : 0));

		// R1 moves to the intake height. Manual controls cancel that target.
		if (estop_combo) {
			cascade_intake_active = false;
			cascade.move(0);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			cascade_intake_active = false;
			cascade.move(127);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			cascade_intake_active = false;
			cascade.move(-85);
		} else if (cascade_intake_active) {
			lift_to(cascade_intake_deg, 45);
		} else {
			cascade.move(0);
		}

		// Claw spin: press A once to spin IN, press again to stop.
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			claw_spin_in = !claw_spin_in;
			claw_spin_out = false;
		}
		// Claw spin: press Y once to spin OUT, press again to stop.
		// (R1 above also starts claw spin out together with the intake.)
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
			claw_spin_out = !claw_spin_out;
			claw_spin_in = false;
		}
		if (intake_reversed) {
			claw_spin.move(127);
		} else if (claw_spin_in) {
			claw_spin.move(45);
		} else if (claw_spin_out) {
			claw_spin.move(-127);
		} else {
			claw_spin.move(0);
		}

		// Claw turn: B cycles vertical -> highest -> lifted -> horizontal -> vertical.
		// With the intake running (R1) it skips the lifted angle: highest -> horizontal.
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
			claw_turn_state = (claw_turn_state + 1) % 4;
			if (intake_running && claw_turn_state == 2) claw_turn_state = 3;
			if (claw_turn_state == 1) {
				claw_turn_target = CLAW_TURN_HIGHEST_DEG;
			} 
			else if (claw_turn_state == 2) {
				claw_turn_target = claw_turn_lift_deg;
			} 
			else if (claw_turn_state == 3) {
				claw_turn_target = CLAW_TURN_HORIZONTAL_DEG;
			}
			else {
				claw_turn_target = CLAW_TURN_VERTICAL_DEG;
			}
			claw_turn_horizontal = (claw_turn_state != 0);  // keep shared flag in sync with auton
		}

		// Re-issue every loop so the motor's position PID keeps fighting gravity
		int velocity = claw_turn_horizontal ? CLAW_TURN_VELOCITY : CLAW_TURN_DOWN_VELOCITY;
		claw_turn.move_absolute(claw_turn_target, velocity);
		pros::delay(20);
	}
}
