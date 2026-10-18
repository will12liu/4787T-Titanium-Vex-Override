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
	// Claw turn states: 0 = down/vertical, 1 = highest, 2 = small lifted angle, 3 = horizontal
	int claw_turn_state = 0;
	// The small lifted angle (2nd B press) is CLAW_TURN_LIFT_DEG in mechanisms.h.
	const double claw_turn_lift_deg = CLAW_TURN_LIFT_DEG;
	double claw_turn_target = CLAW_TURN_VERTICAL_DEG;
	bool claw_hold_here = false;      // after an E-stop: stay where it stopped until B/X/R1
	double claw_last_cmd = claw_turn_target;
	uint32_t claw_cmd_ms = pros::millis();
	bool claw_resting = false;        // down and stopped moving: quit pushing
	bool claw_intake_tilt = false;    // claw tilted for the intake; stays on until the cascade is well down
	bool claw_tilt_blocked = false;   // R1 pressed off: claw goes back to vertical even with the cascade still up

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
			claw_hold_here = estop_triggered();
			if (claw_hold_here) claw_turn_target = claw_turn.get_position();
			intake_running = false;
			intake_reversed = false;
			r1_state = 0;
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

		// R1 toggles claw spin OUT; switching directions cancels spin IN.
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) && !estop_combo) {
			claw_spin_out = !claw_spin_out;
			claw_spin_in = false;
		}
		// R2 toggles claw spin IN; switching directions cancels spin OUT.
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) && !estop_combo) {
			claw_spin_in = !claw_spin_in;
			claw_spin_out = false;
		}
		intake_motor.move(intake_reversed ? -127 : (intake_running ? 127 : 0));

		// R1 moves to the intake height. Manual controls cancel that target.
		if (estop_combo) {
			cascade_intake_active = false;
			lift_release();
			cascade.move(0);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			cascade_intake_active = false;
			lift_release();
			cascade.move(127);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			cascade_intake_active = false;
			lift_release();
			cascade.move(-85);
		} else if (cascade_intake_active) {
			lift_to(cascade_intake_deg, 200);
		} else {
			lift_release();
			cascade.move(0);
		}

		// Vision sensor sees a pin inside the claw: stop spinning in. Checked every loop so the
		// debounce keeps running; only the new-grab edge cancels, so R2 again spins with it inside.
		if (claw_pin_just_grabbed() && claw_spin_in) claw_spin_in = false;

		if (claw_spin_in) {
			claw_spin.move(60);
		} else if (claw_spin_out) {
			claw_spin.move(-127);
		} else {
			claw_spin.move(0);
		}

		// Claw turn: B = grab the cup + pin off the ground, X = score.
		// B: 1st press -> highest angle (over the cup), 2nd press -> down to the small
		//    lifted angle to grab it. Pressing again goes back to highest.
		// X: 1st press -> horizontal (90 deg) to score, 2nd press -> all the way down.
		// Each button looks at where the claw is now, so B then X works straight through.
		bool claw_b = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B);
		bool claw_x = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X);
		if (claw_b || claw_x) {
			claw_hold_here = false;
			if (claw_b) claw_turn_state = (claw_turn_state == 1) ? 2 : 1;
			else claw_turn_state = (claw_turn_state == 3) ? 0 : 3;
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

		// Claw down (state 0): cascade up (R1) -> intake tilt, otherwise the boot spot.
		// Goes by the lift height, not R1, so the 2nd R1 press doesn't lift the claw;
		// it only comes back up once the cascade is lowered.
		double claw_cmd = claw_turn_target;
		if (claw_turn_state == 0 && !claw_hold_here) {
			// Latched with a gap: once tilted, the claw stays tilted until the cascade is
			// well below the intake height. Otherwise a cascade that sags a little after
			// R1 drops under the line and the claw pops back up on its own.
			double lp = lift_pos();
			if (lp < CLAW_CLEAR_CASCADE_DEG - 120.0) claw_tilt_blocked = false;   // cascade down: tilt allowed again
			if (claw_tilt_blocked) claw_intake_tilt = false;
			else if (cascade_intake_active || lp >= CLAW_CLEAR_CASCADE_DEG) claw_intake_tilt = true;
			else if (lp < CLAW_CLEAR_CASCADE_DEG - 120.0) claw_intake_tilt = false;
			claw_cmd = claw_intake_tilt ? CLAW_TURN_INTAKE_DEG : CLAW_TURN_VERTICAL_DEG;
		}
		if (claw_cmd != claw_last_cmd) {
			claw_last_cmd = claw_cmd;
			claw_cmd_ms = pros::millis();
			claw_resting = false;
		}
		// Going down, the claw can land on a hard stop before its target and the motor
		// keeps pushing into it (the sharp whine). Once it has stopped moving, just
		// hold where it is.
		if (claw_turn_state == 0 && !claw_resting && pros::millis() - claw_cmd_ms > 300 &&
		    std::fabs(claw_turn.get_actual_velocity()) < 5.0) {
			claw_resting = true;
		}

		// Re-issue every loop so the motor's position PID keeps fighting gravity
		int velocity = (claw_turn_horizontal || (claw_turn_state == 0 && claw_cmd == CLAW_TURN_INTAKE_DEG))
			? CLAW_TURN_VELOCITY : CLAW_TURN_DOWN_VELOCITY;   // intake tilt is full speed
		if (claw_resting) claw_turn.brake();   // HOLD brake: holds here, no pushing
		else claw_turn.move_absolute(claw_cmd, velocity);
		pros::delay(20);
	}
}
