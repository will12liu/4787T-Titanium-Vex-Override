#include "main.h"
#include "robot-config.h"
#include "mechanisms.h"
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
	int inverse = -1;
	// The drive direction ramps between +1 and -1 instead of snapping, so hitting D-pad Up
	// at full speed doesn't slam the motors from +127 to -127 in one loop (wheel spin,
	// current spike, the robot stalls for a moment).
	double inverse_scale = 1.0;
	const double INVERSE_RAMP_STEP = 0.25;   // per 20 ms loop: full flip takes ~160 ms
	master.print(2, 0, "DRIVE: FWD");
	int r1_state = 0;
	bool cascade_intake_active = false;
	// Cascade brake mode is CASCADE_BRAKE_MODE in mechanisms.h (no active hold).
	cascade.set_brake_mode_all(CASCADE_BRAKE_MODE);   // set in mechanisms.h

	while (true) {
		// Match sim: D-pad DOWN runs the auton like the field does (own task, hard 15 s cut-off),
		// then driver control carries on. Only works with no competition switch / field
		// control plugged in, so it can't fire in a match.
		if (!pros::competition::is_connected() && !tuning_active() &&
		    master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			autonomous_match_sim();
			cascade_intake_active = false;
			r1_state = 0;
			claw_driver_reset();
		}
		// Macro: hold B and press Y to run the routine during driver control. Works in a match
		// (driver control only; the field never lets opcontrol run during auton). L1 + R1 cancels.
		if (!tuning_active() && master.get_digital(pros::E_CONTROLLER_DIGITAL_B) &&
		    master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
			auton_macro();
			cascade_intake_active = false;
			r1_state = 0;
			claw_driver_reset();
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
		// Anti-tip: the higher the lift, the less drive power.
		double tip_scale = lift_drive_scale();
		dir = (int)std::lround(dir * tip_scale);
		turn = (int)std::lround(turn * tip_scale);

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

		// R1 toggles the claw GRAB, R2 or A toggles the claw RELEASE (see claw_driver_update).
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) && !estop_combo) claw_grab_toggle();
		if ((master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) ||
		     master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) && !estop_combo) claw_release_toggle();

		// R1 moves to the intake height. Manual controls cancel that target.
		if (estop_combo) {
			cascade_intake_active = false;
			lift_release();
			cascade.move(0);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			cascade_intake_active = false;
			lift_release();
			cascade_move(127);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			cascade_intake_active = false;
			lift_release();
			cascade_move(-85);
		} else if (cascade_intake_active) {
			lift_to(cascade_intake_deg, 200);
		} else {
			lift_release();
			cascade.move(0);
		}

		claw_driver_update();

		pros::delay(20);
	}
}
