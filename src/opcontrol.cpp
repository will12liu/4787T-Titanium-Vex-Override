#include "main.h"
#include "robot-config.h"
#include "mechanisms.h"
#include "auton.h"
#include "tuning.h"
#include <algorithm>
#include <cmath>

void opcontrol() {
	printf("DBG opcontrol start\n");
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	int drive_direction = -1;
	// Ramp direction changes to avoid suddenly reversing the motors.
	double drive_scale = 1.0;
	const double DIRECTION_RAMP_STEP = 0.25;   // per 20 ms loop: full flip takes ~160 ms
	master.print(2, 0, "DRIVE: FWD");
	cascade.set_brake_mode_all(CASCADE_BRAKE_MODE);

	while (true) {
		// D-pad Down: run a 15 s practice autonomous, then return to driving.
		if (!pros::competition::is_connected() && !tuning_active() &&
		    master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			autonomous_match_sim();
			claw_driver_reset();
		}
		// B + Y: run the routine during driver control. L1 + R1 cancels.
		if (!tuning_active() && master.get_digital(pros::E_CONTROLLER_DIGITAL_B) &&
		    master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
			auton_macro();
			claw_driver_reset();
		}
		// D-pad Up reverses the driving direction; turning stays the same.
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
			drive_direction = -drive_direction;
			master.rumble(drive_direction < 0 ? "-" : ".");
			master.set_text(1, 0, drive_direction < 0 ? "INVERTED  " : "normal    ");
		}
		if (drive_scale < drive_direction) {
			drive_scale = std::min(drive_scale + DIRECTION_RAMP_STEP, double(drive_direction));
		} else if (drive_scale > drive_direction) {
			drive_scale = std::max(drive_scale - DIRECTION_RAMP_STEP, double(drive_direction));
		}
		int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		if (std::abs(dir) < 10) dir = 0;
		if (std::abs(turn) < 10) turn = 0;
		dir = (int)std::lround(dir * drive_scale);
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

		// Ignore lift/grab input while the E-STOP combo is held.
		bool estop_combo = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
		                   master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);

		// R1 toggles the claw GRAB, R2 or A toggles the claw RELEASE (see claw_driver_update).
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) && !estop_combo) claw_grab_toggle();
		if ((master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) ||
		     master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) && !estop_combo) claw_release_toggle();

		// L1 raises, L2 lowers; the E-STOP combo blocks both.
		lift_release();
		if (!estop_combo && master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			cascade_move(127);
		} else if (!estop_combo && master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			cascade_move(-85);
		} else {
			cascade_move(0);
		}

		claw_driver_update();

		pros::delay(20);
	}
}
