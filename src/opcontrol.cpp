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
	bool claw_open = false;
	bool intake_running = false;
	bool claw_spin_in = false;
	bool claw_spin_out = false;	
	// claw_turn is tared once at boot in mech_init(); re-taring here would
	// shift the flip targets after autonomous.
	// Cascade stays up on its own now, so no active hold: BRAKE just shorts the
	// motor when stopped (no power), instead of HOLD fighting gravity.
	cascade.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	double claw_turn_target = CLAW_TURN_VERTICAL_DEG; 

	while (true) {
		// Auton test: D-pad DOWN runs autonomous() once. Only works with no
		// competition switch / field control plugged in, so it can't fire in a match.
		if (!pros::competition::is_connected() && !tuning_active() &&
		    master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			autonomous();
			claw_open = false;           // auton ends with the claw closed
			claw_turn_target = CLAW_TURN_VERTICAL_DEG;
			// E-stopped mid-run: leave the claw flip where it stopped instead of
			// snapping it back to vertical as soon as driver control resumes.
			if (estop_triggered()) claw_turn_target = claw_turn.get_position();
			intake_running = false;
			claw_spin_in = false;
			claw_spin_out = false;
		}
		// Drive-only test: R2 runs the selected auton's driving with no lift or claw.
		if (!pros::competition::is_connected() && !tuning_active() &&
		    master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
			auton_drive_test();
		}

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

			// Tuning screen open: leave the drivetrain to its tests.
			if (!tuning_active()) {
				left_motors.move(left);
				right_motors.move(right);
			}

		// L1 + R1 together is the E-STOP combo: ignore both for the lift and intake
		// so stopping a test (or a tuning run) doesn't raise the lift or start the intake.
		bool estop_combo = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
		                   master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);

		// Intake: R1 in, R2 out
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) && !estop_combo) {
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
		
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !estop_combo) {
			cascade.move(127);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			cascade.move(-85);
		} else {
			cascade.move(0);
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

		bool x_new = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X);
		bool b_new = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B);
		bool x_held = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
		bool b_held = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);

		// Combo: X + B together -> raise claw to horizontal AND open claw at once
		bool combo = (x_new && b_held) || (b_new && x_held);

		if (combo) {
			claw_turn_horizontal = true;
			claw_turn_target = CLAW_TURN_HORIZONTAL_DEG;
			claw_open = true;
		#if USE_CLAW_PISTON
			claw_piston.set_value(claw_open);
		#endif
		} else {
			if (x_new) {
				claw_open = !claw_open;
		#if USE_CLAW_PISTON
				claw_piston.set_value(claw_open);
		#endif
			}
			if (b_new) {
				claw_turn_horizontal = !claw_turn_horizontal;
				claw_turn_target = claw_turn_horizontal ? CLAW_TURN_HORIZONTAL_DEG : CLAW_TURN_VERTICAL_DEG;

				// Going back down: also close the claw
				if (!claw_turn_horizontal) {
					claw_open = false;
			#if USE_CLAW_PISTON
					claw_piston.set_value(claw_open);
			#endif
				}
				}
		}

			

			// Re-issue every loop so the motor's position PID keeps fighting gravity
			int velocity = claw_turn_horizontal ? CLAW_TURN_VELOCITY : CLAW_TURN_DOWN_VELOCITY;
			claw_turn.move_absolute(claw_turn_target, velocity);
			pros::delay(20);
		}
       


		
	}
