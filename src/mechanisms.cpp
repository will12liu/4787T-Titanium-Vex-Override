#include "mechanisms.h"
#include "estop.h"
#include <cmath>

void mech_init() {
	cascade.tare_position_all();   // _all: plain tare_position() only zeroes the first motor
	cascade.set_brake_mode_all(CASCADE_BRAKE_MODE);   // _all, or only the first motor gets it
	// Boot position = 0 = vertical. CLAW_TURN_VERTICAL_DEG is 0 to match, so nothing
	// moves when driver control starts.
	claw_turn.tare_position();
	claw_turn_horizontal = false;
	intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	claw_spin.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void intake_set(int voltage) {
	intake_motor.move(voltage);
}

void lift_to(double deg, int velocity) {
	cascade.move_absolute(deg, velocity);
}

bool lift_wait(double deg, int timeout_ms, double tol_deg) {
	uint32_t start = pros::millis();
	while (std::fabs(cascade.get_position() - deg) > tol_deg) {
		if (pros::millis() - start > (uint32_t)timeout_ms || estop_triggered()) return false;
		pros::delay(10);
	}
	return true;
}

// Motor flip. When the pneumatic flip arrives, replace the body of these two
// functions with a piston set_value() + a fixed settle delay and nothing else
// in the codebase needs to change.
void claw_flip(bool horizontal) {
	claw_turn_horizontal = horizontal;
	double target = horizontal ? CLAW_TURN_HORIZONTAL_DEG : CLAW_TURN_VERTICAL_DEG;
	int velocity = horizontal ? CLAW_TURN_VELOCITY : CLAW_TURN_DOWN_VELOCITY;
	claw_turn.move_absolute(target, velocity);
}

bool claw_flip_wait(int timeout_ms, double tol_deg) {
	double target = claw_turn_horizontal ? CLAW_TURN_HORIZONTAL_DEG : CLAW_TURN_VERTICAL_DEG;
	uint32_t start = pros::millis();
	while (std::fabs(claw_turn.get_position() - target) > tol_deg) {
		if (pros::millis() - start > (uint32_t)timeout_ms || estop_triggered()) return false;
		pros::delay(10);
	}
	return true;
}

bool claw_lift_off(int timeout_ms, double tol_deg) {
	claw_turn_horizontal = true;
	claw_turn.move_absolute(CLAW_TURN_LIFT_DEG, CLAW_TURN_VELOCITY);
	uint32_t start = pros::millis();
	while (std::fabs(claw_turn.get_position() - CLAW_TURN_LIFT_DEG) > tol_deg) {
		if (pros::millis() - start > (uint32_t)timeout_ms || estop_triggered()) return false;
		pros::delay(10);
	}
	return true;
}

void mech_stop_all() {
	intake_motor.move(0);
	claw_spin.move(0);
	cascade.move(0);
	claw_turn.move(0);
}
