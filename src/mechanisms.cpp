#include "mechanisms.h"
#include "estop.h"
#include <cmath>

void mech_init() {
	cascade.tare_position_all();   // _all: plain tare_position() only zeroes the first motor
	cascade.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);   // _all, or only the first motor brakes. Stays up on its own, no active hold
	claw_turn.tare_position();
	claw_turn.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	claw_turn_horizontal = false;
	intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	claw_spin.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
#if USE_CLAW_PISTON
	claw_piston.set_value(false);
#endif
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

void claw_set_open(bool open) {
#if USE_CLAW_PISTON
	claw_piston.set_value(open);
#else
	(void)open;
#endif
}

void grab_pin_start() {
	claw_set_open(true);
	intake_set(127);
}

void grab_pin_finish() {
	pros::delay(INTAKE_GRAB_SETTLE_MS);
	claw_set_open(false);
	pros::delay(CLAW_PISTON_SETTLE_MS);
	intake_set(0);
}

void score_pin(double lift_deg) {
	// Each step bails out on E-STOP (L1 + R1) without moving anything else.
	lift_to(lift_deg);
	lift_wait(lift_deg, 1500);
	if (estop_triggered()) return;

	claw_flip(true);
	claw_flip_wait(1200);
	if (estop_triggered()) return;

	claw_set_open(true);
	pros::delay(PIN_DROP_SETTLE_MS);
	if (estop_triggered()) return;

	// Flip back before lowering so the claw doesn't come down onto the goal.
	// ~940 deg at CLAW_TURN_DOWN_VELOCITY (100 rpm) takes ~1.6 s.
	claw_flip(false);
	claw_flip_wait(2000);
	if (estop_triggered()) return;
	claw_set_open(false);

	lift_to(LIFT_DOWN_DEG);
}

void mech_stop_all() {
	intake_motor.move(0);
	claw_spin.move(0);
	cascade.move(0);
	claw_turn.move(0);
}
