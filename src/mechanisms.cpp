#include "mechanisms.h"
#include "estop.h"
#include <cmath>
#include <algorithm>

void mech_init() {
	cascade.tare_position_all();   // _all: plain tare_position() only zeroes the first motor
	cascade.set_brake_mode_all(CASCADE_BRAKE_MODE);   // _all, or only the first motor gets it
	cascade_rotation.set_position(0);   // lift boots fully lowered = 0, same as the motor tare
	// Boot position = 0 = down (the claw boots slanted; that spot is "down").
	claw_turn.tare_position();
	claw_turn_horizontal = false;
	intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	claw_spin.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void intake_set(int voltage) {
	intake_motor.move(voltage);
}

// The cascade motors slip, so their own counts drift away from the real lift height.
// The lift position comes from the cascade rotation sensor. The lift is moved by a
// simple controller that reads ONLY the sensor and sends the same power to both motors
// (aiming each motor by its own count made them fight each other).
double lift_pos() {
	// get_position() is centidegrees
	return cascade_rotation.get_position() / 100.0 / CASCADE_ROTATION_RATIO;
}

namespace {
volatile bool lift_active = false;
volatile double lift_goal_deg = 0.0;
volatile int lift_max_volt = 127;

void lift_task_fn() {
	while (true) {
		if (lift_active) {
			double err = lift_goal_deg - lift_pos();
			if (std::fabs(err) < 8.0) cascade.move(0);   // close enough: brake mode holds it
			else {
				// Plain 1 volt per degree ran out of push a few dozen degrees short (the lift
				// stalled against gravity and friction), so use a bigger gain plus a minimum
				// push: more going up (against gravity), less coming down.
				int max_v = lift_max_volt;
				int v = (int)(3.0 * err);
				int min_v = err > 0 ? 40 : 15;
				if (std::abs(v) < min_v) v = err > 0 ? min_v : -min_v;
				cascade.move(std::clamp(v, -max_v, max_v));
			}
		}
		pros::delay(10);
	}
}
} // namespace

void lift_to(double deg, int velocity) {
	static pros::Task lift_task(lift_task_fn);   // started the first time the lift is aimed
	lift_max_volt = std::clamp((int)(velocity * 127 / 200), 20, 127);   // 200 rpm = full power
	lift_goal_deg = deg;
	lift_active = true;
}

void lift_release() {
	lift_active = false;   // stop aiming; whoever calls this moves the cascade by hand
}

bool lift_wait(double deg, int timeout_ms, double tol_deg) {
	uint32_t start = pros::millis();
	while (std::fabs(lift_pos() - deg) > tol_deg) {
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
	lift_release();
	intake_motor.move(0);
	claw_spin.move(0);
	cascade.move(0);
	claw_turn.move(0);
}
