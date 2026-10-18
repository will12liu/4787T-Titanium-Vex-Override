#include "mechanisms.h"
#include "estop.h"
#include <cmath>
#include <algorithm>

void mech_init() {
	cascade.tare_position_all();   // _all: plain tare_position() only zeroes the first motor
	lift_rot.reset_position();     // lift sensor zero = lift down at boot
	cascade.set_brake_mode_all(CASCADE_BRAKE_MODE);   // _all, or only the first motor gets it
	// Boot position = 0 = down (the claw boots slanted; that spot is "down").
	claw_turn.tare_position();
	claw_turn_horizontal = false;
	intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	claw_spin.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

void claw_vision_init() {
	// PLACEHOLDER numbers: replace with the output of the Vision Utility for the pin.
	vision_signature_s_t sig = pros::Vision::signature_from_utility(
		CLAW_VISION_SIG_ID, 8973, 11143, 10058, -2119, -1053, -1586, 5.4, 0);
	claw_vision.set_signature(CLAW_VISION_SIG_ID, &sig);
	claw_vision.set_zero_point(pros::E_VISION_ZERO_TOPLEFT);
}

bool claw_vision_sees_pin() {
	vision_object_s_t obj = claw_vision.get_by_size(0);   // biggest object
	if (obj.signature == VISION_OBJECT_ERR_SIG) return false;   // nothing seen, or sensor unplugged
	return obj.width >= CLAW_VISION_MIN_WIDTH && obj.height >= CLAW_VISION_MIN_HEIGHT;
}

bool claw_pin_just_grabbed() {
	static uint32_t seen_since = 0;
	static bool was_in = false;
	if (!claw_vision_sees_pin()) {
		seen_since = 0;
		was_in = false;
		return false;
	}
	if (seen_since == 0) seen_since = pros::millis();
	bool in = pros::millis() - seen_since >= (uint32_t)CLAW_VISION_CONFIRM_MS;
	bool rising = in && !was_in;
	was_in = in;
	return rising;
}

bool claw_spin_until_pin(int speed, int timeout_ms) {
	uint32_t start = pros::millis();
	claw_spin.move(speed);
	bool got = false;
	while (pros::millis() - start < (uint32_t)timeout_ms && !estop_triggered()) {
		if (claw_pin_just_grabbed()) { got = true; break; }
		pros::delay(10);
	}
	claw_spin.move(0);
	return got;
}

void intake_set(int voltage) {
	intake_motor.move(voltage);
}

// Lift height from the rotation sensor on the middle axis, so motor slip can't throw it off.
// If the sensor is unplugged, fall back to the port 19 motor count (the port 20 motor drifts).
double lift_pos() {
	int32_t c = lift_rot.get_position();   // centidegrees
	if (c == PROS_ERR) return cascade.get_position(1);   // index 1 = port 19 in {-20, 19}
	return c / 100.0 * LIFT_ROT_TO_LIFT;
}

bool lift_at(double deg, double tol_deg) {
	return std::fabs(lift_pos() - deg) <= tol_deg;
}

namespace {
volatile bool lift_active = false;
volatile double lift_goal_deg = 0.0;
volatile int lift_max_volt = 127;

void lift_task_fn() {
	bool holding = false;   // inside the dead band: motors stopped, brake mode holds
	while (true) {
		if (lift_active) {
			double err = lift_goal_deg - lift_pos();   // + = too low, - = too high (over)
			double a = std::fabs(err);
			// Hysteresis: stop inside the hold band, but only start correcting again once it has
			// slipped past the re-correct band, so a small sag doesn't make it buzz.
			if (a < LIFT_HOLD_BAND_DEG) holding = true;
			else if (a > LIFT_RECORRECT_DEG) holding = false;
			if (holding) cascade.move(0);
			else {
				int max_v = lift_max_volt;
				int v = (int)(3.0 * err);
				if (err > 0) {   // under: push up against gravity
					v = std::clamp(std::max(v, LIFT_UNDER_MIN_VOLT), LIFT_UNDER_MIN_VOLT, max_v);
				} else {         // over: come back down gently
					v = std::clamp(std::min(v, -LIFT_OVER_MIN_VOLT), -std::min(max_v, LIFT_OVER_MAX_VOLT), -LIFT_OVER_MIN_VOLT);
				}
				cascade.move(v);
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
