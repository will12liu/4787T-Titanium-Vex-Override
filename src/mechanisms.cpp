#include "mechanisms.h"
#include "estop.h"
#include <cmath>
#include <algorithm>

void mech_init() {
	cascade.tare_position_all();   // _all: plain tare_position() only zeroes the first motor
	lift_rot.reset_position();     // lift sensor zero = lift down at boot
	cascade.set_brake_mode_all(CASCADE_BRAKE_MODE);   // _all, or only the first motor gets it
	claw_spin.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

namespace {
bool claw_grabbing = false;
bool claw_releasing = false;
}

void claw_grab_toggle() {
	claw_grabbing = !claw_grabbing;
	claw_releasing = false;
}

void claw_release_toggle() {
	claw_releasing = !claw_releasing;
	claw_grabbing = false;
}

void claw_driver_update() {
	if (claw_releasing) claw_spin.move(CLAW_RELEASE_SPEED);
	else if (claw_grabbing) claw_spin.move(CLAW_GRAB_SPEED);
	else claw_spin.move(0);
}

void claw_driver_reset() {
	claw_grabbing = false;
	claw_releasing = false;
	claw_spin.move(0);
}

// Lift height from the rotation sensor on the middle axis, so motor slip can't throw it off.
// If the sensor is unplugged, fall back to the port 19 motor count (the port 20 motor drifts).
double lift_pos() {
	int32_t c = lift_rot.get_position();   // centidegrees
	if (c == PROS_ERR) return cascade.get_position(1);   // index 1 = port 19 in {-20, 19}
	return c / 100.0 * LIFT_ROT_TO_LIFT;
}

void cascade_move(int voltage) {
	if (voltage == 0 || lift_rot.get_position() == PROS_ERR) {
		cascade.move(voltage);
		return;
	}
	double pos = lift_pos();
	double s = LIFT_LIMIT_TOP_DEG > LIFT_LIMIT_BOTTOM_DEG ? 1.0 : -1.0;   // direction of "up" in lift_pos()
	double room = voltage > 0 ? (LIFT_LIMIT_TOP_DEG - pos) * s : (pos - LIFT_LIMIT_BOTTOM_DEG) * s;
	if (room <= 0) {
		cascade.move(0);   // at or past the end: brake mode holds it there
		return;
	}
	if (room < LIFT_LIMIT_SLOW_DEG) {   // ease off so it doesn't coast past the end
		int cap = std::max(LIFT_LIMIT_MIN_VOLT, (int)(127.0 * room / LIFT_LIMIT_SLOW_DEG));
		voltage = std::clamp(voltage, -cap, cap);
	}
	cascade.move(voltage);
}

double lift_drive_scale() {
	if (lift_rot.get_position() == PROS_ERR) return 1.0;
	double frac = (lift_pos() - LIFT_LIMIT_BOTTOM_DEG) / (LIFT_LIMIT_TOP_DEG - LIFT_LIMIT_BOTTOM_DEG);
	if (frac <= LIFT_SLOW_START_FRAC) return 1.0;
	frac = std::min(frac, 1.0);
	double t = (frac - LIFT_SLOW_START_FRAC) / (1.0 - LIFT_SLOW_START_FRAC);
	return 1.0 - t * (1.0 - LIFT_SLOW_MIN_SCALE);
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
			// Sensor numbers go DOWN as the lift goes up (or up, if the limits are the other way round),
			// so flip the sign by the direction of "up": + = too low, - = too high (over).
			double up = LIFT_LIMIT_TOP_DEG > LIFT_LIMIT_BOTTOM_DEG ? 1.0 : -1.0;
			double err = (lift_goal_deg - lift_pos()) * up;
			double a = std::fabs(err);
			// Hysteresis: stop inside the hold band, but only start correcting again once it has
			// slipped past the re-correct band, so a small sag doesn't make it buzz.
			if (a < LIFT_HOLD_BAND_DEG) holding = true;
			else if (a > LIFT_RECORRECT_DEG) holding = false;
			if (holding) cascade_move(0);
			else {
				int max_v = lift_max_volt;
				int v = (int)(3.0 * err);
				if (err > 0) {   // under: push up against gravity
					v = std::clamp(std::max(v, LIFT_UNDER_MIN_VOLT), LIFT_UNDER_MIN_VOLT, max_v);
				} else {         // over: come back down gently
					v = std::clamp(std::min(v, -LIFT_OVER_MIN_VOLT), -std::min(max_v, LIFT_OVER_MAX_VOLT), -LIFT_OVER_MIN_VOLT);
				}
				cascade_move(v);
			}
			static uint32_t last_dbg = 0;   // 4x a second: on the Brain screen (row 4) and the terminal
			if (pros::millis() - last_dbg > 250) {
				last_dbg = pros::millis();
				pros::screen::print(pros::E_TEXT_MEDIUM, 4, "lift goal %5.0f pos %5.0f err %5.0f %s   ", lift_goal_deg, lift_pos(), err, holding ? "HOLD" : "go");
				printf("DBG lift goal=%.0f pos=%.0f err=%.0f holding=%d\n", lift_goal_deg, lift_pos(), err, (int)holding);
			}
		}
		pros::delay(10);
	}
}
} // namespace

void lift_task_start() {
	static pros::Task lift_task(lift_task_fn);   // starts once; later calls do nothing
}

void lift_to(double deg, int velocity) {
	lift_task_start();   // normally already running from initialize()
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

void mech_stop_all() {
	lift_release();
	claw_spin.move(0);
	cascade.move(0);
}
