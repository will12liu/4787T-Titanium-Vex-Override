#include "mechanisms.h"
#include <cmath>
#include <algorithm>
#include <atomic>

void mech_init() {
	cascade.tare_position_all();   // Apply setup to every lift motor.
	lift_rot.reset_position();     // lift sensor zero = lift down at boot
	cascade.set_brake_mode_all(CASCADE_BRAKE_MODE);
	claw_spin.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

namespace {
enum class ClawMode { STOPPED, GRAB, RELEASE };
std::atomic<ClawMode> claw_mode{ClawMode::STOPPED};
std::atomic<bool> claw_auton_active{false};
pros::Mutex claw_auton_mutex;
}

void claw_grab_toggle() {
	claw_mode = claw_mode == ClawMode::GRAB ? ClawMode::STOPPED : ClawMode::GRAB;
}

void claw_release_toggle() {
	claw_mode = claw_mode == ClawMode::RELEASE ? ClawMode::STOPPED : ClawMode::RELEASE;
}

void claw_driver_update() {
	if (claw_mode == ClawMode::RELEASE) claw_spin.move(CLAW_RELEASE_SPEED);
	else if (claw_mode == ClawMode::GRAB) claw_spin.move(CLAW_GRAB_SPEED);
	else claw_spin.move(0);
}

void claw_driver_reset() {
	claw_mode = ClawMode::STOPPED;
	claw_spin.move(0);
}

void claw_auton_start() {
	claw_auton_mutex.take();
	claw_driver_reset();
	claw_auton_active = true;
	claw_auton_mutex.give();
	static pros::Task claw_task([]() {
		while (true) {
			claw_auton_mutex.take();
			if (claw_auton_active) claw_driver_update();
			claw_auton_mutex.give();
			pros::delay(20);
		}
	});
}

// Lift height from the rotation sensor on the middle axis, so motor slip can't throw it off.
// If unplugged, use the port 19 motor encoder (index 1).
double lift_pos() {
	int32_t c = lift_rot.get_position();   // centidegrees
	if (c == PROS_ERR) return cascade.get_position(1);
	return c / 100.0 * LIFT_ROT_TO_LIFT;
}

void cascade_move(int power) {
	if (power == 0 || lift_rot.get_position() == PROS_ERR) {
		cascade.move(power);
		return;
	}
	const double position = lift_pos();
	const double up_sign = LIFT_LIMIT_TOP_DEG > LIFT_LIMIT_BOTTOM_DEG ? 1.0 : -1.0;
	const double room = power > 0 ? (LIFT_LIMIT_TOP_DEG - position) * up_sign
	                              : (position - LIFT_LIMIT_BOTTOM_DEG) * up_sign;
	if (room <= 0) {
		cascade.move(0);   // at or past the end: brake mode holds it there
		return;
	}
	if (room < LIFT_LIMIT_SLOW_DEG) {   // ease off so it doesn't coast past the end
		int cap = std::max(LIFT_LIMIT_MIN_POWER, (int)(127.0 * room / LIFT_LIMIT_SLOW_DEG));
		power = std::clamp(power, -cap, cap);
	}
	cascade.move(power);
}

double lift_drive_scale() {
	if (lift_rot.get_position() == PROS_ERR) return 1.0;
	double height_fraction = (lift_pos() - LIFT_LIMIT_BOTTOM_DEG) / (LIFT_LIMIT_TOP_DEG - LIFT_LIMIT_BOTTOM_DEG);
	if (height_fraction <= LIFT_SLOW_START_FRAC) return 1.0;
	height_fraction = std::min(height_fraction, 1.0);
	const double reduction = (height_fraction - LIFT_SLOW_START_FRAC) / (1.0 - LIFT_SLOW_START_FRAC);
	return 1.0 - reduction * (1.0 - LIFT_SLOW_MIN_SCALE);
}

namespace {
volatile bool lift_active = false;
volatile double lift_goal_deg = 0.0;

void lift_task_fn() {
	bool holding = false;   // inside the dead band: motors stopped, brake mode holds
	while (true) {
		if (lift_active) {
			// Positive error means the lift needs to rise.
			const double up_sign = LIFT_LIMIT_TOP_DEG > LIFT_LIMIT_BOTTOM_DEG ? 1.0 : -1.0;
			const double error = (lift_goal_deg - lift_pos()) * up_sign;
			const double distance = std::fabs(error);
			// Separate stop/restart thresholds prevent buzzing near the target.
			if (distance < LIFT_HOLD_BAND_DEG) holding = true;
			else if (distance > LIFT_RECORRECT_DEG) holding = false;
			if (holding) cascade_move(0);
			else {
				int power = (int)(3.0 * error);
				if (error > 0) {   // under: push up against gravity
					power = std::clamp(power, LIFT_UP_MIN_POWER, 127);
				} else {         // over: come back down gently
					power = std::clamp(power, -LIFT_DOWN_MAX_POWER, -LIFT_DOWN_MIN_POWER);
				}
				cascade_move(power);
			}
			static uint32_t last_dbg = 0;   // 4x a second: on the Brain screen (row 4) and the terminal
			if (pros::millis() - last_dbg > 250) {
				last_dbg = pros::millis();
				pros::screen::print(pros::E_TEXT_MEDIUM, 4, "lift goal %5.0f pos %5.0f err %5.0f %s   ", lift_goal_deg, lift_pos(), error, holding ? "HOLD" : "go");
				printf("DBG lift goal=%.0f pos=%.0f err=%.0f holding=%d\n", lift_goal_deg, lift_pos(), error, (int)holding);
			}
		}
		pros::delay(10);
	}
}
} // namespace

void lift_task_start() {
	static pros::Task lift_task(lift_task_fn);   // starts once; later calls do nothing
}

void lift_to(double rotation) {
	lift_task_start();   // normally already running from initialize()
	lift_goal_deg = rotation;
	lift_active = true;
}

void lift_release() {
	lift_active = false;   // stop aiming; whoever calls this moves the cascade by hand
}

void mech_stop_all() {
	lift_release();
	claw_auton_mutex.take();
	claw_auton_active = false;
	claw_driver_reset();
	claw_auton_mutex.give();
	cascade.move(0);
}
