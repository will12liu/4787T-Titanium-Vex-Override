#include "auton_cmds.h"
#include "main.h"
#include "robot-config.h"
#include "mechanisms.h"
#include "auton_selector.h"
#include "auton.h"
#include "estop.h"
#include <cmath>
#include <algorithm>

// =============================================================================
// Implementation of the auton commands. You normally don't edit this file; the
// numbers below are the "advanced" settings.
// =============================================================================

namespace {

// ---- Advanced settings ---------------------------------------------------------
constexpr float STRAIGHT_KP = 3.0f;          // drive_time: how hard it steers back to straight (per degree off). 0 = off
// Auton is 15 s. Stop a little early so the routine ends cleanly (motors braked, claw off)
// instead of being cut off mid-move by the field.
constexpr int AUTON_BUDGET_MS = 14800;
// false = the routine is NOT stopped by the budget: it runs until it finishes or the field / match
// sim cuts it at 15 s. The budget is then only used for skip decisions (auton_time_left()).
constexpr bool END_AT_BUDGET = false;
// IMU turn controller (face_heading). Per degree of error; the D term is per 10 ms loop.
constexpr double TURN_KP = 2.0;
constexpr double TURN_KD = 7.0;
constexpr double TURN_MIN_POWER = 18.0;   // never push less than this while outside the tolerance (stops it stalling short)
constexpr double TURN_TOL_DEG = 1.5;      // ends when this close to the heading
// ---------------------------------------------------------------------------------

uint32_t start_ms = 0;

int time_left() { return AUTON_BUDGET_MS - (int)(pros::millis() - start_ms); }

// E-STOP or out of time: every command stops right away and later ones do nothing.
bool stopped() { return estop_triggered() || (END_AT_BUDGET && time_left() <= 0); }

// A move can never run past the end of the budget.
int cap(int timeout_ms) { return END_AT_BUDGET ? std::min(timeout_ms, std::max(time_left(), 0)) : timeout_ms; }

void stop_drive() {
	left_motors.move(0);
	right_motors.move(0);
}

int clamp_speed(int s) { return std::clamp(s, 0, 127); }

} // namespace

// ================================ internal ========================================
void auton_cmds_reset() {
	start_ms = pros::millis();
	left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	if (auton_imu_ready()) imu.set_rotation(0);   // the start is heading 0
}

int auton_time_left() { return std::max(time_left(), 0); }
bool auton_out_of_time() { return END_AT_BUDGET && time_left() <= 0; }

namespace { char last_msg[24] = ""; int last_ms = 0; }
const char* auton_last_step() { return last_msg; }
int auton_last_step_ms() { return last_ms; }

void auton_status(const char* msg) {
	int t = (int)(pros::millis() - start_ms);
	snprintf(last_msg, sizeof(last_msg), "%s", msg);
	last_ms = t;
	pros::screen::print(pros::E_TEXT_MEDIUM, 5, "auton %-4s %-5s %5dms %-12s",
	                    is_red() ? "RED" : "BLUE", on_right() ? "RIGHT" : "LEFT", t, msg);
	printf("DBG auton t=%d %s\n", t, msg);
}

// ================================= selection ======================================
bool on_right() { return auton_selector_get() == AutonSide::RIGHT; }
bool on_left() { return !on_right(); }
bool is_red() { return auton_color_get() == AutonColor::RED; }
bool is_blue() { return !is_red(); }
float mirror(float degrees) { return on_left() ? -degrees : degrees; }

// ================================== waiting =======================================
void wait(int ms) {
	uint32_t end = pros::millis() + ms;
	while (pros::millis() < end && !stopped()) pros::delay(10);
}

// ================================== driving =======================================
void drive_time(int speed, int ms) {
	auton_status("drive_time");
	bool steer = auton_imu_ready() && STRAIGHT_KP > 0;
	double hold = steer ? imu.get_rotation() : 0;   // keep facing the way it faces right now
	uint32_t end = pros::millis() + ms;
	while (pros::millis() < end && !stopped()) {
		int turn_part = 0;
		if (steer) turn_part = (int)std::clamp(STRAIGHT_KP * (hold - imu.get_rotation()), -40.0, 40.0);
		left_motors.move(std::clamp(speed + turn_part, -127, 127));
		right_motors.move(std::clamp(speed - turn_part, -127, 127));
		pros::delay(10);
	}
	stop_drive();
}

// ========================= IMU turns + encoder distances ==========================
// No odometry: distance comes from the drive motor encoders, heading from the IMU. The IMU is
// zeroed at the start of the auton (the robot starts facing heading 0), so headings are relative
// to the start, clockwise positive.
namespace {
double enc_to_in(double deg) {
	return deg / 360.0 * M_PI * drivetrain.wheelDiameter * (drivetrain.rpm / 600.0);
}
double drive_pos_in() {   // how far the robot has driven, average of both sides (+ = forward)
	return (enc_to_in(left_motors.get_position()) + enc_to_in(right_motors.get_position())) / 2.0;
}
double wrap180(double a) {
	a = std::fmod(a + 180.0, 360.0);
	if (a < 0) a += 360.0;
	return a - 180.0;
}
bool imu_ok(const char* name) {
	auton_status(name);
	if (auton_imu_ready()) return !stopped();
	auton_status("NO IMU: skipped");
	return false;
}
} // namespace

// Straight line for `inches` (+ forward, - backward): distance from the encoders, IMU holds the
// heading the robot had at the start. Eases in over the last 6".
void drive_dist(float inches, int speed) {
	if (!imu_ok(inches >= 0 ? "drive_dist" : "back_dist")) return;
	const double sign = inches >= 0 ? 1.0 : -1.0;
	const double start = drive_pos_in();
	const double hold = imu.get_rotation();
	while (!stopped()) {
		double left_in = std::fabs(inches) - std::fabs(drive_pos_in() - start);
		if (left_in <= 0.5) break;
		int v = clamp_speed(speed);
		if (left_in < 6.0) v = std::max(35, (int)(v * left_in / 6.0));
		int turn_part = (int)std::clamp(STRAIGHT_KP * (hold - imu.get_rotation()), -40.0, 40.0);
		left_motors.move(std::clamp((int)(sign * v) + turn_part, -127, 127));
		right_motors.move(std::clamp((int)(sign * v) - turn_part, -127, 127));
		pros::delay(10);
	}
	stop_drive();
}
void back_dist(float inches, int speed) { drive_dist(-std::fabs(inches), speed); }

void set_heading(float heading) {
	if (imu_ok("set_heading")) imu.set_rotation(heading);
}

// Turn in place to an IMU heading (shortest way). P + D on the heading error, with a minimum
// push so it can't stall short of the target; ends within TURN_TOL_DEG.
void face_heading(float heading, int speed) {
	if (!imu_ok("face_heading")) return;
	const uint32_t end = pros::millis() + TURN_TIMEOUT_MS;
	double last_err = wrap180(heading - imu.get_rotation());
	while (pros::millis() < end && !stopped()) {
		double err = wrap180(heading - imu.get_rotation());
		if (std::fabs(err) <= TURN_TOL_DEG) break;
		double power = TURN_KP * err + TURN_KD * (err - last_err);
		last_err = err;
		power = std::clamp(power, -(double)clamp_speed(speed), (double)clamp_speed(speed));
		if (std::fabs(power) < TURN_MIN_POWER) power = err > 0 ? TURN_MIN_POWER : -TURN_MIN_POWER;
		left_motors.move((int)power);     // + = clockwise: left forward, right back
		right_motors.move((int)-power);
		pros::delay(10);
	}
	stop_drive();
}

// ================================== lift ==========================================
namespace { double lift_target = LIFT_DOWN_DEG; }
void lift_goal() { auton_status("lift_goal"); lift_target = LIFT_ALLIANCE_GOAL_DEG; lift_to(lift_target); }
void lift_to_rot(double rot) {
	auton_status("lift_to_rot");
	printf("DBG lift_to_rot rot=%.0f pos=%.0f stopped=%d time_left=%d\n", rot, lift_pos(), (int)stopped(), time_left());
	pros::screen::print(pros::E_TEXT_MEDIUM, 4, "lift_to_rot %5.0f called, pos %5.0f %s   ", rot, lift_pos(), stopped() ? "STOPPED" : "ok");
	if (stopped()) return;
	lift_target = rot;
	lift_to(rot);
}
void lift_down() { auton_status("lift_down"); lift_target = LIFT_DOWN_DEG; lift_to(lift_target); }
void lift_wait(int timeout_ms) { auton_status("lift_wait"); if (!stopped()) ::lift_wait(lift_target, timeout_ms); }

// ================================== claw ==========================================
void claw_drop(int speed, int ms) {
	auton_status("claw_drop");
	if (speed == 1000) speed = CLAW_DROP_SPEED;
	if (ms <= 0) ms = CLAW_DROP_MS;
	claw_spin.move(speed);
	wait(ms);
	claw_spin.move(0);
}

void claw_grab(int ms) {
	auton_status("claw_grab");
	if (stopped()) return;
	claw_spin.move(CLAW_GRAB_SPEED);
	if (ms <= 0) return;
	wait(ms);
	claw_spin.move(0);
}

void claw_release(int ms) {
	auton_status("claw_release");
	if (stopped()) return;
	claw_spin.move(CLAW_RELEASE_SPEED);
	if (ms <= 0) return;
	wait(ms);
	claw_spin.move(0);
}

void claw_stop() { claw_spin.move(0); }

// ================================= claw spin ======================================
void spin_claw(int speed) { claw_spin.move(speed); }
// Moves the lift by `deg` of the lift rotation sensor (the "rot" number): + = up, - = down.
// Reads the sensor every loop, eases off in the last 60 degrees, and goes through cascade_move()
// so the travel limits still apply. Same speeds as L1 / L2 at full power.
void lift_by(double deg, int timeout_ms, bool wait_done) {
	auton_status("lift_by");
	const double up_sign = LIFT_LIMIT_TOP_DEG > LIFT_LIMIT_BOTTOM_DEG ? 1.0 : -1.0;   // which way "up" counts on the sensor
	const double target = lift_pos() + deg * up_sign;
	if (!wait_done) {                    // simultaneous: the lift task goes there in the background
		if (stopped()) return;
		lift_target = target;            // so lift_wait() waits for this height
		lift_to(target);
		return;
	}
	lift_release();                      // stop any lift_goal()/lift_down() target first
	const double TOL = 8.0, EASE = 60.0;
	uint32_t end = pros::millis() + cap(timeout_ms);
	while (pros::millis() < end && !stopped()) {
		double remaining = (target - lift_pos()) * up_sign;   // + = still has to go up
		if (std::fabs(remaining) <= TOL) break;
		int full = remaining > 0 ? 127 : -85;
		double ease = std::min(1.0, std::fabs(remaining) / EASE);
		int v = (int)(full * ease);
		int min_v = remaining > 0 ? 35 : -30;
		if (std::abs(v) < std::abs(min_v)) v = min_v;
		cascade_move(v);                 // re-issued every loop so the travel limits keep applying
		pros::delay(10);
	}
	cascade.move(0);                     // brake mode holds it (CASCADE_BRAKE_MODE)
}
