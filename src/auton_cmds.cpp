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
// If move(24) goes too far / not far enough, change DRIVE_SCALE (e.g. it drives 20"
// when you asked for 24 -> DRIVE_SCALE = 24 / 20 = 1.2).
constexpr float WHEEL_DIAMETER_IN = 3.25f;
constexpr float DRIVE_GEAR_RATIO = 1.0f;     // wheel turns per motor turn (1.0 = direct drive)
constexpr float DRIVE_SCALE = 1.0f;

constexpr float STRAIGHT_KP = 3.0f;          // how hard it steers back to straight (per degree off). 0 = off
constexpr int   ACCEL_STEP = 25;             // most the speed may rise per 10 ms. Bigger = punchier start
constexpr int   MOVE_MIN_SPEED = 15;         // never go slower than this until it arrives (stops it stalling short)
constexpr int   TURN_MIN_SPEED = 18;
constexpr float MOVE_TOLERANCE_IN = 0.5f;    // "arrived" when within this many inches
constexpr float TURN_TOLERANCE_DEG = 1.5f;   // "arrived" when within this many degrees

// Starting PID. The routine can change these with move_pid() / turn_pid().
constexpr float START_MOVE_KP = 8.0f;
constexpr float START_MOVE_KD = 0.4f;
constexpr float START_TURN_KP = 1.6f;
constexpr float START_TURN_KD = 0.12f;
// ---------------------------------------------------------------------------------

float move_kp = START_MOVE_KP, move_kd = START_MOVE_KD;
float turn_kp = START_TURN_KP, turn_kd = START_TURN_KD;

double start_heading = 0;   // IMU reading when the auton started
double goal_heading = 0;    // the heading the robot should be holding right now
uint32_t start_ms = 0;

bool imu_ok() { return auton_imu_ready(); }
bool stopped() { return estop_triggered(); }

void stop_drive() {
	left_motors.move(0);
	right_motors.move(0);
}

// Inches the drive has travelled (average of both sides).
double pos_in() {
	double deg = (left_motors.get_position() + right_motors.get_position()) / 2.0;
	return deg / 360.0 * M_PI * WHEEL_DIAMETER_IN * DRIVE_GEAR_RATIO * DRIVE_SCALE;
}

// Steering (in motor speed) that pulls the robot back to goal_heading.
int steer() {
	if (!imu_ok() || STRAIGHT_KP <= 0) return 0;
	double s = STRAIGHT_KP * (goal_heading - imu.get_rotation());
	return (int)std::clamp(s, -40.0, 40.0);
}

void drive_lr(int base, int turn_part) {
	left_motors.move(std::clamp(base + turn_part, -127, 127));
	right_motors.move(std::clamp(base - turn_part, -127, 127));
}

int clamp_speed(int s) { return std::clamp(s, 0, 127); }

} // namespace

// ================================ internal ========================================
void auton_cmds_reset() {
	move_kp = START_MOVE_KP;
	move_kd = START_MOVE_KD;
	turn_kp = START_TURN_KP;
	turn_kd = START_TURN_KD;
	start_ms = pros::millis();
	start_heading = imu_ok() ? imu.get_rotation() : 0;
	goal_heading = start_heading;
	left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
}

void auton_status(const char* msg) {
	int t = (int)(pros::millis() - start_ms);
	pros::screen::print(pros::E_TEXT_MEDIUM, 5, "auton %-4s %-5s %5dms %-12s",
	                    is_red() ? "RED" : "BLUE", on_right() ? "RIGHT" : "LEFT", t, msg);
	printf("DBG auton t=%d %s\n", t, msg);
}

// ================================= selection ======================================
bool on_right() { return auton_selector_get() == AutonSide::RIGHT; }
bool on_left() { return !on_right(); }
bool is_red() { return auton_color_get() == AutonColor::RED; }
bool is_blue() { return !is_red(); }
float mirror(float degrees) { return on_right() ? -degrees : degrees; }

// ================================== settings ======================================
void move_pid(float kP, float kD) { move_kp = kP; move_kd = kD; }
void turn_pid(float kP, float kD) { turn_kp = kP; turn_kd = kD; }

// ================================== waiting =======================================
void wait(int ms) {
	uint32_t end = pros::millis() + ms;
	while (pros::millis() < end && !stopped()) pros::delay(10);
}

// ================================== driving =======================================
void drive_time(int speed, int ms) {
	auton_status("drive_time");
	uint32_t end = pros::millis() + ms;
	while (pros::millis() < end && !stopped()) {
		drive_lr(speed, steer());
		pros::delay(10);
	}
	stop_drive();
}

void move(float inches, int speed, float kP, float kD, int timeout_ms) {
	auton_status("move");
	if (kP < 0) kP = move_kp;
	if (kD < 0) kD = move_kd;
	speed = clamp_speed(speed);
	if (timeout_ms <= 0) timeout_ms = 1000 + (int)(std::fabs(inches) * 60);

	double start = pos_in();
	double last_err = inches;
	double out_prev = 0;
	uint32_t t0 = pros::millis();
	int settled_ms = 0;

	while (!stopped() && (int)(pros::millis() - t0) < timeout_ms) {
		double err = inches - (pos_in() - start);
		double deriv = (err - last_err) / 0.01;   // inches per second
		last_err = err;

		double out = kP * err + kD * deriv;
		out = std::clamp(out, -(double)speed, (double)speed);
		// Don't let the speed jump up faster than ACCEL_STEP per loop.
		if (out > out_prev + ACCEL_STEP) out = out_prev + ACCEL_STEP;
		if (out < out_prev - ACCEL_STEP) out = out_prev - ACCEL_STEP;
		// Keep moving until it arrives.
		if (std::fabs(err) > MOVE_TOLERANCE_IN && std::fabs(out) < MOVE_MIN_SPEED) out = err > 0 ? MOVE_MIN_SPEED : -MOVE_MIN_SPEED;
		out_prev = out;

		drive_lr((int)out, steer());

		if (std::fabs(err) < MOVE_TOLERANCE_IN && std::fabs(deriv) < 3) settled_ms += 10; else settled_ms = 0;
		if (settled_ms >= 100) break;
		pros::delay(10);
	}
	stop_drive();
}

// ================================== turning =======================================
namespace {
void turn_to_heading(double target, int speed, float kP, float kD, int timeout_ms) {
	goal_heading = target;
	if (!imu_ok()) return;   // can't turn without the IMU
	if (kP < 0) kP = turn_kp;
	if (kD < 0) kD = turn_kd;
	speed = clamp_speed(speed);
	if (timeout_ms <= 0) timeout_ms = 1500 + (int)(std::fabs(target - imu.get_rotation()) * 10);

	double last_err = target - imu.get_rotation();
	uint32_t t0 = pros::millis();
	int settled_ms = 0;

	while (!stopped() && (int)(pros::millis() - t0) < timeout_ms) {
		double err = target - imu.get_rotation();
		double deriv = (err - last_err) / 0.01;   // degrees per second
		last_err = err;

		double out = kP * err + kD * deriv;
		out = std::clamp(out, -(double)speed, (double)speed);
		if (std::fabs(err) > TURN_TOLERANCE_DEG && std::fabs(out) < TURN_MIN_SPEED) out = err > 0 ? TURN_MIN_SPEED : -TURN_MIN_SPEED;

		// Positive = clockwise: left side forward, right side backward.
		left_motors.move((int)std::clamp(out, -127.0, 127.0));
		right_motors.move((int)std::clamp(-out, -127.0, 127.0));

		if (std::fabs(err) < TURN_TOLERANCE_DEG && std::fabs(deriv) < 10) settled_ms += 10; else settled_ms = 0;
		if (settled_ms >= 150) break;
		pros::delay(10);
	}
	stop_drive();
}
} // namespace

void turn(float degrees, int speed, float kP, float kD, int timeout_ms) {
	auton_status("turn");
	turn_to_heading(goal_heading + degrees, speed, kP, kD, timeout_ms);
}

void turn_to(float degrees_from_start, int speed, float kP, float kD, int timeout_ms) {
	auton_status("turn_to");
	turn_to_heading(start_heading + degrees_from_start, speed, kP, kD, timeout_ms);
}

// ================================== lift ==========================================
namespace { double lift_target = LIFT_DOWN_DEG; }
void lift_goal() { auton_status("lift_goal"); lift_target = LIFT_ALLIANCE_GOAL_DEG; lift_to(lift_target); }
void lift_down() { auton_status("lift_down"); lift_target = LIFT_DOWN_DEG; lift_to(lift_target); }
void lift_wait(int timeout_ms) { auton_status("lift_wait"); if (!stopped()) ::lift_wait(lift_target, timeout_ms); }

// ================================== claw ==========================================
void claw_in() { auton_status("claw_in"); claw_flip(false); claw_flip_wait(2000); }
void claw_out() { auton_status("claw_out"); claw_flip(true); claw_flip_wait(1200); }
void claw_up() { auton_status("claw_up"); if (!stopped()) claw_lift_off(); }
void claw_drop(int speed, int ms) {
	auton_status("claw_drop");
	if (speed == 1000) speed = CLAW_DROP_SPEED;
	if (ms <= 0) ms = CLAW_DROP_MS;
	claw_spin.move(speed);
	wait(ms);
	claw_spin.move(0);
}

// ============================= intake / claw spin =================================
void intake(int speed) { intake_set(speed); }
void spin_claw(int speed) { claw_spin.move(speed); }

// ================================ all-in-one ======================================
void score() {
	lift_goal();
	lift_wait();
	if (stopped()) return;
	claw_out();
	if (stopped()) return;
	claw_drop();
	if (stopped()) return;
	claw_up();
}
