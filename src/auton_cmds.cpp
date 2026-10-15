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

// Where the claw is in the B-button cycle (same numbers as opcontrol):
// 0 = vertical, 1 = highest, 2 = angled, 3 = horizontal (90 degrees).
int claw_step = 0;

// ---- Advanced settings ---------------------------------------------------------
constexpr float STRAIGHT_KP = 3.0f;          // drive_time: how hard it steers back to straight (per degree off). 0 = off
// ---------------------------------------------------------------------------------

uint32_t start_ms = 0;

bool stopped() { return estop_triggered(); }

void stop_drive() {
	left_motors.move(0);
	right_motors.move(0);
}

int clamp_speed(int s) { return std::clamp(s, 0, 127); }

} // namespace

// ================================ internal ========================================
void auton_cmds_reset() {
	start_ms = pros::millis();
	claw_step = 0;
	left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	if (auton_odom_ready()) chassis.setPose(0, 0, 0);   // the start spot is (0, 0), facing 0
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

// ============================= LemLib (odometry) ==================================
// Each one blocks (async = false) so the routine still reads top to bottom.
// E-STOP cancels the motion, which makes the call return.
namespace {
bool odom_ok(const char* name) {
	auton_status(name);
	if (auton_odom_ready()) return !stopped();
	auton_status("NO ODOM: skipped");
	return false;
}
} // namespace

void set_pose(float x, float y, float heading) {
	if (odom_ok("set_pose")) chassis.setPose(x, y, heading);
}

void drive_to(float x, float y, int speed, int timeout_ms) {
	if (odom_ok("drive_to"))
		chassis.moveToPoint(x, y, timeout_ms, {.forwards = true, .maxSpeed = (float)clamp_speed(speed)}, false);
}

void back_to(float x, float y, int speed, int timeout_ms) {
	if (odom_ok("back_to"))
		chassis.moveToPoint(x, y, timeout_ms, {.forwards = false, .maxSpeed = (float)clamp_speed(speed)}, false);
}

void face(float x, float y, int speed, int timeout_ms) {
	if (odom_ok("face"))
		chassis.turnToPoint(x, y, timeout_ms, {.maxSpeed = clamp_speed(speed)}, false);
}

void face_heading(float heading, int speed, int timeout_ms) {
	if (odom_ok("face_heading"))
		chassis.turnToHeading(heading, timeout_ms, {.maxSpeed = clamp_speed(speed)}, false);
}

void drive_to_pose(float x, float y, float heading, int speed, int timeout_ms) {
	if (odom_ok("drive_to_pose"))
		chassis.moveToPose(x, y, heading, timeout_ms, {.forwards = true, .maxSpeed = (float)clamp_speed(speed)}, false);
}

void back_to_pose(float x, float y, float heading, int speed, int timeout_ms) {
	if (odom_ok("back_to_pose"))
		chassis.moveToPose(x, y, heading, timeout_ms, {.forwards = false, .maxSpeed = (float)clamp_speed(speed)}, false);
}

// ================================== lift ==========================================
namespace { double lift_target = LIFT_DOWN_DEG; }
void lift_goal() { auton_status("lift_goal"); lift_target = LIFT_ALLIANCE_GOAL_DEG; lift_to(lift_target); }
void lift_down() { auton_status("lift_down"); lift_target = LIFT_DOWN_DEG; lift_to(lift_target); }
void lift_wait(int timeout_ms) { auton_status("lift_wait"); if (!stopped()) ::lift_wait(lift_target, timeout_ms); }

// ================================== claw ==========================================
void claw_in() { auton_status("claw_in"); claw_step = 0; claw_flip(false); claw_flip_wait(2000); }
void claw_out() { auton_status("claw_out"); claw_step = 3; claw_flip(true); claw_flip_wait(200); }
void claw_up() { auton_status("claw_up"); claw_step = 2; if (!stopped()) claw_lift_off(); }

namespace {
// Move the claw to the given B-cycle step and wait until it is there.
void claw_go_to_step(int step) {
	claw_step = step;
	double target = step == 1 ? CLAW_TURN_HIGHEST_DEG
	              : step == 2 ? CLAW_TURN_LIFT_DEG
	              : step == 3 ? CLAW_TURN_HORIZONTAL_DEG
	              : CLAW_TURN_VERTICAL_DEG;
	claw_turn_horizontal = (step != 0);   // keep shared flag in sync with opcontrol
	claw_turn.move_absolute(target, step != 0 ? CLAW_TURN_VELOCITY : CLAW_TURN_DOWN_VELOCITY);
	uint32_t start = pros::millis();
	while (std::fabs(claw_turn.get_position() - target) > 30.0) {
		if (pros::millis() - start > 2000 || stopped()) return;
		pros::delay(10);
	}
}
} // namespace

void claw_next() {
	auton_status("claw_next");
	claw_go_to_step((claw_step + 1) % 4);
}
void claw_next_intake() {
	auton_status("claw_next_in");
	int step = (claw_step + 1) % 4;
	if (step == 2) step = 3;   // intake running: skip the angled stop
	claw_go_to_step(step);
}
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
