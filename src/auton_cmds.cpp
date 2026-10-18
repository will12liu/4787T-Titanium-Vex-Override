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

// Axis lock: a move along y keeps x where it is, and a move along x keeps y where it is,
// so the robot drives dead straight instead of angling toward a slightly-off other axis.
// The bigger of the two distances decides which axis the move is on.
void axis_lock(float& x, float& y) {
	lemlib::Pose p = chassis.getPose();
	if (std::fabs(y - p.y) >= std::fabs(x - p.x)) x = p.x;
	else y = p.y;
}
} // namespace

void set_pose(float x, float y, float heading) {
	if (odom_ok("set_pose")) chassis.setPose(x, y, heading);
}

void drive_to(float x, float y, int speed, int timeout_ms) {
	if (odom_ok("drive_to")) {
		axis_lock(x, y);
		chassis.moveToPoint(x, y, timeout_ms, {.forwards = true, .maxSpeed = (float)clamp_speed(speed)}, false);
	}
}

void back_to(float x, float y, int speed, int timeout_ms) {
	if (odom_ok("back_to")) {
		axis_lock(x, y);
		chassis.moveToPoint(x, y, timeout_ms, {.forwards = false, .maxSpeed = (float)clamp_speed(speed)}, false);
	}
}

// Back up in a dead-straight line for `inches`, holding the heading the robot has right now
// (IMU steering, like drive_time) and measuring the distance with odometry. Face the way you
// want to go first (face_heading), then call this. No path planning, so it can't curve.
void back_dist(float inches, int speed, int timeout_ms) {
	if (!odom_ok("back_dist")) return;
	lemlib::Pose start = chassis.getPose();
	double hold = imu.get_rotation();
	uint32_t end = pros::millis() + timeout_ms;
	while (pros::millis() < end && !stopped()) {
		lemlib::Pose p = chassis.getPose();
		double done = std::hypot(p.x - start.x, p.y - start.y);
		double left_in = inches - done;
		if (left_in <= 0.5) break;
		int v = clamp_speed(speed);
		if (left_in < 6.0) v = std::max(35, (int)(v * left_in / 6.0));   // ease in over the last 6"
		int turn_part = (int)std::clamp(STRAIGHT_KP * (hold - imu.get_rotation()), -40.0, 40.0);
		left_motors.move(std::clamp(-v + turn_part, -127, 127));
		right_motors.move(std::clamp(-v - turn_part, -127, 127));
		pros::delay(10);
	}
	stop_drive();
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
void claw_drop(int speed, int ms) {
	auton_status("claw_drop");
	if (speed == 1000) speed = CLAW_DROP_SPEED;
	if (ms <= 0) ms = CLAW_DROP_MS;
	claw_spin.move(speed);
	wait(ms);
	claw_spin.move(0);
}

// ================================= claw spin ======================================
void spin_claw(int speed) { claw_spin.move(speed); }
void lift_secs(int ms, bool up)
{
	auton_status("lift_secs");
	lift_release();                      // stop any lift_goal()/lift_down() target first
	cascade.move(up ? 127 : -85);        // same speeds as L1 / L2
	wait(ms);                            // returns early on E-STOP
	cascade.move(0);                     // brake mode holds it (CASCADE_BRAKE_MODE)
}
