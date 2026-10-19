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
// A move counts as "straight along an axis" when the other axis differs by less than this (inches).
// Then that coordinate is kept where it is, so tuning error in the target can't make it curve.
constexpr float AXIS_LOCK_TOL_IN = 1.5f;
// Before a straight move, turn to the exact axis heading if the robot is more than this many degrees off.
constexpr float ALIGN_TOL_DEG = 1.5f;
// Skip the pre-turn for moves shorter than this (inches): too short to curve, not worth the time.
constexpr float ALIGN_MIN_MOVE_IN = 3.0f;
// ---------------------------------------------------------------------------------

uint32_t start_ms = 0;

int time_left() { return AUTON_BUDGET_MS - (int)(pros::millis() - start_ms); }

// E-STOP or out of time: every command stops right away and later ones do nothing.
bool stopped() { return estop_triggered() || time_left() <= 0; }

// A move can never run past the end of the budget.
int cap(int timeout_ms) { return std::min(timeout_ms, std::max(time_left(), 0)); }

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

int auton_time_left() { return std::max(time_left(), 0); }
bool auton_out_of_time() { return time_left() <= 0; }

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

// Axis lock: a move that is straight along y keeps x where it is, and a move straight along x
// keeps y where it is, so the robot drives dead straight instead of angling toward a slightly-off
// other axis. Only when the other axis is off by less than AXIS_LOCK_TOL_IN; a real diagonal
// move (bigger difference on both axes) goes to the target as written.
// Returns true if the move is straight along an axis. Then, if the robot isn't already pointing
// along that axis (within ALIGN_TOL_DEG), it turns to it first, so LemLib has no heading error
// to steer out while driving (that steering is what curves a "straight" move).
void axis_lock(float& x, float& y, bool forwards) {
	lemlib::Pose p = chassis.getPose();
	float dx = std::fabs(x - p.x), dy = std::fabs(y - p.y);
	bool along_y;
	if (dx < AXIS_LOCK_TOL_IN && dy >= dx) { x = p.x; along_y = true; }
	else if (dy < AXIS_LOCK_TOL_IN && dx > dy) { y = p.y; along_y = false; }
	else return;
	if (std::max(dx, dy) < ALIGN_MIN_MOVE_IN) return;
	// Heading of the front: 0 = +y, 90 = +x, 180 = -y, 270 = -x. Backing up faces the other way.
	float h = along_y ? (y > p.y ? 0.0f : 180.0f) : (x > p.x ? 90.0f : 270.0f);
	if (!forwards) h += 180.0f;
	float err = std::fmod(h - (float)p.theta + 540.0f, 360.0f) - 180.0f;   // -180..180
	if (std::fabs(err) > ALIGN_TOL_DEG && !stopped())
		chassis.turnToHeading(h, cap(1500), {.maxSpeed = 100}, false);
}
} // namespace

void set_pose(float x, float y, float heading) {
	if (odom_ok("set_pose")) chassis.setPose(x, y, heading);
}

void drive_to(float x, float y, int speed, int timeout_ms, float min_speed, float early_exit) {
	if (odom_ok("drive_to")) {
		axis_lock(x, y, true);
		chassis.moveToPoint(x, y, cap(timeout_ms), {.forwards = true, .maxSpeed = (float)clamp_speed(speed), .minSpeed = min_speed, .earlyExitRange = early_exit}, false);
	}
}

void back_to(float x, float y, int speed, int timeout_ms, float min_speed, float early_exit) {
	if (odom_ok("back_to")) {
		axis_lock(x, y, false);
		chassis.moveToPoint(x, y, cap(timeout_ms), {.forwards = false, .maxSpeed = (float)clamp_speed(speed), .minSpeed = min_speed, .earlyExitRange = early_exit}, false);
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
		chassis.turnToPoint(x, y, cap(timeout_ms), {.maxSpeed = clamp_speed(speed)}, false);
}

void face_heading(float heading, int speed, int timeout_ms, float min_speed, float early_exit) {
	if (odom_ok("face_heading"))
		chassis.turnToHeading(heading, cap(timeout_ms), {.maxSpeed = clamp_speed(speed), .minSpeed = (int)min_speed, .earlyExitRange = early_exit}, false);
}

void drive_to_pose(float x, float y, float heading, int speed, int timeout_ms, float min_speed, float early_exit) {
	if (odom_ok("drive_to_pose"))
		chassis.moveToPose(x, y, heading, cap(timeout_ms), {.forwards = true, .maxSpeed = (float)clamp_speed(speed), .minSpeed = min_speed, .earlyExitRange = early_exit}, false);
}

void back_to_pose(float x, float y, float heading, int speed, int timeout_ms, float min_speed, float early_exit) {
	if (odom_ok("back_to_pose"))
		chassis.moveToPose(x, y, heading, cap(timeout_ms), {.forwards = false, .maxSpeed = (float)clamp_speed(speed), .minSpeed = min_speed, .earlyExitRange = early_exit}, false);
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
