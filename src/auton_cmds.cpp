#include "auton_cmds.h"
#include "auton.h"
#include "auton_selector.h"
#include "estop.h"
#include "mechanisms.h"
#include "robot-config.h"
#include <algorithm>
#include <cmath>

namespace {

constexpr float STRAIGHT_KP = 3.0f;
constexpr int AUTON_BUDGET_MS = 14800;
// Keep the existing behavior: this budget skips optional steps; the field/sim cuts off at 15 s.
constexpr bool END_AT_BUDGET = false;
constexpr float AXIS_LOCK_TOL_IN = 1.5f;
constexpr float ALIGN_TOL_DEG = 1.5f;
constexpr float ALIGN_MIN_MOVE_IN = 3.0f;
constexpr double SCORE_PROGRESS_MIN_IN = 0.1;
constexpr uint32_t SCORE_STALL_MS = 250;

uint32_t start_ms = 0;
char last_step[24] = "";
int last_step_ms = 0;

void stop_drive() {
	left_motors.move(0);
	right_motors.move(0);
}

int motion_timeout(int requested_ms) {
	return END_AT_BUDGET ? std::min(requested_ms, auton_time_left()) : requested_ms;
}

bool odom_ok(const char* step) {
	auton_status(step);
	if (auton_odom_ready()) return !auton_stopped();
	auton_status("NO ODOM: skipped");
	return false;
}

// For nearly straight moves, keep the other coordinate fixed and face the travel direction.
void align_drive(float& x, float& y) {
	const lemlib::Pose pose = chassis.getPose();
	const float dx = std::fabs(x - pose.x);
	const float dy = std::fabs(y - pose.y);
	float heading;
	if (dx < AXIS_LOCK_TOL_IN && dy >= dx) {
		x = pose.x;
		heading = y > pose.y ? 0.0f : 180.0f;
	} else if (dy < AXIS_LOCK_TOL_IN && dx > dy) {
		y = pose.y;
		heading = x > pose.x ? 90.0f : 270.0f;
	} else {
		return;
	}
	if (std::max(dx, dy) < ALIGN_MIN_MOVE_IN) return;
	const float error = std::fmod(heading - pose.theta + 540.0f, 360.0f) - 180.0f;
	if (std::fabs(error) > ALIGN_TOL_DEG && !auton_stopped()) {
		chassis.turnToHeading(heading, motion_timeout(TURN_TIMEOUT_MS), {.maxSpeed = 100}, false);
	}
}

} // namespace

bool on_right() { return auton_selector_get() == AutonSide::RIGHT; }
bool is_blue() { return auton_color_get() == AutonColor::BLUE; }
int auton_time_left() { return std::max(0, AUTON_BUDGET_MS - (int)(pros::millis() - start_ms)); }
bool auton_out_of_time() { return END_AT_BUDGET && auton_time_left() == 0; }
bool auton_stopped() { return estop_triggered() || auton_out_of_time(); }
const char* auton_last_step() { return last_step; }
int auton_last_step_ms() { return last_step_ms; }

void auton_cmds_reset() {
	start_ms = pros::millis();
	left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	if (auton_odom_ready()) chassis.setPose(0, 0, 0);
}

void auton_status(const char* message) {
	last_step_ms = (int)(pros::millis() - start_ms);
	snprintf(last_step, sizeof(last_step), "%s", message);
	pros::screen::print(pros::E_TEXT_MEDIUM, 5, "auton %-4s %-5s %5dms %-12s",
	                    is_blue() ? "BLUE" : "RED", on_right() ? "RIGHT" : "LEFT", last_step_ms, message);
	printf("DBG auton t=%d %s\n", last_step_ms, message);
}

void wait(int ms) {
	const uint32_t start = pros::millis();
	while (pros::millis() - start < (uint32_t)ms && !auton_stopped()) pros::delay(10);
}

void drive_time(int speed, int ms) {
	auton_status("drive_time");
	const bool steer = auton_imu_ready() && STRAIGHT_KP > 0;
	const double heading = steer ? imu.get_rotation() : 0;
	const uint32_t start = pros::millis();
	while (pros::millis() - start < (uint32_t)ms && !auton_stopped()) {
		const int turn = steer ? (int)std::clamp(STRAIGHT_KP * (heading - imu.get_rotation()), -40.0, 40.0) : 0;
		left_motors.move(std::clamp(speed + turn, -127, 127));
		right_motors.move(std::clamp(speed - turn, -127, 127));
		pros::delay(10);
	}
	stop_drive();
}

void set_pose(float x, float y, float heading) {
	if (odom_ok("set_pose")) chassis.setPose(x, y, heading);
}

void drive_to(float x, float y, int speed, int timeout_ms, bool align_first) {
	if (!odom_ok("drive_to")) return;
	if (align_first) align_drive(x, y);
	chassis.moveToPoint(x, y, motion_timeout(timeout_ms),
	                    {.forwards = true, .maxSpeed = (float)std::clamp(speed, 0, 127)}, false);
}

// Reverse a measured distance, holding the heading at the start of the move.
void back_dist(float inches, int speed, int timeout_ms) {
	if (!odom_ok("back_dist")) return;
	const lemlib::Pose start_pose = chassis.getPose();
	const double heading = imu.get_rotation();
	const uint32_t start = pros::millis();
	while (pros::millis() - start < (uint32_t)timeout_ms && !auton_stopped()) {
		const lemlib::Pose pose = chassis.getPose();
		const double distance = std::hypot(pose.x - start_pose.x, pose.y - start_pose.y);
		const double remaining = inches - distance;
		if (remaining <= 0.5) break;
		int power = std::clamp(speed, 0, 127);
		if (remaining < 6.0) power = std::max(35, (int)(power * remaining / 6.0));
		const int turn = (int)std::clamp(STRAIGHT_KP * (heading - imu.get_rotation()), -40.0, 40.0);
		left_motors.move(std::clamp(-power + turn, -127, 127));
		right_motors.move(std::clamp(-power - turn, -127, 127));
		pros::delay(10);
	}
	stop_drive();
}

ScoreApproachResult back_dist_score(float inches, int speed, int timeout_ms, float slowdown_in, int min_speed) {
	if (!odom_ok("back_dist_score")) return ScoreApproachResult::FAILED;
	const lemlib::Pose start_pose = chassis.getPose();
	const double radians = start_pose.theta * 3.14159265 / 180.0;
	const double heading = imu.get_rotation();
	const int max_power = std::clamp(speed, 0, 127);
	const int min_power = std::clamp(min_speed, 0, max_power);
	const uint32_t start = pros::millis();
	const int duration = motion_timeout(timeout_ms);
	bool arrived = false;
	bool stalled = false;
	double last_progress_distance = 0;
	uint32_t last_progress_ms = start;
	while (pros::millis() - start < (uint32_t)duration && !auton_stopped()) {
		const lemlib::Pose pose = chassis.getPose();
		const double distance = -((pose.x - start_pose.x) * std::sin(radians)
		                        + (pose.y - start_pose.y) * std::cos(radians));
		const double remaining = inches - distance;
		if (remaining <= 0.5) {
			arrived = true;
			break;
		}
		if (distance >= last_progress_distance + SCORE_PROGRESS_MIN_IN) {
			last_progress_distance = distance;
			last_progress_ms = pros::millis();
		}
		// Contact with the goal can prevent reaching an intentionally extended target.
		if (remaining <= slowdown_in && pros::millis() - last_progress_ms >= SCORE_STALL_MS) {
			stalled = true;
			break;
		}
		int power = max_power;
		if (slowdown_in > 0 && remaining < slowdown_in) {
			power = min_power + (int)((max_power - min_power) * remaining / slowdown_in);
		}
		const int turn = (int)std::clamp(STRAIGHT_KP * (heading - imu.get_rotation()), -40.0, 40.0);
		left_motors.move(std::clamp(-power + turn, -127, 127));
		right_motors.move(std::clamp(-power - turn, -127, 127));
		pros::delay(10);
	}
	stop_drive();
	if (!arrived && !auton_stopped()) auton_status(stalled ? "score stalled" : "reverse timeout");
	if (auton_stopped()) return ScoreApproachResult::FAILED;
	if (arrived) return ScoreApproachResult::REACHED;
	return stalled ? ScoreApproachResult::STALLED_NEAR_GOAL : ScoreApproachResult::FAILED;
}

void face_heading(float heading, int speed, int timeout_ms) {
	if (odom_ok("face_heading")) {
		chassis.turnToHeading(heading, motion_timeout(timeout_ms), {.maxSpeed = std::clamp(speed, 0, 127)}, false);
	}
}

void lift_to_rot(double rotation) {
	auton_status("lift_to_rot");
	if (!auton_stopped()) lift_to(rotation);
}

void claw_grab(int ms) {
	auton_status("claw_grab");
	if (auton_stopped()) return;
	claw_grab_toggle();
	claw_driver_update();
	if (ms <= 0) return;  // Leave it running while another move executes.
	wait(ms);
	claw_driver_reset();
}

void claw_release(int ms) {
	auton_status("claw_release");
	if (auton_stopped()) return;
	claw_release_toggle();  // Exactly one R2 press, including toggling off if already releasing.
	claw_driver_update();
	if (ms <= 0) return;  // Keep opening while the routine continues.
	wait(ms);
	claw_driver_reset();
}
