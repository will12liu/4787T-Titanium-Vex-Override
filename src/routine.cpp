#include "auton.h"
#include "auton_cmds.h"
#include "mechanisms.h"
#include "robot-config.h"
#include <cmath>

namespace {

// Distances are inches; headings are degrees clockwise from the starting direction.
constexpr int DRIVE_SPEED = 100;
constexpr int PAUSE_MS = 50;
constexpr int SCORE_ACTION_PAUSE_MS = 100;  // After intake and before the final release.

// Toggle: reverse to this y, drive into the wall, then correct the robot's known position.
constexpr float TOGGLE_BACK_Y = -5.4f;
constexpr float WALL_Y = 0.7f;
constexpr int TOGGLE_HIT_MS = 900;
constexpr int SECOND_HIT_MIN_TIME_MS = 9000;

// Scoring: a more negative lift rotation means a higher lift.
constexpr double SCORE_LIFT_ROT = -250.0;
constexpr float SCORE_APPROACH_Y = -11.8f;
constexpr float SCORE_Y = -14.4f;
constexpr float SCORE_EXTRA_IN = 4.0f;
constexpr float SCORE_SLOWDOWN_IN = 8.0f;  // Ease off over the last 8 inches into either goal.
constexpr int SCORE_MIN_SPEED = 25;
constexpr int SCORE_DRIVE_TIMEOUT_MS = 4000;
constexpr int SCORE_SETTLE_MS = 200;
constexpr int FIRST_RELEASE_PAUSE_MS = 250;  // Let the pin clear before pulling away.
constexpr int CLAW_RELEASE_MS = 1500;
constexpr int CLAW_RELEASE_POWER = 127;  // Full power for opening under load.
constexpr int CLAMP_DOWN_POWER = 85;
constexpr int CLAMP_PUSH_MS = 900;
constexpr int CLAMP_HOLD_POWER = 40;

// Leave the first approach move early so the turn starts while the robot is moving.
constexpr float APPROACH_MIN_SPEED = 50.0f;
constexpr float APPROACH_EARLY_EXIT_IN = 4.0f;

// Exit: complete the original reverse move, then reverse slowly while grabbing.
constexpr float EXIT_Y_CHANGE = -16.0f;
constexpr float EXIT_HEADING_ADJUST = 3.0f;
constexpr float EXIT_DISTANCE_ADJUST = -1.0f;
constexpr float EXIT_SLOW_BACK_IN = 9.75f;
constexpr int EXIT_SLOW_BACK_SPEED = 35;
constexpr int EXIT_SLOW_BACK_TIMEOUT_MS = 3000;

// After intake: raise first, turn and reverse, then lower onto the goal and release.
constexpr float SECOND_SCORE_LEFT_TURN_DEG = 133.0f;
constexpr int SECOND_SCORE_TURN_TIMEOUT_MS = 2000;
constexpr float SECOND_SCORE_BACK_IN = 23.0f;
constexpr double SECOND_SCORE_LIFT_ROT = -450.0;
constexpr double SECOND_SCORE_RELEASE_ROT = -365.0;
constexpr int SECOND_SCORE_LIFT_WAIT_MS = 2000;

struct Side {
	float score_heading;
	float score_x;
	float pull_away_x;
	float exit_heading;
};

constexpr Side RIGHT_SIDE = {
	.score_heading = -87.1f,
	.score_x = 21.2f,
	.pull_away_x = 5.4f,
	.exit_heading = 316.3f
};
constexpr Side LEFT_SIDE = {
	.score_heading = 87.1f,
	.score_x = -21.2f,
	.pull_away_x = -5.4f,
	.exit_heading = 43.7f
};

void hit_toggle(float from_y) {
	back_dist(from_y - TOGGLE_BACK_Y, DRIVE_SPEED);
	wait(PAUSE_MS);
	drive_time(DRIVE_SPEED, TOGGLE_HIT_MS);
	wait(PAUSE_MS);
	set_pose(0, WALL_Y, 0);
}

void approach_goal(const Side& side) {
	if (!auton_odom_ready() || auton_stopped()) return;

	// Reverse along y, turn the claw toward the goal, then reverse along x.
	chassis.moveToPoint(chassis.getPose().x, SCORE_APPROACH_Y, DRIVE_TIMEOUT_MS,
	                    {.forwards = false, .maxSpeed = DRIVE_SPEED,
	                     .minSpeed = APPROACH_MIN_SPEED, .earlyExitRange = APPROACH_EARLY_EXIT_IN}, false);
	if (auton_stopped()) return;
	face_heading(side.score_heading, DRIVE_SPEED);
	wait(PAUSE_MS);
	if (auton_stopped()) return;
	const float distance = std::fabs(side.score_x - chassis.getPose().x) + SCORE_EXTRA_IN;
	back_dist_score(distance, DRIVE_SPEED, SCORE_DRIVE_TIMEOUT_MS, SCORE_SLOWDOWN_IN, SCORE_MIN_SPEED);
}

bool clamp_down() {
	auton_status("clamp_down");
	lift_release();  // Give manual commands control of the lift.
	const uint32_t start = pros::millis();
	while (pros::millis() - start < CLAMP_PUSH_MS && !auton_stopped()) {
		cascade_move(-CLAMP_DOWN_POWER);  // Recheck the travel limits every 10 ms.
		pros::delay(10);
	}
	if (auton_stopped()) {
		cascade_move(0);
		return false;
	}
	cascade_move(-CLAMP_HOLD_POWER);
	return true;
}

bool wait_for_lift(double target) {
	char step[24];
	snprintf(step, sizeof(step), "lift to %.0f", target);
	auton_status(step);
	const uint32_t start = pros::millis();
	while (std::fabs(lift_pos() - target) > LIFT_RECORRECT_DEG) {
		if (auton_stopped()) return false;
		if (pros::millis() - start >= SECOND_SCORE_LIFT_WAIT_MS) {
			auton_status("lift wait timeout");
			return false;
		}
		pros::delay(10);
	}
	return !auton_stopped();
}

} // namespace

void run_routine() {
	// Keep the current field setup: blue swaps the selected left/right path.
	bool use_right = on_right();
	if (is_blue()) use_right = !use_right;
	const Side& side = use_right ? RIGHT_SIDE : LEFT_SIDE;

	set_pose(0, 0, 0);
	lift_to_rot(SCORE_LIFT_ROT);  // Raise the lift while driving.

	// 1. Hit the toggle. Repeat only if enough time remains to score.
	hit_toggle(0);
	if (auton_time_left() > SECOND_HIT_MIN_TIME_MS) {
		wait(PAUSE_MS);
		hit_toggle(WALL_Y);
	}
	wait(PAUSE_MS);

	// 2. Approach the goal, push the lift down, and release the pin.
	approach_goal(side);
	wait(SCORE_SETTLE_MS);
	if (!clamp_down()) return;
	claw_release(0, CLAW_RELEASE_POWER);  // Open before switching to R1 as the robot pulls away.
	cascade_move(0);
	wait(FIRST_RELEASE_PAUSE_MS);

	// 3. Pull away, turn, and complete the original backward drive.
	claw_grab(0);  // Reset the claw while pulling away; keep grabbing until the second release.
	// Start moving away without an automatic alignment turn at the goal.
	drive_to(side.pull_away_x, SCORE_Y, DRIVE_SPEED, DRIVE_TIMEOUT_MS, false);
	wait(PAUSE_MS);
	face_heading(side.exit_heading + EXIT_HEADING_ADJUST, DRIVE_SPEED);
	wait(PAUSE_MS);
	const float exit_radians = (side.exit_heading + 180.0f) * 3.14159265f / 180.0f;
	const float exit_distance = EXIT_Y_CHANGE / std::cos(exit_radians) + EXIT_DISTANCE_ADJUST;
	back_dist(exit_distance, DRIVE_SPEED);

	// 4. Keep R1 running during the slow intake and second scoring approach.
	back_dist(EXIT_SLOW_BACK_IN, EXIT_SLOW_BACK_SPEED, EXIT_SLOW_BACK_TIMEOUT_MS);

	// 5. Raise the pin before turning toward the second score.
	if (!auton_odom_ready() || auton_stopped()) return;
	lift_to_rot(SECOND_SCORE_LIFT_ROT);
	if (!wait_for_lift(SECOND_SCORE_LIFT_ROT)) return;
	wait(SCORE_ACTION_PAUSE_MS);
	face_heading(chassis.getPose().theta - SECOND_SCORE_LEFT_TURN_DEG,
	             DRIVE_SPEED, SECOND_SCORE_TURN_TIMEOUT_MS);
	wait(PAUSE_MS);
	if (auton_stopped()) return;

	// Reverse with the pin raised, slowing as the robot reaches the goal.
	const auto approach_result = back_dist_score(SECOND_SCORE_BACK_IN, DRIVE_SPEED, SCORE_DRIVE_TIMEOUT_MS,
	                                            SCORE_SLOWDOWN_IN, SCORE_MIN_SPEED);
	if (approach_result == ScoreApproachResult::FAILED) return;

	// At the target or stopped near it: lower onto the goal before releasing.
	lift_to_rot(SECOND_SCORE_RELEASE_ROT);
	if (!wait_for_lift(SECOND_SCORE_RELEASE_ROT)) return;
	wait(SCORE_ACTION_PAUSE_MS);
	claw_release(CLAW_RELEASE_MS, CLAW_RELEASE_POWER);
}
