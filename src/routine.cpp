#include "main.h"
#include "auton_cmds.h"
#include "auton.h"
#include "estop.h"
#include "robot-config.h"   // chassis (raw LemLib)
#include <algorithm>
#include <cmath>

// =============================================================================
// The routine. All the field numbers are in this block: change them here, not in the steps.
// Inches / degrees, written for the LEFT side (RIGHT is mirrored by mirror()).
// =============================================================================
namespace {

// Where the robot's centre is when the auton starts, and which way it faces. The auton works in
// coordinates where the start is (0, 0, 0); if you place the robot differently on the start
// line, change these and everything below shifts with it.
constexpr float START_X = 0.0f, START_Y = 0.0f, START_HEADING = 0.0f;

// 1. Toggle: back off, drive in and hit it, twice.
constexpr float TOGGLE_BACK_Y = -8.4f;       // back up to this y before each hit
constexpr int   TOGGLE_HIT_SPEED = 127;
constexpr int   TOGGLE_HIT_MS = 900;         // drive-in time: ends flat on the wall
constexpr float WALL_X = 0.0f, WALL_Y = 0.7f;   // robot pose when flat on the toggle wall (measured 10/13)
// The 2nd hit is skipped when less than this is left, so the pin still gets scored.
constexpr int   HIT2_MIN_TIME_LEFT_MS = 9000;   // GUESS: set from the real run times

// 2. Score the pin.
constexpr float SCORE_AWAY_Y = -16.8f;       // straight back off the toggle to this y
constexpr float SCORE_FACE_H = -87.1f;       // heading so the back (claw side) faces our goal
constexpr float SCORE_X = 21.2f, SCORE_Y = -17.4f;   // the score spot
constexpr double LIFT_BUMP_DEG = 300.0;      // PLACEHOLDER: lift up / down this many rotation-sensor degrees after dropping. Measure it

// 3. Drive away.
constexpr float PULL_X = 5.4f;               // pull away from the goal to this x
constexpr float EXIT_H = 316.3f;             // heading for the final straight back-up
constexpr float EXIT_DY = -16.0f;            // y change of the final back-up
constexpr int   CLAW_OUT_SPEED = -127;       // Y button: claw spin OUT, stays on until the auton ends

constexpr int SPEED = 100;

// Chained move (raw LemLib, see score_approach): how fast and how far short of the target the
// first move hands over to the turn without stopping. 0 / 0 = stop and settle like the wrappers.
constexpr float CHAIN_MIN_SPEED = 50.0f;
constexpr float CHAIN_EXIT_IN = 4.0f;

// Backs into the toggle wall and hits it. Pressed flat on the wall the pose is known exactly,
// so it is reset there to wipe out any error (wheel slip from the hit) built up so far.
void hit_toggle() {
	back_to(WALL_X, TOGGLE_BACK_Y, SPEED);
	drive_time(TOGGLE_HIT_SPEED, TOGGLE_HIT_MS);
	set_pose(WALL_X, WALL_Y, 0);
	wait(50);
}

void score_approach() {
	if (!auton_odom_ready()) return;
	auto t = [](int ms) { return std::min(ms, auton_time_left()); };
	auto go = []() { return !estop_triggered() && !auton_out_of_time(); };

	// 1) straight back along y, hands over to the turn 4" early, still moving
	if (!go()) return;
	chassis.moveToPoint(chassis.getPose().x, SCORE_AWAY_Y, t(3000),
	                    {.forwards = false, .maxSpeed = SPEED, .minSpeed = CHAIN_MIN_SPEED, .earlyExitRange = CHAIN_EXIT_IN}, false);
	// 2) turn so the BACK (claw side) faces our goal; stops fully so the next leg is square
	if (!go()) return;
	chassis.turnToHeading(mirror(SCORE_FACE_H), t(1500), {.maxSpeed = SPEED}, false);
	// 3) straight back along x to the score spot, settles
	if (!go()) return;
	chassis.moveToPoint(mirror(SCORE_X), chassis.getPose().y, t(3000), {.forwards = false, .maxSpeed = SPEED}, false);
}

} // namespace

void run_routine() {
	set_pose(mirror(START_X), START_Y, mirror(START_HEADING));

	// 1. Hit the toggle twice
	hit_toggle();
	if (auton_time_left() > HIT2_MIN_TIME_LEFT_MS) hit_toggle();

	// 2. Score the pin
	score_approach();                              // back off, turn, back to the score spot (raw LemLib, chained)
	claw_drop();                                   // claw spin (A button) to drop the pin
	wait(50);
	lift_by(LIFT_BUMP_DEG);

	// 3. Drive away
	const float back_dir = (EXIT_H + 180.0f) * 3.14159265f / 180.0f;
	const float run = EXIT_DY / std::cos(back_dir);  // distance along that heading to reach the end y
	drive_to(mirror(PULL_X), SCORE_Y, SPEED);        // 1) pull away from the goal
	lift_by(-LIFT_BUMP_DEG);
	face_heading(mirror(EXIT_H), SPEED);             // 2) turn
	back_dist(run, SPEED);                           // 3) straight down that heading, IMU-held so it can't curve
	spin_claw(CLAW_OUT_SPEED);
}
