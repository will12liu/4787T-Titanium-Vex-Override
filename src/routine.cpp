#include "main.h"
#include "auton_cmds.h"
#include "auton.h"
#include "estop.h"
#include "robot-config.h"   // chassis (raw LemLib)
#include <algorithm>
#include <cmath>

// =============================================================================
// The routine. All the field numbers are in this block: change them here, not in the steps.
// Inches / degrees. RIGHT and LEFT each have their own function and their own coordinates (the
// LEFT numbers are the RIGHT ones with x and headings inverted). Edit a side's numbers in its block.
// =============================================================================
namespace {

// Where the robot's centre is when the auton starts, and which way it faces. The auton works in
// coordinates where the start is (0, 0, 0); if you place the robot differently on the start
// line, change these and everything below shifts with it.
constexpr float START_Y = 0.0f;

// 1. Toggle: back off, drive in and hit it, twice.
constexpr float TOGGLE_BACK_Y = -8.4f;       // back up to this y before each hit
constexpr int   TOGGLE_HIT_SPEED = 127;
constexpr int   TOGGLE_HIT_MS = 900;         // drive-in time: ends flat on the wall
constexpr float WALL_HEADING = 0.0f;         // heading when square to the toggle wall
constexpr float WALL_X = 0.0f, WALL_Y = 0.7f;   // robot pose when flat on the toggle wall (measured 10/13)
// The 2nd hit is skipped when less than this is left, so the pin still gets scored.
constexpr int   HIT2_MIN_TIME_LEFT_MS = 9000;   // GUESS: set from the real run times

// 2. Score the pin.
constexpr float SCORE_AWAY_Y = -16.8f;       // straight back off the toggle to this y
constexpr float SCORE_Y = -17.4f;            // the score spot's y (same on both sides)
constexpr double LIFT_BUMP_DEG = 300.0;      // PLACEHOLDER: lift up / down this many rotation-sensor degrees after dropping. Measure it

// 3. Drive away.
constexpr float EXIT_DY = -16.0f;            // y change of the final back-up
constexpr int   CLAW_OUT_SPEED = -127;       // Y button: claw spin OUT, stays on until the auton ends

constexpr int SPEED = 100;

// Chained move (raw LemLib, see score_approach): how fast and how far short of the target the
// first move hands over to the turn without stopping. 0 / 0 = stop and settle like the wrappers.
constexpr float CHAIN_MIN_SPEED = 50.0f;
constexpr float CHAIN_EXIT_IN = 4.0f;

// Everything that differs between the two sides. x in inches (+ = right), headings in degrees
// (+ = clockwise). LEFT is RIGHT with the signs flipped.
struct Side {
	float start_x;       // where the robot's centre starts
	float start_heading;
	float score_face_h;  // heading so the back (claw side) faces our goal
	float score_x;       // the score spot's x
	float pull_x;        // pull away from the goal to this x
	float exit_h;        // heading for the final straight back-up
};
constexpr Side RIGHT_SIDE = {0.0f, 0.0f, -87.1f,  21.2f,  5.4f, 316.3f};
constexpr Side LEFT_SIDE  = {0.0f, 0.0f,  87.1f, -21.2f, -5.4f,  43.7f};

// Backs straight off the wall and drives straight back in to hit the toggle. No turning and no
// point-to-point moves, so it can't curve: back_dist and drive_time both hold the heading the
// robot has at the start (square to the wall) with the IMU. `from_y` is where the robot is now.
// Pressed flat on the wall the pose is known exactly, so it is reset there to wipe out any
// error (wheel slip from the hit) built up so far.
void hit_toggle(float from_y) {
	back_dist(from_y - TOGGLE_BACK_Y, SPEED);
	drive_time(TOGGLE_HIT_SPEED, TOGGLE_HIT_MS);
	set_pose(WALL_X, WALL_Y, WALL_HEADING);
	wait(50);
}

// The combined movement, written with LemLib's chassis directly so the moves run into each other:
// back off the toggle -> turn so the claw faces our goal -> back to the score spot.
// Because it skips the wrappers it does its own safety checks: no odom / out of time / E-STOP
// means do nothing, and every timeout is cut to the time left. No axis lock or pre-turn here,
// so x and y are read from the current pose by hand to keep the straight legs straight.
void score_approach(const Side& side) {
	if (!auton_odom_ready()) return;
	auto t = [](int ms) { return std::min(ms, auton_time_left()); };
	auto go = []() { return !estop_triggered() && !auton_out_of_time(); };

	// 1) straight back along y, hands over to the turn 4" early, still moving
	if (!go()) return;
	chassis.moveToPoint(chassis.getPose().x, SCORE_AWAY_Y, t(3000),
	                    {.forwards = false, .maxSpeed = SPEED, .minSpeed = CHAIN_MIN_SPEED, .earlyExitRange = CHAIN_EXIT_IN}, false);
	// 2) turn so the BACK (claw side) faces our goal; stops fully so the next leg is square
	if (!go()) return;
	chassis.turnToHeading(side.score_face_h, t(1500), {.maxSpeed = SPEED}, false);
	// 3) straight back along x to the score spot, settles
	if (!go()) return;
	chassis.moveToPoint(side.score_x, chassis.getPose().y, t(3000), {.forwards = false, .maxSpeed = SPEED}, false);
}

void run_side(const Side& side) {
	set_pose(side.start_x, START_Y, side.start_heading);

	// 1. Hit the toggle twice
	hit_toggle(START_Y);
	if (auton_time_left() > HIT2_MIN_TIME_LEFT_MS) hit_toggle(WALL_Y);

	// 2. Score the pin
	score_approach(side);                          // back off, turn, back to the score spot (raw LemLib, chained)
	claw_drop();                                   // claw spin (A button) to drop the pin
	wait(50);
	lift_by(LIFT_BUMP_DEG);

	// 3. Drive away
	const float back_dir = (side.exit_h + 180.0f) * 3.14159265f / 180.0f;
	const float run = EXIT_DY / std::cos(back_dir);  // distance along that heading to reach the end y
	drive_to(side.pull_x, SCORE_Y, SPEED);           // 1) pull away from the goal
	lift_by(-LIFT_BUMP_DEG);
	face_heading(side.exit_h, SPEED);                // 2) turn
	back_dist(run, SPEED);                           // 3) straight down that heading, IMU-held so it can't curve
	spin_claw(CLAW_OUT_SPEED);
}

} // namespace

void routine_right() { run_side(RIGHT_SIDE); }
void routine_left() { run_side(LEFT_SIDE); }

void run_routine() {
	if (on_right()) routine_right();
	else routine_left();
}
