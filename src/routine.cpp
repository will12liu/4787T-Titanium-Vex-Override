#include "main.h"
#include "auton_cmds.h"
#include "auton.h"
#include "estop.h"
#include "robot-config.h"   // chassis (raw LemLib)
#include "mechanisms.h"     // lift_pos, lift_release, cascade_move
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
constexpr float TOGGLE_BACK_Y = -5.4f;       // back up to this y before each hit (was -8.4, 3" less)
constexpr int   TOGGLE_HIT_SPEED = 100;
constexpr int   TOGGLE_HIT_MS = 900;         // drive-in time: ends flat on the wall
constexpr float WALL_HEADING = 0.0f;         // heading when square to the toggle wall
constexpr float WALL_X = 0.0f, WALL_Y = 0.7f;   // robot pose when flat on the toggle wall (measured 10/13)
// The 2nd hit is skipped when less than this is left, so the pin still gets scored.
constexpr int   HIT2_MIN_TIME_LEFT_MS = 9000;   // GUESS: set from the real run times

// 2. Score the pin.
constexpr float SCORE_AWAY_Y = -13.8f;       // straight back off the toggle to this y
constexpr float SCORE_Y = -14.4f;            // the score spot's y (same on both sides)
constexpr float SCORE_EXTRA_IN = 2.0f;        // drive this much further toward the goal at the score spot
constexpr int   SCORE_SETTLE_MS = 250;       // pause once there, before clamping down and opening the claw
constexpr int   CLAW_DROP_TIME_MS = 1000;    // claw open: how long R2's release spin runs (was 700)
constexpr double LIFT_SCORE_ROT = -250.0;    // lift rot to raise to after the toggle hits (rises while driving to the score spot)
constexpr double LIFT_CLAMP_ROT = 10.0;      // lift rot to clamp down to as the claw opens

// 3. Drive away.
constexpr float EXIT_DY = -16.0f;            // y change of the final back-up
constexpr float EXIT_TILT_DEG = -15.0f;      // tilt the final back-up heading: negative = left (counter-clockwise), both sides
constexpr float EXIT_EXTRA_IN = -1.0f;       // distance added to the final back-up (was +2, it went ~3" too far)
constexpr int   CLAW_OUT_SPEED = -127;       // Y button: claw spin OUT, stays on until the auton ends

constexpr int SPEED = 100;
// How long to wait for the lift to reach a height before carrying on. Every move has no timeout of
// its own, but a lift that never gets there (stuck, sensor off) would otherwise hold the whole
// auton until the budget runs out. Set to NO_TIMEOUT to wait forever.
constexpr int LIFT_WAIT_MS = 1500;
// The clamp comes down from -450 now (a much longer way than from -140), so it gets longer before the
// claw opens. It still moves on the moment the lift is within 20 of the clamp height.
constexpr int LIFT_CLAMP_WAIT_MS = 3000;
// Every drive move gets a time limit. The commands have none of their own (NO_TIMEOUT = 60 s), so a move
// that never finishes (blocked, odometry off) used to freeze the whole auton: nothing after it ran.
constexpr int MOVE_TIMEOUT_MS = 2500;
// The clamp drives the cascade straight down (not through the lift controller, which stops pushing
// as soon as it is "close enough" and never corrected a lift that stopped short).
constexpr int    CLAMP_DOWN_VOLT = 85;     // push down this hard (same as L2 in driver control)
constexpr int    CLAMP_HOLD_VOLT = 40;     // keeps pressing down while the claw opens
constexpr double CLAMP_TOL = 2.0;          // counts as clamped within this of LIFT_CLAMP_ROT (the bottom limit)
constexpr int    CLAMP_MAX_MS = 2000;      // longest it pushes down before moving on
// Short pause after each turn (and the pull-away) so the robot has settled before the next move.
// Too short = misses the mark, too long = wastes the 15 s. 0 = no pause.
constexpr int PAUSE_MS = 50;   // brief pause between moves so the robot settles

// Chained move (raw LemLib, see score_approach): how fast and how far short of the target the
// first move hands over to the turn without stopping. 0 / 0 = stop and settle like the wrappers.
constexpr float CHAIN_MIN_SPEED = 50.0f;
constexpr float CHAIN_EXIT_IN = 4.0f;
// The turn toward the score spot also hands over without stopping: it ends this many degrees short
// of the heading, still turning at this speed, and the last leg takes over. 0 / 0 = full stop.
constexpr int   TURN_CHAIN_MIN_SPEED = 0;    // 0 / 0: off (full stop, then PAUSE_MS). Was 40 / 6 and missed the mark
constexpr float TURN_CHAIN_EXIT_DEG = 0.0f;
// The exit turn can hand over early the same way. Leave at 0 / 0: back_dist holds the heading the turn
// ended on, so an early hand-over would lock in the leftover error.
constexpr int   EXIT_TURN_MIN_SPEED = 0;     // 0 / 0: off (full stop, then PAUSE_MS). Was 30 / 6
constexpr float EXIT_TURN_EXIT_DEG = 0.0f;

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
	back_dist(from_y - TOGGLE_BACK_Y, SPEED, MOVE_TIMEOUT_MS);
	wait(PAUSE_MS);
	drive_time(TOGGLE_HIT_SPEED, TOGGLE_HIT_MS);
	wait(PAUSE_MS);
	set_pose(WALL_X, WALL_Y, WALL_HEADING);
}

// The combined movement, written with LemLib's chassis directly so the moves run into each other:
// back off the toggle -> turn so the claw faces our goal -> back to the score spot.
// Because it skips the wrappers it does its own safety checks: no odom / out of time / E-STOP
// means do nothing, and every timeout is cut to the time left. No axis lock or pre-turn here,
// so x and y are read from the current pose by hand to keep the straight legs straight.
void score_approach(const Side& side) {
	if (!auton_odom_ready()) return;
	auto go = []() { return !estop_triggered() && !auton_out_of_time(); };

	// 1) straight back along y, hands over to the turn 4" early, still moving
	if (!go()) return;
	chassis.moveToPoint(chassis.getPose().x, SCORE_AWAY_Y, MOVE_TIMEOUT_MS,
	                    {.forwards = false, .maxSpeed = SPEED, .minSpeed = CHAIN_MIN_SPEED, .earlyExitRange = CHAIN_EXIT_IN}, false);
	auton_status("score turn");
	// 2) turn so the BACK (claw side) faces our goal; hands over to the last leg a few degrees early
	if (!go()) return;
	chassis.turnToHeading(side.score_face_h, TURN_TIMEOUT_MS,
	                      {.maxSpeed = SPEED, .minSpeed = TURN_CHAIN_MIN_SPEED, .earlyExitRange = TURN_CHAIN_EXIT_DEG}, false);
	// 3) straight back along x to the score spot's x. back_dist drives a measured distance while
	// holding the heading the turn ended on (no LemLib point-move, so it can't curve or exit early).
	wait(PAUSE_MS);
	if (!go()) return;
	lemlib::Pose p = chassis.getPose();
	printf("DBG score leg3: pose x=%.1f y=%.1f th=%.1f, going %.1f in\n", p.x, p.y, p.theta, std::fabs(side.score_x - p.x));
	back_dist(std::fabs(side.score_x - p.x) + SCORE_EXTRA_IN, SPEED, MOVE_TIMEOUT_MS);
}

// Pushes the cascade all the way down to the bottom limit (within CLAMP_TOL of LIFT_CLAMP_ROT) or until CLAMP_MAX_MS,
// then keeps a light downward push on. The travel limits still apply (cascade_move).
void clamp_down() {
	auton_status("clamp_down");
	lift_release();   // the lift controller must not fight this
	const uint32_t end = pros::millis() + CLAMP_MAX_MS;
	while (pros::millis() < end && !estop_triggered() && lift_pos() < LIFT_CLAMP_ROT - CLAMP_TOL) {
		cascade_move(-CLAMP_DOWN_VOLT);
		pros::delay(10);
	}
	cascade_move(-CLAMP_HOLD_VOLT);
}

void run_side(const Side& side) {
	set_pose(side.start_x, START_Y, side.start_heading);

	// Lift starts rising right away, so it is already up by the time the robot reaches the score spot
	lift_to_rot(LIFT_SCORE_ROT);

	// 1. Hit the toggle twice
	hit_toggle(START_Y);
	if (auton_time_left() > HIT2_MIN_TIME_LEFT_MS) { wait(PAUSE_MS); hit_toggle(WALL_Y); }
	wait(PAUSE_MS);

	// 2. Score the pin
	score_approach(side);                          // back off, turn, back to the score spot (raw LemLib, chained)
	lift_wait(LIFT_WAIT_MS);                       // make sure it reached the score height
	wait(SCORE_SETTLE_MS);                         // short pause at the score spot
	clamp_down();                                  // clamp down first (waits until it is down)...
	claw_release(CLAW_DROP_TIME_MS);               // ...then the claw opens the same way R2 does (release speed)
	cascade.move(0);                               // stop pressing down (brake mode holds it)

	// 3. Drive away
	const float back_dir = (side.exit_h + 180.0f) * 3.14159265f / 180.0f;
	const float run = EXIT_DY / std::cos(back_dir);  // distance along the untilted heading to reach the end y (the tilt does not change how far it goes)
	drive_to(side.pull_x, SCORE_Y, SPEED, MOVE_TIMEOUT_MS);          // 1) pull away from the goal
	wait(PAUSE_MS);
	const float exit_heading = side.exit_h + EXIT_TILT_DEG;
	face_heading(exit_heading, SPEED, TURN_TIMEOUT_MS, EXIT_TURN_MIN_SPEED, EXIT_TURN_EXIT_DEG);   // 2) turn, hands over a few degrees early
	wait(PAUSE_MS);
	back_dist(run + EXIT_EXTRA_IN, SPEED, MOVE_TIMEOUT_MS);                           // 3) straight down that heading, IMU-held so it can't curve
	spin_claw(CLAW_OUT_SPEED);
}

} // namespace

void routine_right() { run_side(RIGHT_SIDE); }
void routine_left() { run_side(LEFT_SIDE); }

// TEST: on BLUE the two sides swap (blue right runs the left numbers and blue left the right ones).
// Set to false to go back to colour not mattering.
constexpr bool BLUE_INVERTS_SIDE = true;

void run_routine() {
	bool right = on_right();
	if (BLUE_INVERTS_SIDE && is_blue()) right = !right;
	if (right) routine_right();
	else routine_left();
}
