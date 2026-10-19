#include "main.h"
#include "auton_cmds.h"
#include "auton.h"
#include <algorithm>
#include <cmath>

// =============================================================================
// The routine. No odometry: every move is a straight distance (drive motor encoders) or a turn to
// an IMU heading. The distances are worked out from the coordinates below, so you can still think
// in coordinates, but nothing tracks position: the robot is assumed to start at (0, 0), facing
// heading 0, and the toggle wall is behind it.
// Inches / degrees. RIGHT and LEFT each have their own coordinates (the LEFT numbers are the RIGHT
// ones with x and headings inverted). Edit a side's numbers in its block.
// =============================================================================
namespace {

constexpr float START_Y = 0.0f;              // where the robot's centre starts (x is in the Side below)

// 1. Toggle: back off, drive in and hit it, twice.
constexpr float TOGGLE_BACK_Y = -8.4f;       // back up to this y before each hit
constexpr int   TOGGLE_HIT_SPEED = 127;
constexpr int   TOGGLE_HIT_MS = 900;         // drive-in time: ends flat on the wall
constexpr float WALL_HEADING = 0.0f;         // heading when square to the toggle wall (the IMU is reset to this on the wall)
constexpr float WALL_Y = 0.7f;               // y when flat on the toggle wall (measured 10/13)
// The 2nd hit is skipped when less than this is left, so the pin still gets scored.
constexpr int   HIT2_MIN_TIME_LEFT_MS = 9000;   // GUESS: set from the real run times

// 2. Score the pin.
constexpr float SCORE_AWAY_Y = -13.8f;       // straight back off the toggle to this y
constexpr float SCORE_Y = -14.4f;            // the score spot's y (same on both sides)
constexpr double LIFT_SCORE_ROT = -140.0;    // lift rot to raise to after the toggle hits (rises while driving to the score spot)
constexpr double LIFT_CLAMP_ROT = 10.0;      // lift rot to clamp down to as the claw opens

// 3. Drive away.
constexpr float EXIT_DY = -16.0f;            // y change of the final back-up
constexpr float EXIT_TILT_DEG = -15.0f;      // tilt of the final back-up heading, RIGHT side: negative = left (counter-clockwise). LEFT uses the opposite sign
constexpr float EXIT_EXTRA_IN = -1.0f;       // distance added to the final back-up (was +2, it went ~3" too far)
constexpr int   CLAW_OUT_SPEED = -127;       // Y button: claw spin OUT, stays on until the auton ends

constexpr int SPEED = 100;
// Short pause after each turn (and the pull-away) so the robot has settled before the next move.
// Too short = misses the mark, too long = wastes the 15 s. 0 = no pause.
constexpr int PAUSE_MS = 50;
// How long to wait for the lift to reach a height before carrying on. A lift that never gets there
// (stuck, sensor off) would otherwise hold the whole auton. Set to NO_TIMEOUT to wait forever.
constexpr int LIFT_WAIT_MS = 1500;

// Everything that differs between the two sides. x in inches (+ = right), headings in degrees
// (+ = clockwise). LEFT is RIGHT with the signs flipped.
struct Side {
	float start_x;       // where the robot's centre starts
	float score_face_h;  // heading so the back (claw side) faces our goal
	float score_x;       // the score spot's x
	float pull_x;        // pull away from the goal to this x
	float exit_h;        // heading for the final straight back-up
	float exit_tilt;     // tilt added to it
};
constexpr Side RIGHT_SIDE = {0.0f, -87.1f,  21.2f,  5.4f, 316.3f, EXIT_TILT_DEG};
constexpr Side LEFT_SIDE  = {0.0f,  87.1f, -21.2f, -5.4f,  43.7f, -EXIT_TILT_DEG};

// Backs straight off the wall and drives straight back in to hit the toggle. No point-to-point
// moves, so it can't curve: back_dist and drive_time hold the heading the robot has at the start
// (square to the wall) with the IMU. `from_y` is where the robot is now. Pressed flat on the wall
// the heading is known exactly, so the IMU is reset to it to wipe out any drift built up so far.
void hit_toggle(float from_y) {
	back_dist(from_y - TOGGLE_BACK_Y, SPEED);
	drive_time(TOGGLE_HIT_SPEED, TOGGLE_HIT_MS);
	set_heading(WALL_HEADING);
}

void run_side(const Side& side) {
	// 1. Hit the toggle twice
	hit_toggle(START_Y);
	if (auton_time_left() > HIT2_MIN_TIME_LEFT_MS) hit_toggle(WALL_Y);

	// 2. Score the pin
	lift_to_rot(LIFT_SCORE_ROT);                   // lift rises while the robot drives to the score spot
	back_dist(WALL_Y - SCORE_AWAY_Y, SPEED);       // 1) straight back off the toggle
	face_heading(side.score_face_h, SPEED);        // 2) turn so the BACK (claw side) faces our goal
	wait(PAUSE_MS);
	back_dist(std::fabs(side.score_x - side.start_x), SPEED);   // 3) straight back to the score spot's x
	lift_wait(LIFT_WAIT_MS);                       // make sure it reached the score height
	lift_to_rot(LIFT_CLAMP_ROT);                   // clamp down...
	claw_drop();                                   // ...while the claw spins open (A button) to drop the pin
	lift_wait(LIFT_WAIT_MS);                       // finish the clamp before driving away

	// 3. Drive away
	const float back_dir = (side.exit_h + 180.0f) * 3.14159265f / 180.0f;
	const float run = EXIT_DY / std::cos(back_dir);  // distance along the untilted heading to reach the end y (the tilt does not change how far it goes)
	drive_dist(std::fabs(side.score_x - side.pull_x), SPEED);   // 1) pull away from the goal
	wait(PAUSE_MS);
	face_heading(side.exit_h + side.exit_tilt, SPEED);           // 2) turn
	wait(PAUSE_MS);
	back_dist(run + EXIT_EXTRA_IN, SPEED);                      // 3) straight down that heading, IMU-held so it can't curve
	spin_claw(CLAW_OUT_SPEED);
}

} // namespace

void routine_right() { run_side(RIGHT_SIDE); }
void routine_left() { run_side(LEFT_SIDE); }

// Red and blue run the same routine: LEFT goes left, RIGHT goes right.
void run_routine() {
	if (on_right()) routine_right();
	else routine_left();
}
