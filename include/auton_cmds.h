#pragma once
#include <cmath>   // NAN

// Moves have no timeout of their own: they end when they arrive. The only thing that cuts one short
// is the 15 s auton time budget (and E-STOP). Pass a real number only if you want a limit.
constexpr int NO_TIMEOUT = 60000;
// The one exception: turns. The turn controller has no integral term, so near the target its push
// can drop below what the drivetrain needs to move, and the robot stalls a couple of degrees short
// and never reaches the exit range. A turn that never exits holds the whole auton, so turns end
// after this long at most. Set to NO_TIMEOUT to remove it.
constexpr int TURN_TIMEOUT_MS = 1000;

// =============================================================================
// AUTON COMMANDS. The building blocks you use in src/routine.cpp.
// Units: inches, degrees, speed 0-127, time in milliseconds.
// Forward / clockwise / right turn = positive. Backward / counter-clockwise = negative.
// Every command stops right away if E-STOP (L1 + R1) is pressed.
// =============================================================================

// ---- Driving -----------------------------------------------------------------
// Drive straight at `speed` for `ms` milliseconds, no distance sensing. Good for
// pushing into things. Negative speed = backwards. Uses the IMU to keep facing
// the way the robot faced when it started.
void drive_time(int speed, int ms);

// ---- LemLib (odometry): go to a spot on the field -------------------------------
// The robot always knows where it is as (x, y, heading), in inches and degrees.
// At the start of the auton that is (0, 0, 0):
//   (0, 0)    = where the robot's centre was when the auton started
//   +y        = the way the robot was facing at the start, +x = to its right
//   heading 0 = facing the start direction, positive = clockwise (90 = facing +x)
// Because every target is a fixed spot, small errors don't add up from one
// command to the next.
// Wrap x and headings in mirror() for a LEFT / RIGHT auton: drive_to(mirror(20), 30).
// These need odometry. Without it (IMU missing) they do nothing.
// timeout = give up after this many ms.

// Tell the robot where it is. Use it when the robot is pressed flat against a
// wall, where you know its exact position, to wipe out any error built up so far.
void set_pose(float x, float y, float heading);

// Drive to (x, y), turning toward it on the way. drive_to goes front first,
// back_to goes back first (claw side).
//
// CHAINING (don't stop between moves): give min_speed > 0 (e.g. 40-60) and early_exit (inches
// from the target, e.g. 4-8) and the move ends that far short of the target still moving at
// min_speed, so the next command starts without a settle. It is less exact: use it when the next
// move takes over anyway. Leave both 0 for the normal stop-and-settle. Same options on the
// pose moves and face_heading (early_exit there is in degrees).
void drive_to(float x, float y, int speed = 100, int timeout_ms = NO_TIMEOUT, float min_speed = 0, float early_exit = 0);
void back_to(float x, float y, int speed = 100, int timeout_ms = NO_TIMEOUT, float min_speed = 0, float early_exit = 0);
// back_dist: back up in a dead-straight line for `inches` along the heading the robot faces now
// (face_heading first). Holds that heading with the IMU, measures distance with odometry.
void back_dist(float inches, int speed = 100, int timeout_ms = NO_TIMEOUT, float heading = NAN);   // heading: steer to this absolute heading while driving (lets the turn before it end early); NAN = hold the current one

// Turn in place to face the point (x, y), or to face a heading.
void face(float x, float y, int speed = 100, int timeout_ms = TURN_TIMEOUT_MS);
void face_heading(float heading, int speed = 100, int timeout_ms = TURN_TIMEOUT_MS, float min_speed = 0, float early_exit = 0);

// Drive on a curve to (x, y) and arrive facing `heading`. back_to_pose drives
// back first, and `heading` is still the way the FRONT faces when it arrives.
void drive_to_pose(float x, float y, float heading, int speed = 100, int timeout_ms = NO_TIMEOUT, float min_speed = 0, float early_exit = 0);
void back_to_pose(float x, float y, float heading, int speed = 100, int timeout_ms = NO_TIMEOUT, float min_speed = 0, float early_exit = 0);

// ---- Time budget ---------------------------------------------------------------
// The auton has a 15 s budget (AUTON_BUDGET_MS in auton_cmds.cpp, a little under 15 s). When it
// runs out every command stops and later ones do nothing; move timeouts are cut to fit.
// Use auton_time_left() to skip optional steps when the run is going slowly.
int auton_time_left();          // ms left in the budget (0 when used up)
bool auton_out_of_time();

// ---- Waiting -------------------------------------------------------------------
void wait(int ms);

// ---- Lift ----------------------------------------------------------------------
void lift_goal();               // start raising the lift to alliance-goal height (does NOT wait)
void lift_down();               // start lowering the lift (does NOT wait)
void lift_wait(int timeout_ms = NO_TIMEOUT);   // wait until the lift has reached its last target

// ---- Claw ----------------------------------------------------------------------
// Spin the claw to drop the pin (the A button). speed 0-127 (negative = reverse),
// ms = how long. Leave the numbers out to use CLAW_DROP_SPEED / CLAW_DROP_MS in mechanisms.h.
void claw_drop(int speed = 1000, int ms = 0);

// Same as the driver buttons. Both stop right away on E-STOP / out of time.
// claw_grab (R1): spin in for `ms` then stop. ms <= 0 leaves it spinning until claw_stop().
void claw_grab(int ms = 700);
// claw_release (R2): spin out for `ms` then stop. ms <= 0 leaves it spinning until claw_stop().
void claw_release(int ms = 700);
void claw_stop();

// ---- Claw spin ----------------------------------------------------------------
void spin_claw(int speed);      // claw spin motor, 0 = stop, negative = reverse (stays on until you set 0)
void lift_to_rot(double rot);   // start moving the lift to this rotation-sensor number ("rot") and return at once (does NOT wait). Lower number = higher lift
void lift_by(double deg, int timeout_ms = NO_TIMEOUT, bool wait_done = true);  // move the lift by `deg` of the rotation sensor ("rot"): + up, - down. Waits until it gets there (or times out). wait_done = false: starts it and returns at once, so it runs while you drive; call lift_wait() later to be sure it arrived (timeout_ms is ignored then)

// ---- Which auton was picked (for mirroring) ------------------------------------
bool on_right();                // true if RIGHT was selected
bool on_left();                 // true if LEFT was selected
bool is_red();
bool is_blue();
// Flips the sign for a LEFT auton, so you write the RIGHT version once (the routine is
// written for RIGHT: +x = toward the right).
//   face_heading(mirror(85));   // written for RIGHT: faces right on RIGHT, left on LEFT
float mirror(float degrees);

// ---- Internal: used by autonomous.cpp ------------------------------------------
void auton_cmds_reset();                  // call once when the auton starts
void auton_status(const char* msg);
const char* auton_last_step();            // name of the last command that started
int auton_last_step_ms();                 // when it started (ms since the auton began)       // shows a message on the Brain screen
