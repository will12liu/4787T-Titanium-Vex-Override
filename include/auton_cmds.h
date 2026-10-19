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

// ---- IMU turns + encoder distances (no odometry) ---------------------------------
// Distance is measured with the drive motor encoders and heading with the IMU, which is zeroed
// at the start of the auton (the robot starts facing heading 0). Headings are clockwise positive:
// 90 = a quarter turn right, -90 = left. These need the IMU; without it they do nothing.
// Straight drive: + inches = forward, back_dist = backward. The IMU holds the heading the robot
// has when the move starts, so turn first (face_heading), then drive.
void drive_dist(float inches, int speed = 100);
void back_dist(float inches, int speed = 100);
// Tell the robot which way it faces (IMU heading). Use it when the robot is pressed flat against a
// wall, where you know the heading exactly, to wipe out any drift built up so far.
void set_heading(float heading);
// Turn in place to a heading (shortest way). Ends within 1.5 degrees, or after TURN_TIMEOUT_MS.
void face_heading(float heading, int speed = 100);

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
