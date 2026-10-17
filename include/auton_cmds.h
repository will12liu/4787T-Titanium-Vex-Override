#pragma once

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
// Wrap x and headings in mirror() for a LEFT / RIGHT auton: drive_to(mirror(-20), 30).
// These need odometry. Without it (IMU missing) they do nothing.
// timeout = give up after this many ms.

// Tell the robot where it is. Use it when the robot is pressed flat against a
// wall, where you know its exact position, to wipe out any error built up so far.
void set_pose(float x, float y, float heading);

// Drive to (x, y), turning toward it on the way. drive_to goes front first,
// back_to goes back first (claw side).
void drive_to(float x, float y, int speed = 100, int timeout_ms = 3000);
void back_to(float x, float y, int speed = 100, int timeout_ms = 3000);
// back_past: like back_to but doesn't slow down and settle at the end (stops ~2" early,
// still moving). Faster, less exact: use it when the next move takes over anyway
// (e.g. right before a drive_time into the wall).
void back_past(float x, float y, int speed = 100, int timeout_ms = 3000);

// Turn in place to face the point (x, y), or to face a heading.
void face(float x, float y, int speed = 100, int timeout_ms = 1500);
void face_heading(float heading, int speed = 100, int timeout_ms = 1500);

// Drive on a curve to (x, y) and arrive facing `heading`. back_to_pose drives
// back first, and `heading` is still the way the FRONT faces when it arrives.
void drive_to_pose(float x, float y, float heading, int speed = 100, int timeout_ms = 4000);
void back_to_pose(float x, float y, float heading, int speed = 100, int timeout_ms = 4000);

// ---- Waiting -------------------------------------------------------------------
void wait(int ms);

// ---- Lift ----------------------------------------------------------------------
void lift_goal();               // start raising the lift to alliance-goal height (does NOT wait)
void lift_down();               // start lowering the lift (does NOT wait)
void lift_wait(int timeout_ms = 1500);   // wait until the lift has reached its last target

// ---- Claw ----------------------------------------------------------------------
// The claw flips between three positions (same as the B button on the controller):
//   claw_in()  = vertical (resting)      claw_out() = horizontal      claw_up() = slightly up
// Each one moves the claw and waits until it is there.
void claw_in();
void claw_out();
void claw_up();
// One B-button press, same as driver control. Each moves the claw and waits.
//   claw_next()        = intake OFF:     vertical -> highest -> angled -> horizontal (90) -> vertical
//   claw_next_intake() = intake running: vertical -> highest -> horizontal (90) -> vertical
// claw_in() / claw_out() / claw_up() also set where the cycle is.
void claw_next();
void claw_next_intake();
// Spin the claw to drop the pin (the A button). speed 0-127 (negative = reverse),
// ms = how long. Leave the numbers out to use CLAW_DROP_SPEED / CLAW_DROP_MS in mechanisms.h.
void claw_drop(int speed = 1000, int ms = 0);

// ---- Intake / claw spin --------------------------------------------------------
void intake(int speed);         // intake roller, 0 = stop, negative = reverse
void spin_claw(int speed);      // claw spin motor, 0 = stop, negative = reverse (stays on until you set 0)
void lift_secs(int ms, bool up);  // run the cascade up (true, like L1) or down (false, like L2) for ms, then stop. Waits until done

// ---- Which auton was picked (for mirroring) ------------------------------------
bool on_right();                // true if RIGHT was selected
bool on_left();                 // true if LEFT was selected
bool is_red();
bool is_blue();
// Flips the sign for a RIGHT auton, so you write the LEFT version once.
//   face_heading(mirror(-85));   // faces left on LEFT, right on RIGHT
float mirror(float degrees);

// ---- Internal: used by autonomous.cpp ------------------------------------------
void auton_cmds_reset();                  // call once when the auton starts
void auton_status(const char* msg);       // shows a message on the Brain screen
