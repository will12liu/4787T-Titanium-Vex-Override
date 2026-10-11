#pragma once

// =============================================================================
// AUTON COMMANDS. The building blocks you use in src/routine.cpp.
// Units: inches, degrees, speed 0-127, time in milliseconds.
// Forward / clockwise / right turn = positive. Backward / counter-clockwise = negative.
// Every command stops right away if E-STOP (L1 + R1) is pressed.
// =============================================================================

// ---- Driving -----------------------------------------------------------------
// Drive straight `inches` (negative = backwards), using the encoders + a PID that
// slows down near the target, and the IMU to stay straight.
//   speed   = top speed 0-127
//   kP, kD  = leave out to use the current default (set by move_pid). Per-call
//             override: move(24, 100, 6.0, 0.3)
//   timeout = give up after this many ms. 0 = automatic.
void move(float inches, int speed = 100, float kP = -1, float kD = -1, int timeout_ms = 0);

// Drive straight at `speed` for `ms` milliseconds, no distance sensing. Good for
// pushing into things. Negative speed = backwards. Stays straight using the IMU.
void drive_time(int speed, int ms);

// ---- Turning -----------------------------------------------------------------
// Turn `degrees` from where the robot is facing now. Positive = clockwise (right),
// negative = counter-clockwise (left). Uses the IMU + a PID.
void turn(float degrees, int speed = 100, float kP = -1, float kD = -1, int timeout_ms = 0);

// Turn to face `degrees` measured from the way the robot was facing when the auton
// started (0 = the start direction, positive = clockwise).
void turn_to(float degrees_from_start, int speed = 100, float kP = -1, float kD = -1, int timeout_ms = 0);

// ---- PID defaults (apply to every move / turn after this line) -----------------
//   kP = how hard it pushes per inch (move) or per degree (turn) of error. Bigger =
//        faster / stronger, but overshoots if too big.
//   kD = brakes as it closes in. Bigger = less overshoot, but slower to finish.
void move_pid(float kP, float kD);
void turn_pid(float kP, float kD);

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
// Spin the claw to drop the pin (the A button). speed 0-127 (negative = reverse),
// ms = how long. Leave the numbers out to use CLAW_DROP_SPEED / CLAW_DROP_MS in mechanisms.h.
void claw_drop(int speed = 1000, int ms = 0);

// ---- Intake / claw spin --------------------------------------------------------
void intake(int speed);         // intake roller, 0 = stop, negative = reverse
void spin_claw(int speed);      // claw spin motor, 0 = stop, negative = reverse (stays on until you set 0)

// ---- All-in-one (optional) -----------------------------------------------------
// Same as: lift_goal(); lift_wait(); claw_out(); claw_drop(); claw_up();
void score();

// ---- Which auton was picked (for mirroring) ------------------------------------
bool on_right();                // true if RIGHT was selected
bool on_left();                 // true if LEFT was selected
bool is_red();
bool is_blue();
// Flips the sign for a RIGHT auton, so you write the LEFT version once.
//   turn(mirror(-85));   // turns left on LEFT, right on RIGHT
float mirror(float degrees);

// ---- Internal: used by autonomous.cpp ------------------------------------------
void auton_cmds_reset();                  // call once when the auton starts
void auton_status(const char* msg);       // shows a message on the Brain screen
