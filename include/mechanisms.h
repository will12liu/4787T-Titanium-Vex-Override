#pragma once
#include "main.h"
#include "robot-config.h"

// Subsystem helpers shared by autonomous and (optionally) driver control.
// Every physical action is one small function so a mechanism redesign
// (e.g. the upcoming pneumatic claw flip) only changes this file.

// ---- Claw flip (motor-driven today) ------------------------------------------
// Vertical = where the claw sits at boot (position tared to 0). Horizontal is 940
// degrees of travel from there (it was 60 -> 1000, same travel, but the 60 made the
// claw drive up the moment driver control started).
constexpr double CLAW_TURN_VERTICAL_DEG = 25.0;
constexpr double CLAW_TURN_HORIZONTAL_DEG = 900.0;
constexpr double cascade_intake_deg = 180.0;  // cascade motor position to run the intake (claw side) at a low speed
// B button's 2nd press: the claw lifts a bit past horizontal (off the cone / goal).
// It is a fraction of the vertical-to-horizontal travel past horizontal: 0.25 = 235 deg
// past horizontal. Lower = less lift. If it lifts the wrong way, use a negative number.
constexpr double CLAW_LIFT_FRACTION = 0.25;
constexpr double CLAW_TURN_LIFT_DEG = 2*CLAW_TURN_HORIZONTAL_DEG * CLAW_LIFT_FRACTION;
constexpr double CLAW_TURN_HIGHEST_DEG = CLAW_TURN_HORIZONTAL_DEG + CLAW_TURN_LIFT_DEG;
// Dropping the pin = the A button's claw spin. Speed 127 = A's spin, if it spins the
// wrong way for dropping use -127. Time is how long it spins.
constexpr int CLAW_DROP_SPEED = 127;
constexpr int CLAW_DROP_MS = 700;
constexpr int CLAW_TURN_VELOCITY = 200;
constexpr int CLAW_TURN_DOWN_VELOCITY = 100;

// ---- Lift heights, cascade motor degrees from the fully-lowered position ----
// Heights are of the claw scoring point (where the held pin's bottom is) above
// the floor, in inches. Degrees are worked out by straight-line interpolation
// between lift fully down and fully up, so only the two ends need measuring.
// Read the lift degrees on the tuning screen ("lift" on the bottom line).
constexpr double CLAW_HEIGHT_MAX_IN = 42.1;   // measured: 107 cm, lift fully up
constexpr double CLAW_HEIGHT_DOWN_IN = 0.0;   // PLACEHOLDER: same point, lift fully down
constexpr double LIFT_MAX_DEG = 0.0;          // PLACEHOLDER: lift degrees when fully up (0 = not measured yet)
constexpr double LIFT_CLEARANCE_IN = 1.0;     // how far above the goal top to hold the pin

// Cascade brake mode when no input, applied to BOTH motors. COAST = rolls to a
// stop slowly, BRAKE = stops almost instantly, HOLD = actively holds.
constexpr pros::motor_brake_mode_e_t CASCADE_BRAKE_MODE = pros::E_MOTOR_BRAKE_BRAKE;

// Goal heights from the manual: alliance 3.25", neutral short 5.77", centre 8.77".
constexpr double lift_deg_for_height(double height_in, double fallback_deg) {
	return LIFT_MAX_DEG <= 0.0 ? fallback_deg
	     : (height_in - CLAW_HEIGHT_DOWN_IN) / (CLAW_HEIGHT_MAX_IN - CLAW_HEIGHT_DOWN_IN) * LIFT_MAX_DEG;
}
constexpr double LIFT_DOWN_DEG = 0.0;
// Second number is the old guess, used until LIFT_MAX_DEG is measured.
constexpr double LIFT_ALLIANCE_GOAL_DEG = lift_deg_for_height(3.25 + LIFT_CLEARANCE_IN, 400.0);
constexpr double LIFT_SHORT_GOAL_DEG = lift_deg_for_height(5.77 + LIFT_CLEARANCE_IN, 700.0);
constexpr double LIFT_TALL_GOAL_DEG = lift_deg_for_height(8.77 + LIFT_CLEARANCE_IN, 1100.0);
constexpr int LIFT_VELOCITY = 200;

// ---- Timing, ms. PLACEHOLDER — tune on hardware ----------------------------
constexpr int CLAW_PISTON_SETTLE_MS = 250;   // time for the piston to fully open/close
constexpr int INTAKE_GRAB_SETTLE_MS = 500;   // intake run time after reaching a pin
constexpr int PIN_DROP_SETTLE_MS = 300;      // time for a released pin to clear the claw

// Call once from initialize() with the lift fully down and the claw vertical.
void mech_init();

// Intake roller. voltage in [-127, 127], 0 stops.
void intake_set(int voltage);

// Lift. lift_to() is non-blocking; lift_wait() blocks until within tol or timeout.
void lift_to(double deg, int velocity = LIFT_VELOCITY);
bool lift_wait(double deg, int timeout_ms, double tol_deg = 20.0);

// Claw flip. Non-blocking; claw_flip_wait() blocks until in position or timeout.
// Keeps the shared claw_turn_horizontal flag in sync so opcontrol agrees with auton.
void claw_flip(bool horizontal);
bool claw_flip_wait(int timeout_ms, double tol_deg = 30.0);
// Move the claw to the 'slightly up' position (B's 2nd press) and wait for it.
bool claw_lift_off(int timeout_ms = 1500, double tol_deg = 30.0);

// Pneumatic claw jaw. No-op when USE_CLAW_PISTON is 0.
void claw_set_open(bool open);

// ---- Composite actions used by autonomous ----------------------------------
// Prepare to take a pin: claw open, intake running. Call before driving onto a pin.
void grab_pin_start();
// Finish taking a pin: let the intake settle, close the claw, stop the intake.
void grab_pin_finish();
// Blocking. Assumes a pin is held and the robot is positioned at the goal:
// lift to lift_deg, flip the claw to horizontal, spin the claw (A button) to drop the
// pin, then move the claw to the slightly-up position (B's 2nd press). Leaves the
// lift up.
void score_pin(double lift_deg);
// Stop every non-drive motor.
void mech_stop_all();
