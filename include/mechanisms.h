#pragma once
#include "main.h"
#include "robot-config.h"

// Subsystem helpers shared by autonomous and (optionally) driver control.
// Every physical action is one small function so a mechanism redesign
// (e.g. the upcoming pneumatic claw flip) only changes this file.

// ---- Claw flip (motor-driven today) ------------------------------------------
constexpr double CLAW_TURN_VERTICAL_DEG = 60.0;
constexpr double CLAW_TURN_HORIZONTAL_DEG = 1000.0;
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

// Pneumatic claw jaw. No-op when USE_CLAW_PISTON is 0.
void claw_set_open(bool open);

// ---- Composite actions used by autonomous ----------------------------------
// Prepare to take a pin: claw open, intake running. Call before driving onto a pin.
void grab_pin_start();
// Finish taking a pin: let the intake settle, close the claw, stop the intake.
void grab_pin_finish();
// Blocking. Assumes a pin is held and the robot is positioned at the goal:
// lift to lift_deg, flip the claw out over the goal, release, flip back, close,
// then start lowering the lift (does not wait for it to come down).
void score_pin(double lift_deg);
// Stop every non-drive motor.
void mech_stop_all();
