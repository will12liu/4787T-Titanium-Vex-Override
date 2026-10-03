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
// PLACEHOLDER — measure on the robot: lower the lift fully, power on (tare),
// raise it until the claw clears each goal, read cascade.get_position().
// Goal heights from the manual: alliance 3.25", neutral short 5.77", centre 8.77".
constexpr double LIFT_DOWN_DEG = 0.0;
constexpr double LIFT_ALLIANCE_GOAL_DEG = 400.0;
constexpr double LIFT_SHORT_GOAL_DEG = 700.0;
constexpr double LIFT_TALL_GOAL_DEG = 1100.0;
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
