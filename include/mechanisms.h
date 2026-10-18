#pragma once
#include "main.h"
#include "robot-config.h"

// Subsystem helpers shared by autonomous and (optionally) driver control.
// Every physical action is one small function so a mechanism redesign
// only changes this file.

// ---- Claw ------------------------------------------------------------------------
constexpr double cascade_intake_deg = 350.0;  // cascade motor position R1 lifts to for the intake. 300 was a bit high
// Dropping the pin = the A button's claw spin. Speed 127 = A's spin, if it spins the
// wrong way for dropping use -127. Time is how long it spins.
constexpr int CLAW_DROP_SPEED = 127;
constexpr int CLAW_DROP_MS = 700;

// ---- Claw optical sensor (detects a pin inside the claw, stops the inward spin) ----
// Proximity reads 0 (nothing) to 255 (touching). The pin counts as "in the claw" once the
// reading is at least this. Hold a pin in the claw and read the value to tune it.
constexpr int CLAW_OPTICAL_PROX_MIN = 150;
constexpr int CLAW_OPTICAL_CONFIRM_MS = 80;   // must see it this long in a row (ignores flicker)

// Call once from initialize().
void claw_optical_init();
// Raw check, true while something is close enough to the sensor.
bool claw_sees_pin();
// Debounced. Call every loop. Returns true once, on the loop the pin newly counts as in the claw
// (so the driver can spin again with the pin already inside without it re-triggering).
bool claw_pin_just_grabbed();
// Auton: spin the claw at speed until a pin is detected or timeout. Returns true if one was grabbed.
bool claw_spin_until_pin(int speed, int timeout_ms);

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
// Lift position now comes from the rotation sensor on the middle axis (motor slip can't fool it).
// Sensor degrees * this = "lift degrees" (the numbers above, like cascade_intake_deg). 1.0 = raw axis
// degrees; to keep the old motor-degree numbers, set it to (motor degrees / axis degrees) for the lift.
constexpr double LIFT_ROT_TO_LIFT = 1.0;
// R1 intake height window (lift degrees). Inside the dead band the lift just holds. Slipping past the
// re-correct band, it pushes back: up hard if too low, down gently if too high.
constexpr double LIFT_HOLD_BAND_DEG = 6.0;     // stop correcting once this close
constexpr double LIFT_RECORRECT_DEG = 14.0;    // start correcting again once this far off
constexpr int LIFT_UNDER_MIN_VOLT = 40;        // min push going up (against gravity)
constexpr int LIFT_OVER_MIN_VOLT = 15;         // min push going down
constexpr int LIFT_OVER_MAX_VOLT = 60;         // cap going down so it can't slam / bounce back up
constexpr double LIFT_DOWN_DEG = 0.0;
// Second number is the old guess, used until LIFT_MAX_DEG is measured.
constexpr double LIFT_ALLIANCE_GOAL_DEG = lift_deg_for_height(3.25 + LIFT_CLEARANCE_IN, 400.0);
constexpr double LIFT_SHORT_GOAL_DEG = lift_deg_for_height(5.77 + LIFT_CLEARANCE_IN, 700.0);
constexpr double LIFT_TALL_GOAL_DEG = lift_deg_for_height(8.77 + LIFT_CLEARANCE_IN, 1100.0);
constexpr int LIFT_VELOCITY = 200;

// Call once from initialize() with the lift fully down and the claw vertical.
void mech_init();

// Lift. lift_to() is non-blocking; lift_wait() blocks until within tol or timeout.
// lift_pos() = lift position from the cascade rotation sensor (motor counts drift).
double lift_pos();
// true when the lift is within tol_deg of deg (rotation sensor)
bool lift_at(double deg, double tol_deg = LIFT_HOLD_BAND_DEG + 4.0);
// Stop aiming at the last lift_to() height. Call before moving the cascade by hand.
void lift_release();
void lift_to(double deg, int velocity = LIFT_VELOCITY);
bool lift_wait(double deg, int timeout_ms, double tol_deg = 20.0);

// Stop every non-drive motor.
void mech_stop_all();
