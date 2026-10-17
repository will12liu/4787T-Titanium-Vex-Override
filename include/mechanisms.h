#pragma once
#include "main.h"
#include "robot-config.h"

// Subsystem helpers shared by autonomous and (optionally) driver control.
// Every physical action is one small function so a mechanism redesign
// (e.g. the upcoming pneumatic claw flip) only changes this file.

// ---- Claw flip (motor-driven today) ------------------------------------------
// Claw angles are motor degrees from where the claw sits at boot (tared to 0 in
// mech_init). The claw boots slanted (~25 deg), and that slanted spot IS "down".
// About 10 motor degrees per claw degree, so claw angle ~= 25 + number / 10.
constexpr double CLAW_TURN_VERTICAL_DEG = 25.0;   // down, a bit above the boot spot (X's 2nd press)
constexpr double CLAW_TURN_HORIZONTAL_DEG = 700.0; // 90 deg (X's 1st press). 860 was ~105, 760 still too high
constexpr double CLAW_TURN_LIFT_DEG = 230.0;       // B's 2nd press: small acute angle to pick up pins
constexpr double CLAW_TURN_HIGHEST_DEG = 1060.0;   // B's 1st press: top angle. 1100 too high, 1025 too low
constexpr double cascade_intake_deg = 350.0;  // cascade motor position R1 lifts to for the intake. 300 was a bit high
// Cascade up (R1 intake): claw drops to slightly slanted. That is below the boot spot,
// so it only goes there while the cascade is at least CLAW_CLEAR_CASCADE_DEG high.
// More negative = closer to straight down. -150 was too far, -90 not vertical enough.
constexpr double CLAW_TURN_INTAKE_DEG = -125.0;
constexpr double CLAW_CLEAR_CASCADE_DEG = cascade_intake_deg - 40.0;
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

// Intake roller. voltage in [-127, 127], 0 stops.
void intake_set(int voltage);

// Lift. lift_to() is non-blocking; lift_wait() blocks until within tol or timeout.
// lift_pos() = lift position from port 19 only (port 20's count drifts).
double lift_pos();
// true when the lift is within tol_deg of deg (rotation sensor)
bool lift_at(double deg, double tol_deg = LIFT_HOLD_BAND_DEG + 4.0);
// Stop aiming at the last lift_to() height. Call before moving the cascade by hand.
void lift_release();
void lift_to(double deg, int velocity = LIFT_VELOCITY);
bool lift_wait(double deg, int timeout_ms, double tol_deg = 20.0);

// Claw flip. Non-blocking; claw_flip_wait() blocks until in position or timeout.
// Keeps the shared claw_turn_horizontal flag in sync so opcontrol agrees with auton.
void claw_flip(bool horizontal);
bool claw_flip_wait(int timeout_ms, double tol_deg = 30.0);
// Move the claw to the 'slightly up' position (B's 2nd press) and wait for it.
bool claw_lift_off(int timeout_ms = 1500, double tol_deg = 30.0);

// Stop every non-drive motor.
void mech_stop_all();
