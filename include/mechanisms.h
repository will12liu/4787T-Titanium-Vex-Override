#pragma once
#include "main.h"
#include "robot-config.h"

// Subsystem helpers shared by autonomous and (optionally) driver control.
// Every physical action is one small function so a mechanism redesign
// only changes this file.

// ---- Claw ------------------------------------------------------------------------
// Dropping the pin = the A button's claw spin. Speed 127 = A's spin, if it spins the
// wrong way for dropping use -127. Time is how long it spins.
constexpr int CLAW_DROP_SPEED = 127;
constexpr int CLAW_DROP_MS = 700;

// ---- Claw driver control (R1 / R2) ---------------------------------------------------
// Two toggles on the claw spin motor. Pressing one turns the other off.
//   R1 = GRAB: spin in until pressed again.
//   R2 (and A) = RELEASE: spin out until pressed again.
constexpr int CLAW_GRAB_SPEED = -127;     // motor command while grabbing (negative = in, on this robot)
constexpr int CLAW_RELEASE_SPEED = 85;    // motor command while releasing
void claw_grab_toggle();                  // R1 pressed
void claw_release_toggle();               // R2 / A pressed
// Call every driver-control loop: drives the claw motor for the current mode (or stops it).
void claw_driver_update();
// Turn both modes off and stop the claw (after an auton run / macro, or on E-STOP).
void claw_driver_reset();

// ---- Lift heights ----------------------------------------------------------------
// ALL lift numbers are rotation-sensor numbers, the "rot" value on the tuning screen / lift_pos().
// On this sensor the number goes DOWN as the lift goes UP (bottom 10, top -1444, see the limits
// below), so a higher lift = a more negative number. Heights are of the claw scoring point
// (where the held pin's bottom is) above the floor, in inches, worked out by straight-line
// interpolation between the two limits, so only the two ends need measuring.
constexpr double CLAW_HEIGHT_MAX_IN = 42.1;   // measured: 107 cm, lift fully up
constexpr double CLAW_HEIGHT_DOWN_IN = 0.0;   // PLACEHOLDER: same point, lift fully down
constexpr double LIFT_CLEARANCE_IN = 1.0;     // how far above the goal top to hold the pin

// Cascade brake mode when no input, applied to BOTH motors. COAST = rolls to a
// stop slowly, BRAKE = stops almost instantly, HOLD = actively holds.
constexpr pros::motor_brake_mode_e_t CASCADE_BRAKE_MODE = pros::E_MOTOR_BRAKE_BRAKE;

// Lift position now comes from the rotation sensor on the middle axis (motor slip can't fool it).
// Sensor degrees * this = "lift degrees" (the numbers above, like cascade_intake_deg). 1.0 = raw axis
// degrees; to keep the old motor-degree numbers, set it to (motor degrees / axis degrees) for the lift.
constexpr double LIFT_ROT_TO_LIFT = 1.0;
// R1 intake height window (lift degrees). Inside the dead band the lift just holds. Slipping past the
// re-correct band, it pushes back: up hard if too low, down gently if too high.
constexpr double LIFT_HOLD_BAND_DEG = 6.0;     // stop correcting once this close
constexpr double LIFT_RECORRECT_DEG = 14.0;    // start correcting again once this far off
constexpr int LIFT_UNDER_MIN_VOLT = 40;        // min push going up (against gravity)
constexpr int LIFT_OVER_MIN_VOLT = 30;         // min push going down (was 15: too weak to start it moving)
constexpr int LIFT_OVER_MAX_VOLT = 100;        // cap going down (was 60: too weak for the auton clamp). L2 uses 85
constexpr double LIFT_DOWN_DEG = 10.0;   // = LIFT_LIMIT_BOTTOM_DEG, the lowest sensor reading
// Hard travel limits in lift_pos() units (the "rot" number on the tuning screen). Measured:
// lowest = 10, highest = -1444, so on this sensor the number goes DOWN as the lift goes up.
// cascade_move() refuses to drive past either end and slows down inside LIFT_LIMIT_SLOW_DEG of it.
// Zero is taken at boot (mech_init), so the lift must start fully down.
constexpr double LIFT_LIMIT_BOTTOM_DEG = 10.0;
constexpr double LIFT_LIMIT_TOP_DEG = -1444.0;
constexpr double LIFT_LIMIT_SLOW_DEG = 80.0;   // start slowing this far from an end
constexpr int LIFT_LIMIT_MIN_VOLT = 30;        // slowest the taper goes before the stop
// Sensor number for a claw height in inches (linear between the two limits).
constexpr double lift_for_height(double height_in) {
	return LIFT_LIMIT_BOTTOM_DEG + (height_in - CLAW_HEIGHT_DOWN_IN) / (CLAW_HEIGHT_MAX_IN - CLAW_HEIGHT_DOWN_IN)
	                                   * (LIFT_LIMIT_TOP_DEG - LIFT_LIMIT_BOTTOM_DEG);
}
// Goal heights from the manual: alliance 3.25", neutral short 5.77", centre 8.77". Calculated, not measured:
// check each on the robot (tuning screen "rot") and overwrite with the real number.
constexpr double LIFT_ALLIANCE_GOAL_DEG = lift_for_height(3.25 + LIFT_CLEARANCE_IN);
constexpr double LIFT_SHORT_GOAL_DEG = lift_for_height(5.77 + LIFT_CLEARANCE_IN);
constexpr double LIFT_TALL_GOAL_DEG = lift_for_height(8.77 + LIFT_CLEARANCE_IN);
// R1 intake height. PLACEHOLDER: the old 350 had no sensor reading behind it. Lift to the intake
// height by hand and copy the "rot" number here (it will be negative).
constexpr double cascade_intake_deg = -350.0;
// Anti-tip: driver-control drive power is scaled down as the lift rises. Below LIFT_SLOW_START_FRAC of
// the lift's travel (0 = fully down, 1 = fully up) it drives at full power; from there it eases
// linearly down to LIFT_SLOW_MIN_SCALE at the top.
constexpr double LIFT_SLOW_START_FRAC = 0.35;
constexpr double LIFT_SLOW_MIN_SCALE = 0.4;
constexpr int LIFT_VELOCITY = 200;

// Call once from initialize() with the lift fully down and the claw vertical.
void mech_init();

// Drive the cascade, voltage [-127, 127], + = up. Respects the travel limits: it won't push past
// either end and eases off near them. Every cascade command goes through this, never cascade.move().
// If the rotation sensor is unplugged it can't tell where the lift is, so it doesn't limit.
void cascade_move(int voltage);

// Start the background lift task. Call once from initialize() so it is already running before
// the first lift_to() (the auton creates it from inside its own task otherwise).
void lift_task_start();

// Lift. lift_to() is non-blocking; lift_wait() blocks until within tol or timeout.
// lift_pos() = lift position from the cascade rotation sensor (motor counts drift).
double lift_pos();
// true when the lift is within tol_deg of deg (rotation sensor)
bool lift_at(double deg, double tol_deg = LIFT_HOLD_BAND_DEG + 4.0);
// Stop aiming at the last lift_to() height. Call before moving the cascade by hand.
void lift_release();
void lift_to(double deg, int velocity = LIFT_VELOCITY);
bool lift_wait(double deg, int timeout_ms, double tol_deg = 20.0);

// Drive power multiplier for the current lift height: 1.0 low, down to LIFT_SLOW_MIN_SCALE when
// high. 1.0 if the lift sensor is unplugged.
double lift_drive_scale();

// Stop every non-drive motor.
void mech_stop_all();
