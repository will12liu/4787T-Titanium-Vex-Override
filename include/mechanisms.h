#pragma once
#include "robot-config.h"

// R1 toggles grab; R2 / A toggles release. Pressing the active mode stops the claw.
constexpr int CLAW_GRAB_SPEED = -127;
constexpr int CLAW_RELEASE_SPEED = 85;
void claw_grab_toggle();
void claw_release_toggle();
void claw_driver_update();  // Apply the selected mode each driver loop.
void claw_driver_reset();

// Lift sensor readings decrease as the lift rises. Start fully down before powering on.
constexpr double LIFT_ROT_TO_LIFT = 1.0;
constexpr double LIFT_LIMIT_BOTTOM_DEG = 10.0;
constexpr double LIFT_LIMIT_TOP_DEG = -1444.0;
constexpr double LIFT_LIMIT_SLOW_DEG = 80.0;
constexpr int LIFT_LIMIT_MIN_POWER = 30;
constexpr pros::motor_brake_mode_e_t CASCADE_BRAKE_MODE = pros::E_MOTOR_BRAKE_BRAKE;

// Background lift controller: stop inside 6 degrees; correct again beyond 14 degrees.
constexpr double LIFT_HOLD_BAND_DEG = 6.0;
constexpr double LIFT_RECORRECT_DEG = 14.0;
constexpr int LIFT_UP_MIN_POWER = 40;
constexpr int LIFT_DOWN_MIN_POWER = 30;
constexpr int LIFT_DOWN_MAX_POWER = 100;

// Reduce drive power above 35% lift travel, reaching 40% power at full height.
constexpr double LIFT_SLOW_START_FRAC = 0.35;
constexpr double LIFT_SLOW_MIN_SCALE = 0.4;

void mech_init();
void mech_stop_all();
void lift_task_start();
double lift_pos();
void lift_to(double rotation);  // Start moving in the background.
void lift_release();           // Cancel the target before moving the lift manually.
double lift_drive_scale();

// Motor power [-127, 127], positive = up. Enforces travel limits and slows near either end.
// With a missing rotation sensor, power passes through because position is unknown.
void cascade_move(int power);
