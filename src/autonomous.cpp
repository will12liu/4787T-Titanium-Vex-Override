#include "main.h"
#include "robot-config.h"

// Drives both sides at the given voltage (-127 to 127) for the given time, then stops.
static void drive_for(int voltage, int duration_ms) {
    left_motors.move(voltage);
    right_motors.move(voltage);
    pros::delay(duration_ms);
    left_motors.move(0);
    right_motors.move(0);
}

void autonomous() {
	// Basic time-based auto. Voltages/durations are untuned guesses.
	drive_for(90, 800);
#if USE_CLAW_PISTON
	claw_piston.set_value(true);
#endif
	pros::delay(300);
	drive_for(-90, 800);
}
