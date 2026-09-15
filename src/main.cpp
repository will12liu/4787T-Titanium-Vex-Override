#include "main.h"
#include "robot-config.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	optical_sensor.set_led_pwm(100);

	pros::Task print_task([]() {
		while (true) {
			pros::lcd::print(0, "Hue: %.1f", optical_sensor.get_hue());
			pros::lcd::print(1, "Prox: %d", optical_sensor.get_proximity());
			pros::lcd::print(2, "Brightness: %.2f", optical_sensor.get_brightness());
			pros::delay(20);
		}
	});

	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
// Drives both sides at the given voltage (-127 to 127) for the given time, then stops.
void drive_for(int voltage, int duration_ms) {
	left_motors.move(voltage);
	right_motors.move(voltage);
	pros::delay(duration_ms);
	left_motors.move(0);
	right_motors.move(0);
}

// Turns in place: positive voltage turns right, negative turns left.
void turn_for(int voltage, int duration_ms) {
	left_motors.move(voltage);
	right_motors.move(-voltage);
	pros::delay(duration_ms);
	left_motors.move(0);
	right_motors.move(0);
}

void autonomous() {
	// === STRATEGY: Toggles first, then yellow Pins ===
	// Yellow Pins only score points if the Quadrant's Toggle is set to our
	// Alliance color (rule SC5). Setting our Toggle(s) first is required
	// before any yellow Pin scoring counts for us.
	// Reference: Autonomous Win Point (SC8) needs 7+ Pins scored for us and
	// 3+ Goals with 2+ Pins each, using only our side of the Autonomous Line.

	// --- Step 1: Score the Preload ---
	// Every robot starts with one Pin (SG5). Score it first, it's guaranteed
	// points with no searching needed.
	// TODO: once claw/intake mechanism is finalized, add code here to
	// deposit the preload into the nearest Goal.

	// --- Step 2: Set our Quadrant Toggle(s) to our Alliance color ---
	// TODO: drive to Toggle, use claw/mechanism to flip it to our color.
	// A Toggle only counts as "set" if it's fully seated and not touching
	// a Robot at rest (SC4), so don't linger in contact with it after flipping.
	// drive_for(...);
	// turn_for(...);

	// --- Step 3: Collect and score yellow Pins (now Owned by us) ---
	// Only do this AFTER Step 2, otherwise these Pins score zero (SC5a).
	// TODO: pick up yellow Pins, drive to a Goal on our side, score them.

	// --- Step 4: If time remains, score additional red/blue Alliance Pins ---
	// These always count for us regardless of Toggle state (SC3a/b), so
	// they're a safe fallback if Toggle-flipping or yellow Pins fail/run out
	// of time.
	// TODO: add once intake mechanism is finalized.

	// === RULES TO NEVER VIOLATE DURING THIS PERIOD ===
	// - Never cross the Autonomous Line (SG7) — any Violation auto-awards
	//   the Autonomous Bonus to the opponent (SC7c).
	// - Stay off Midfield unless intentional — Midfield scoring doesn't
	//   count toward the Autonomous Bonus anyway (SC7a), so it's not worth
	//   the risk during auto.
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	bool claw_open = false;

	while (true) {
		int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		left_motors.move(dir + turn);
		right_motors.move(dir - turn);

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
			claw_open = !claw_open;
			claw_piston.set_value(claw_open);
		}

		pros::delay(20);
	}
}