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
	//optical_sensor.set_led_pwm(100);

	// Port check for smart ports 1-14. M=motor I=imu R=rotation D=distance -=nothing
	// Any mismatch between "code expects" and "brain sees" means wrong port or loose cable.
	pros::Task port_check_task([]() {
		const char* expected = "MMMMMMM-IMDRRR";
		while (true) {
			char seen[15];
			for (int i = 0; i < 14; i++) {
				switch (pros::Device::get_plugged_type(i + 1)) {
					case pros::DeviceType::motor: seen[i] = 'M'; break;
					case pros::DeviceType::imu: seen[i] = 'I'; break;
					case pros::DeviceType::rotation: seen[i] = 'R'; break;
					case pros::DeviceType::distance: seen[i] = 'D'; break;
					case pros::DeviceType::none: seen[i] = '-'; break;
					default: seen[i] = '?'; break;
				}
			}
			seen[14] = '\0';
			pros::lcd::print(0, "ports  12345678901234");
			pros::lcd::print(1, "expect %s", expected);
			pros::lcd::print(2, "brain  %s", seen);
			pros::delay(200);
		}
	});
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

