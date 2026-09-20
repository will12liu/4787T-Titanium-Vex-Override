#include "main.h"
#include "robot-config.h"

void initialize() {
	// Uses pros::screen (not pros::lcd). Shows expected vs. seen devices on smart ports 1-14:
	// M=motor I=imu R=rotation D=distance -=nothing. A mismatch means wrong port or loose cable.
	pros::Task port_check_task([]() {
		const char* expected = "MMMMMMM-IMDRRR";
		pros::Controller master(pros::E_CONTROLLER_MASTER);
		pros::delay(300);
		master.set_text(0, 0, "init ran");
		pros::screen::set_pen(pros::Color::white);
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
			pros::screen::print(pros::E_TEXT_MEDIUM, 1, "ports  12345678901234");
			pros::screen::print(pros::E_TEXT_MEDIUM, 2, "expect %s", expected);
			pros::screen::print(pros::E_TEXT_MEDIUM, 3, "brain  %s", seen);
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

