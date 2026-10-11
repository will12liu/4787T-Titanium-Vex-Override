#include "main.h"
#include "robot-config.h"
#include "mechanisms.h"
#include "auton_selector.h"
#include "auton.h"
#include "tuning.h"
#include "estop.h"
#include <cstdlib>

namespace {

constexpr int NUM_PORTS = 21;

// Builds the expected device-type string for smart ports 1..21 from the
// devices declared in robot-config.cpp, so it can't go stale when ports move.
void build_expected(char* expected) {
	for (int i = 0; i < NUM_PORTS; i++) expected[i] = '-';
	expected[NUM_PORTS] = '\0';
	auto mark = [&](int port, char c) {
		port = std::abs(port);
		if (port >= 1 && port <= NUM_PORTS) expected[port - 1] = c;
	};
	for (auto p : left_motors.get_port_all()) mark(p, 'M');
	for (auto p : right_motors.get_port_all()) mark(p, 'M');
	for (auto p : cascade.get_port_all()) mark(p, 'M');
	mark(claw_turn.get_port(), 'M');
	mark(claw_spin.get_port(), 'M');
	mark(intake_motor.get_port(), 'M');
	mark(imu.get_port(), 'I');
#if USE_TRACKING_WHEELS
	mark(distance_sensor.get_port(), 'D');
#if USE_VERTICAL_POD
	mark(vertical_rotation.get_port(), 'R');
#endif
#if USE_HORIZONTAL_POD
	mark(horiz_odom.get_port(), 'R');
#endif
#endif
}

} // namespace

void initialize() {
	// Uses pros::screen (not pros::lcd). Shows expected vs. seen devices on smart ports 1-21:
	// M=motor I=imu R=rotation D=distance -=nothing. A mismatch means wrong port or loose cable.
	pros::Task port_check_task([]() {
		char expected[NUM_PORTS + 1];
		build_expected(expected);
		pros::Controller master(pros::E_CONTROLLER_MASTER);
		pros::delay(300);
		master.set_text(0, 0, "init ran");
		pros::screen::set_pen(pros::Color::white);
		while (true) {
			if (tuning_active()) {   // tuning screen owns the display
				pros::delay(200);
				continue;
			}
			char seen[NUM_PORTS + 1];
			for (int i = 0; i < NUM_PORTS; i++) {
				switch (pros::Device::get_plugged_type(i + 1)) {
					case pros::DeviceType::motor: seen[i] = 'M'; break;
					case pros::DeviceType::imu: seen[i] = 'I'; break;
					case pros::DeviceType::rotation: seen[i] = 'R'; break;
					case pros::DeviceType::distance: seen[i] = 'D'; break;
					case pros::DeviceType::none: seen[i] = '-'; break;
					default: seen[i] = '?'; break;
				}
			}
			seen[NUM_PORTS] = '\0';
			pros::screen::print(pros::E_TEXT_MEDIUM, 1, "ports  123456789012345678901");
			pros::screen::print(pros::E_TEXT_MEDIUM, 2, "expect %s", expected);
			pros::screen::print(pros::E_TEXT_MEDIUM, 3, "brain  %s", seen);
			pros::delay(200);
		}
	});

	mech_init();             // tare lift + claw flip, set brake modes. Lift must be fully down.
	auton_selector_start();  // LEFT / RIGHT buttons on the Brain + controller D-pad
	auton_init();            // find IMU, calibrate LemLib chassis (blocks ~2-3 s)
	tuning_start();          // TUNE button on the Brain: PID + sensor tuning
	estop_start();           // L1 + R1 stops auton test runs
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

