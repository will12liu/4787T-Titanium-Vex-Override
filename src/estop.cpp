#include "estop.h"
#include "main.h"
#include "robot-config.h"
#include "mechanisms.h"

namespace {

bool armed = false;
bool triggered = false;

void stop_everything() {
	chassis.cancelAllMotions();   // otherwise LemLib keeps driving the motors
	left_motors.move(0);
	right_motors.move(0);
	mech_stop_all();
}

void estop_task_fn() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	while (true) {
		if (armed) {
			if (!triggered && master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
			    master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
				triggered = true;
				printf("DBG E-STOP\n");
				pros::screen::print(pros::E_TEXT_MEDIUM, 5, "E-STOP (L1+R1)                          ");
				master.rumble("---");
			}
			// Keep everything stopped until the routine notices and returns, in
			// case it issues one more command before its next check.
			if (triggered) stop_everything();
		}
		pros::delay(10);
	}
}

} // namespace

void estop_start() {
	static pros::Task estop_task(estop_task_fn);
}

void estop_arm(bool force) {
	triggered = false;
	armed = force || !pros::competition::is_connected();
}

void estop_disarm() {
	armed = false;
}

bool estop_triggered() { return triggered; }
