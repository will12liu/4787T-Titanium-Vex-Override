#include "main.h"
#include "robot-config.h"
#include "mechanisms.h"
#include "auton_selector.h"
#include "auton.h"
#include "auton_cmds.h"
#include "estop.h"

// =============================================================================
// Auton startup + entry point. The routine you edit is in src/routine.cpp and the
// commands it uses (move, turn, ...) are in src/auton_cmds.cpp.
// =============================================================================

namespace {
bool imu_ready = false;
}

// Gets the IMU ready (its heading is all the auton commands need). LemLib's own
// odometry is NOT started any more: it crashed the brain at startup, and move()/
// turn() don't use it. That also means the Brain's TUNE screen is unavailable.
void auton_init() {
	if (imu_ready) return;
	// The IMU can take a moment to enumerate after power-on.
	bool imu_present = false;
	for (int i = 0; i < 20 && !imu_present; i++) {
		imu_present = pros::Device::get_plugged_type(imu.get_port()) == pros::DeviceType::imu;
		if (!imu_present) pros::delay(50);
	}
	if (imu_present) {
		pros::screen::print(pros::E_TEXT_MEDIUM, 6, "imu: calibrating...               ");
		imu.reset(true);   // blocks ~2-3 s until calibrated
		imu_ready = true;
		pros::screen::print(pros::E_TEXT_MEDIUM, 6, "imu: READY                        ");
	} else {
		pros::screen::print(pros::E_TEXT_MEDIUM, 6, "imu: NOT FOUND on port %d          ", imu.get_port());
	}
	printf("DBG imu_ready=%d\n", imu_ready);
}

bool auton_imu_ready() { return imu_ready; }

// LemLib odometry is not used, so the tuning screen (which needs it) stays off.
bool auton_odom_ready() { return false; }

void autonomous() {
	auton_cmds_reset();
	auton_status("start");
	estop_arm();   // L1 + R1 stops the run (test runs only, never in a match)

	if (!imu_ready) auton_status("NO IMU: turns skipped");
	run_routine();   // the routine itself lives in src/routine.cpp

	estop_disarm();
	left_motors.move(0);
	right_motors.move(0);
	mech_stop_all();
	auton_status(estop_triggered() ? "E-STOPPED" : "end");
}

// Same as autonomous() (nothing calls this any more now that R2 is removed).
void auton_drive_test() {
	autonomous();
}
