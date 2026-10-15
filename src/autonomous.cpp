#include "main.h"
#include "robot-config.h"
#include "mechanisms.h"
#include "auton_selector.h"
#include "auton.h"
#include "auton_cmds.h"
#include "estop.h"

// =============================================================================
// Auton startup + entry point. The routine you edit is in src/routine.cpp and the
// commands it uses (back_to, face_heading, ...) are in src/auton_cmds.cpp.
// =============================================================================

namespace {
bool imu_ready = false;
bool odom_ready = false;
}

// Calibrates the IMU, then starts LemLib odometry (tracking the robot's position).
// If startup freezes, the Brain screen line says which step it is stuck on, and
// `pros terminal` shows the last DBG line printed.
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
		printf("DBG imu calibrate start\n");
		imu.reset(true);   // blocks ~2-3 s until calibrated
		printf("DBG imu calibrate done\n");
		imu_ready = true;
		// false = skip LemLib's own IMU calibration. Its wait loop has no timeout
		// and froze startup; the IMU is already calibrated above.
		pros::screen::print(pros::E_TEXT_MEDIUM, 6, "odom: starting...                 ");
		printf("DBG odom start\n");
		chassis.calibrate(false);
		printf("DBG odom done\n");
		odom_ready = true;
		pros::screen::print(pros::E_TEXT_MEDIUM, 6, "odom: READY                       ");
	} else {
		pros::screen::print(pros::E_TEXT_MEDIUM, 6, "imu: NOT FOUND on port %d          ", imu.get_port());
	}
	printf("DBG imu_ready=%d odom_ready=%d\n", imu_ready, odom_ready);
}

bool auton_imu_ready() { return imu_ready; }

// True once LemLib odometry is running. The TUNE screen and the LemLib auton
// commands (drive_to, face, ...) need it.
bool auton_odom_ready() { return odom_ready; }

void autonomous() {
	auton_cmds_reset();
	auton_status("start");
	estop_arm();   // L1 + R1 stops the run (test runs only, never in a match)

	if (!odom_ready) auton_status("NO IMU: LemLib moves skipped");
	run_routine();   // the routine itself lives in src/routine.cpp

	estop_disarm();
	left_motors.move(0);
	right_motors.move(0);
	mech_stop_all();
	auton_status(estop_triggered() ? "E-STOPPED" : "end");
}
