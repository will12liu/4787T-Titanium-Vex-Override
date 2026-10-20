#include "main.h"
#include "robot-config.h"
#include "mechanisms.h"
#include "auton.h"
#include "auton_cmds.h"
#include "estop.h"

// Startup, practice modes, and cleanup. Field steps live in routine.cpp.

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
// driving commands need it.
bool auton_odom_ready() { return odom_ready; }

namespace {
// macro = run from driver control: L1 + R1 cancels it even in a match.
void run_auton(bool macro) {
	auton_cmds_reset();
	auton_status(macro ? "macro start" : "start");
	estop_arm(macro);   // L1 + R1 stops the run (test runs and the macro, never the real auton)

	if (!odom_ready) auton_status("NO IMU: LemLib moves skipped");
	run_routine();   // the routine itself lives in src/routine.cpp

	estop_disarm();
	left_motors.move(0);
	right_motors.move(0);
	mech_stop_all();
	char step[24];   // the last command, saved before the final status message overwrites it
	snprintf(step, sizeof(step), "%s", auton_last_step());
	int step_ms = auton_last_step_ms();
	auton_status(estop_triggered() ? "E-STOPPED" : auton_out_of_time() ? "TIME UP" : "end");
	// Why did it stop, and where? Shown on the controller (line 3) so you don't need a terminal:
	// reason, the last command that started, and when it started.
	pros::Controller(pros::E_CONTROLLER_MASTER).print(2, 0, "%-5s %-9.9s %4.1fs",
	    estop_triggered() ? "ESTOP" : auton_out_of_time() ? "TIME" : "done",
	    step, step_ms / 1000.0);
}
} // namespace

void autonomous() { run_auton(false); }

// Driver-control macro: blocks until finished or L1 + R1 cancels.
void auton_macro() { run_auton(true); }

// Match simulation for practice without a competition switch. Runs autonomous() the way the field
// does: in its own task, and killed hard at 15 s if it is still going. Driver control (the caller)
// starts again afterwards. E-STOP (L1 + R1) still works as a safety, unlike in a real match.
void autonomous_match_sim() {
	constexpr uint32_t AUTON_MS = 15000;
	static volatile bool done;
	done = false;
	pros::Controller(pros::E_CONTROLLER_MASTER).rumble(".");
	pros::Task auton_task([]() {
		autonomous();
		done = true;
	});
	uint32_t start = pros::millis();
	while (!done && pros::millis() - start < AUTON_MS) pros::delay(10);
	if (!done) {   // the field would have killed it here
		auton_task.remove();
		char step[24];
		snprintf(step, sizeof(step), "%s", auton_last_step());
		chassis.cancelAllMotions();   // a killed LemLib move leaves its state set
		left_motors.move(0);
		right_motors.move(0);
		mech_stop_all();
		auton_status("CUT OFF at 15s");
		pros::Controller(pros::E_CONTROLLER_MASTER).print(2, 0, "15s %-18.18s", step);
		printf("DBG 15s cutoff: step=%s lift_pos=%.0f\n", step, lift_pos());
	}
	pros::Controller(pros::E_CONTROLLER_MASTER).rumble("-");   // auton over, driver control
}
