#pragma once

// Call once from initialize(), after mech_init(). Checks the IMU is present
// and calibrates the LemLib chassis. Blocks for ~2-3 s while the IMU calibrates.
// Safe to call again later (e.g. after plugging the IMU back in); does nothing
// once odom is ready.
void auton_init();

// True once the IMU was found and LemLib odom started. The LemLib auton
// commands (drive_to, face, ...) are skipped when false.
bool auton_odom_ready();
// True once the IMU was found and calibrated (drive_time uses it to drive straight).
bool auton_imu_ready();

// The routine itself, written in src/routine.cpp.
void run_routine();
