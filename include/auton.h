#pragma once

// Call once from initialize(), after mech_init(). Checks the IMU is present
// and calibrates the LemLib chassis. Blocks for ~2-3 s while the IMU calibrates.
// Safe to call again later (e.g. after plugging the IMU back in); does nothing
// once odom is ready.
void auton_init();

// True if LemLib odom is compiled in, the IMU was found, and calibration ran.
// autonomous() falls back to blind time-based driving when false.
bool auton_odom_ready();
// True once the IMU was found and calibrated (what move/turn/drive_time need).
bool auton_imu_ready();

// Runs the selected auton's drive path only: Toggle pushes, drive to the
// alliance goal, back off. No lift or claw. Needs the IMU.
void auton_drive_test();

// The routine itself, written in src/routine.cpp.
void run_routine();
