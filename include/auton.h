#pragma once

// Call once from initialize(), after mech_init(). Checks the IMU is present
// and calibrates the LemLib chassis. Blocks for ~2-3 s while the IMU calibrates.
void auton_init();

// True if LemLib odom is compiled in, the IMU was found, and calibration ran.
// autonomous() falls back to blind time-based driving when false.
bool auton_odom_ready();

// Runs the selected auton's drive path only: Toggle pushes, drive to the
// alliance goal, back off. No lift or claw. Needs the IMU.
void auton_drive_test();
