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
void run_routine();      // picks routine_right() or routine_left() from the selector
void routine_right();
void routine_left();

// Practice a real match without a competition switch: runs run_routine() in its own task for
// 15 s (hard cut-off, like the field), then returns so driver control continues.
void autonomous_match_sim();

// Run the routine as a macro during driver control (opcontrol). Blocks until it ends; L1 + R1 cancels.
// The robot must be in the routine's start position, since the routine starts from pose (0, 0, 0).
void auton_macro();
