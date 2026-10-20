#pragma once

// Calibrate the IMU and start odometry. Does nothing once ready; can retry a missing IMU.
void auton_init();
bool auton_odom_ready();
bool auton_imu_ready();

// The field steps and settings live in routine.cpp.
void run_routine();

// D-pad Down: practice autonomous with a hard 15 s cutoff, then return to driving.
void autonomous_match_sim();

// B + Y: run the routine during driver control, until finished or L1 + R1 cancels it.
// Place the robot at the autonomous start position first.
void auton_macro();
