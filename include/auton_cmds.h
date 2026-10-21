#pragma once

// Bounded moves leave time for clamping and releasing before the 15 s match cutoff.
constexpr int DRIVE_TIMEOUT_MS = 2000;
constexpr int TURN_TIMEOUT_MS = 1000;

// Inches, clockwise degrees, and motor power [-127, 127]. Driving commands block.
// Field coordinates: start = (0, 0, 0), +x = right, +y = forward.
// Odometry commands are skipped if the IMU is unavailable.
void set_pose(float x, float y, float heading);
void drive_time(int speed, int ms);  // Hold the current heading; negative speed reverses.
// align_first turns toward an axis-aligned target before driving; false starts driving immediately.
void drive_to(float x, float y, int speed = 100, int timeout_ms = DRIVE_TIMEOUT_MS, bool align_first = true);
void back_dist(float inches, int speed = 100, int timeout_ms = DRIVE_TIMEOUT_MS);
enum class ScoreApproachResult { REACHED, STALLED_NEAR_GOAL, FAILED };
// Reverse toward a goal, reducing power over slowdown_in to min_speed near the target.
// Distinguishes a stall in the slowdown zone from a timeout, E-STOP, or missing odometry.
ScoreApproachResult back_dist_score(float inches, int speed, int timeout_ms, float slowdown_in, int min_speed);
void face_heading(float heading, int speed = 100, int timeout_ms = TURN_TIMEOUT_MS);
void wait(int ms);  // Interruptible by E-STOP.

// Lift starts moving in the background. Timed claw commands spin for ms, then stop.
void lift_to_rot(double rotation);
void claw_grab(int ms);  // Same power as R1; ms <= 0 starts it without waiting or stopping.
void claw_release(int ms);  // One R2 toggle; ms <= 0 leaves the selected mode running.

bool on_right();
bool is_blue();
int auton_time_left();  // Remaining time in the 14.8 s planning budget.
bool auton_out_of_time();  // Only true when END_AT_BUDGET is enabled in auton_cmds.cpp.
bool auton_stopped();     // E-STOP or an enabled budget limit.

// Run setup and progress display, used by autonomous.cpp.
void auton_cmds_reset();
void auton_status(const char* message);
const char* auton_last_step();
int auton_last_step_ms();
