#pragma once

constexpr int AUTON_DRIVE_POWER = 90;
// Compensate times tuned at power 100, rounding up to the next 10 ms control tick.
constexpr int auton_move_time(int ms_at_100) {
	return (ms_at_100 * 100 + AUTON_DRIVE_POWER * 10 - 1) / (AUTON_DRIVE_POWER * 10) * 10;
}
constexpr int DRIVE_TIMEOUT_MS = auton_move_time(2000);
constexpr int TURN_TIMEOUT_MS = auton_move_time(1000);

// Inches, clockwise degrees, and motor power [-127, 127]. Driving commands block.
// Field coordinates: start = (0, 0, 0), +x = right, +y = forward.
// Odometry commands are skipped if the IMU is unavailable.
void set_pose(float x, float y, float heading);
void drive_time(int speed, int ms);  // Power [-127, 127]; hold heading; negative speed reverses.
// align_first turns toward an axis-aligned target before driving; false starts driving immediately.
void drive_to(float x, float y, int speed = AUTON_DRIVE_POWER, int timeout_ms = DRIVE_TIMEOUT_MS, bool align_first = true);
void back_dist(float inches, int speed = AUTON_DRIVE_POWER, int timeout_ms = DRIVE_TIMEOUT_MS);
// Drive a measured distance forward, holding heading. False on stop/timeout/no odometry.
bool forward_dist(float inches, int speed = AUTON_DRIVE_POWER, int timeout_ms = DRIVE_TIMEOUT_MS);
enum class ScoreApproachResult { REACHED, STALLED_NEAR_GOAL, FAILED };
// Approach a goal, reducing power over slowdown_in to min_speed near the target.
// Distinguishes a stall in the slowdown zone from a timeout, E-STOP, or missing odometry.
ScoreApproachResult back_dist_score(float inches, int speed, int timeout_ms, float slowdown_in, int min_speed);
ScoreApproachResult forward_dist_score(float inches, int speed, int timeout_ms, float slowdown_in, int min_speed);
enum class TurnDirection { AUTO, RIGHT, LEFT };
// AUTO takes the shortest turn; RIGHT/LEFT force the requested direction for mirrored routes.
void face_heading(float heading, int speed = AUTON_DRIVE_POWER, int timeout_ms = TURN_TIMEOUT_MS,
                  TurnDirection direction = TurnDirection::AUTO);
void wait(int ms);  // Interruptible by E-STOP.

// Lift starts moving in the background. Timed claw commands spin for ms, then stop.
void lift_to_rot(double rotation);
void claw_grab(int ms);  // Same power as R1; ms <= 0 starts it without waiting or stopping.
void claw_release(int ms, int power);  // Opening power [0, 127]; ms <= 0 starts without waiting.

bool on_right();
bool is_blue();
int auton_time_left();  // Remaining time in the 14.8 s planning budget.
bool auton_out_of_time();  // Only true when END_AT_BUDGET is enabled in auton_cmds.cpp.
bool auton_stopped();     // E-STOP or an enabled budget limit.

// Run setup and progress display, used by autonomous.cpp.
void auton_cmds_reset();
void auton_status(const char* message);
// Preserve the first failure through cleanup so a failed run is not reported as done.
void auton_fail(const char* message);
const char* auton_failure();
const char* auton_last_step();
int auton_last_step_ms();
