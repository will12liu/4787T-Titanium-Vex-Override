#include "main.h"
#include "robot-config.h"
#include "mechanisms.h"
#include "auton_selector.h"
#include "auton.h"
#include "estop.h"
#include <cmath>

// =============================================================================
// Strategy: own our quadrant's Toggle and score the preload. (Manual v1.1)
//
//   1. Push the Toggle to our colour. Each push rolls it one face, and the order
//      is yellow -> red -> blue, so RED pushes once and BLUE pushes twice.
//      Our colour makes the yellow/yellow pin that starts in our neutral goal
//      worth 20, and our preload's yellow half worth 10.
//   2. Score the preload (red/yellow or blue/yellow) in our EMPTY alliance goal:
//      5 + 10 = 15. The neutral goal already holds a pin, and a pin on a pin
//      without a cup scores 0.
//   3. Back off to mid-quadrant: not touching the perimeter (AWP) or the Toggle
//      (a touched Toggle doesn't count).
//
// Start: robot centred on the Toggle (the only open wall, between the two wall
// stations), against the wall, holding the preload in the claw, not touching
// the Toggle.
// =============================================================================

namespace {

bool odom_ready = false;
// Set by auton_drive_test(): run the drive path only, no lift or claw.
bool drive_only = false;

// ---- Time budget -----------------------------------------------------------
constexpr int AUTON_MS = 15000;
// Don't start scoring with less than this left. PLACEHOLDER — measure how long
// face + drive + score_pin() takes on the robot.
constexpr int SCORE_MIN_MS = 5000;

uint32_t auton_start_ms = 0;
int time_left_ms() { return AUTON_MS - (int)(pros::millis() - auton_start_ms); }

// ---- Field points ----------------------------------------------------------
// Inches, in the robot's START frame for the LEFT quadrant:
//   origin  = robot centre at the start, robot against the wall, centred on the Toggle
//   +y      = straight out from the wall into the field
//   +x      = right, standing behind the robot looking into the field
//   heading = LemLib convention: 0 faces +y, clockwise positive
// LEFT  = our alliance goal is on the left  (the side-wall quadrant).
// RIGHT = our alliance goal is on the right (the quadrant in front of our station).
// The two are mirror images, so RIGHT reuses these points with x negated (mx()).
// Red vs blue drives the same path (the field is 180-degree symmetric); only
// the number of Toggle pushes changes.
struct Pt { float x; float y; };

// From the game manual, Appendix A. Same in every quadrant.
constexpr float WALL_TO_GOAL = 23.11f;      // goal centre distance from our wall
constexpr float TOGGLE_TO_GOAL = 23.55f;    // goal centre along the wall from the Toggle centre

constexpr float ROBOT_HALF_LEN = 9.45f;     // robot centre to front bumper (on the wall), measured 24 cm
// PLACEHOLDER — measure on the robot.
constexpr float CLAW_REACH = 14.0f;         // robot centre to where the flipped claw drops the pin

constexpr Pt ALLIANCE_GOAL = {-TOGGLE_TO_GOAL, WALL_TO_GOAL - ROBOT_HALF_LEN};
constexpr Pt LEAVE_WALL    = {0.0f, 8.0f};  // straight out from the Toggle before turning
constexpr Pt END_POINT     = {0.0f, 20.0f}; // mid-quadrant, clear of wall, Toggle and goals

// ---- Toggle ----------------------------------------------------------------
// Which end of the robot hits the Toggle. false = the back hits it (robot starts
// with its back to the wall). true = the front hits it (robot starts facing the wall).
constexpr bool TOGGLE_PUSH_WITH_FRONT = true;
// Each push rolls the Toggle one face: yellow -> red -> blue.
constexpr int TOGGLE_PUSHES_RED = 1;
constexpr int TOGGLE_PUSHES_BLUE = 2;
// Each push: back away for a run-up, then charge at the wall, speeding up the
// closer it gets, and finish at full power so it hits hard enough to roll it.
// Distances come from the drive motor encoders, so no IMU is needed.
// PLACEHOLDER — tune so each hit fully rolls the Toggle over and it seats flat.
constexpr float TOGGLE_RUNUP_IN = 16.0f;         // how far to back up before each charge (more = faster at impact)
constexpr int TOGGLE_RUNUP_VOLTAGE = 70;         // speed backing up
constexpr int TOGGLE_CHARGE_START_VOLTAGE = 127; // speed at the start of the charge... (127 = full power the whole way)
constexpr int TOGGLE_HIT_VOLTAGE = 127;          // ...ramping up to this at the wall
constexpr int TOGGLE_HIT_MS = 500;               // keep full power this long after reaching the wall
constexpr int TOGGLE_CHARGE_TIMEOUT_MS = 1200;   // give up on a charge after this
constexpr float TOGGLE_BACKOFF_IN = 6.0f;        // back off after the last hit so the Toggle can settle
constexpr int TOGGLE_SETTLE_MS = 250;            // let it come to rest before the next charge

constexpr int TOWARD_WALL = TOGGLE_PUSH_WITH_FRONT ? 1 : -1;   // drive sign that moves toward the wall
constexpr float START_HEADING = TOGGLE_PUSH_WITH_FRONT ? 180.0f : 0.0f;

AutonSide side = AutonSide::LEFT;
AutonColor color = AutonColor::RED;
float mx(float x) { return side == AutonSide::RIGHT ? -x : x; }

void status(const char* msg) {
	pros::screen::print(pros::E_TEXT_MEDIUM, 5, "auton %-4s %-5s %5dms %-12s", auton_color_name(color), auton_side_name(side), time_left_ms(), msg);
	printf("DBG auton %s %s t=%d %s\n", auton_color_name(color), auton_side_name(side), time_left_ms(), msg);
}

void stop_drive() {
	left_motors.move(0);
	right_motors.move(0);
}

// Time-based drive, no odom. voltage in [-127, 127]. Stops early on E-STOP.
void drive_for(int voltage, int duration_ms) {
	left_motors.move(voltage);
	right_motors.move(voltage);
	uint32_t end = pros::millis() + duration_ms;
	while (pros::millis() < end && !estop_triggered()) pros::delay(10);
	stop_drive();
}

// Distance driven, in inches, from the drive motor encoders (average of both
// sides, + = forward). Works without the IMU or tracking wheels.
double drive_pos_in() {
	double deg = (left_motors.get_position() + right_motors.get_position()) / 2.0;
#if USE_LEMLIB_ODOM
	return deg / 360.0 * M_PI * drivetrain.wheelDiameter * (drivetrain.rpm / 600.0);
#else
	return deg / 360.0 * M_PI * 3.25;   // 3.25" wheels, direct drive
#endif
}

// Drive straight at a fixed voltage until `inches` are covered (or timeout).
// voltage sign sets the direction.
void drive_dist(int voltage, float inches, int timeout_ms) {
	double p0 = drive_pos_in();
	uint32_t t0 = pros::millis();
	left_motors.move(voltage);
	right_motors.move(voltage);
	while (std::fabs(drive_pos_in() - p0) < inches && pros::millis() - t0 < (uint32_t)timeout_ms &&
	       !estop_triggered()) {
		pros::delay(10);
	}
	stop_drive();
}

// Charge the Toggle from TOGGLE_RUNUP_IN away: speed ramps up with distance
// covered, so it is fastest at the moment of impact. Ends at the wall when the
// run-up distance is used up, the robot stalls against the Toggle, or timeout,
// then holds full power for TOGGLE_HIT_MS to push the roll through.
void charge_toggle() {
	double p0 = drive_pos_in();
	uint32_t t0 = pros::millis();
	while (!estop_triggered() && pros::millis() - t0 < (uint32_t)TOGGLE_CHARGE_TIMEOUT_MS) {
		double covered = std::fabs(drive_pos_in() - p0);
		if (covered >= TOGGLE_RUNUP_IN) break;
		// Stalled against the Toggle/wall early: stop ramping and go to the hit.
		double rpm = (std::fabs(left_motors.get_actual_velocity()) + std::fabs(right_motors.get_actual_velocity())) / 2.0;
		if (pros::millis() - t0 > 200 && covered > 1.0 && rpm < 20) break;
		double frac = covered / TOGGLE_RUNUP_IN;
		int v = TOGGLE_CHARGE_START_VOLTAGE + (int)((TOGGLE_HIT_VOLTAGE - TOGGLE_CHARGE_START_VOLTAGE) * frac);
		left_motors.move(TOWARD_WALL * v);
		right_motors.move(TOWARD_WALL * v);
		pros::delay(10);
	}
	drive_for(TOWARD_WALL * TOGGLE_HIT_VOLTAGE, TOGGLE_HIT_MS);
}

// 1. Hit the Toggle once (red) or twice (blue). Each hit backs up for a run-up
//    first, then charges. Encoder/time based so the fallback auton (no IMU) can
//    use it too. Ends backed off, not touching the Toggle.
void push_toggle() {
	status("toggle");
	int pushes = color == AutonColor::RED ? TOGGLE_PUSHES_RED : TOGGLE_PUSHES_BLUE;
	for (int i = 0; i < pushes && !estop_triggered(); i++) {
		drive_dist(-TOWARD_WALL * TOGGLE_RUNUP_VOLTAGE, TOGGLE_RUNUP_IN, 1000);   // run-up
		pros::delay(TOGGLE_SETTLE_MS);
		charge_toggle();
#if USE_LEMLIB_ODOM
		// Against the wall now, so y is known exactly. Re-zero it: hitting makes
		// the wheels slip and the drive encoders over-count.
		if (odom_ready) {
			lemlib::Pose p = chassis.getPose();
			chassis.setPose(p.x, 0, p.theta);
		}
#endif
	}
	if (!estop_triggered()) drive_dist(-TOWARD_WALL * TOGGLE_RUNUP_VOLTAGE, TOGGLE_BACKOFF_IN, 800);
	pros::delay(TOGGLE_SETTLE_MS);
}

#if USE_LEMLIB_ODOM
// Blocking wrappers (async = false) so the routine reads top to bottom.
void drive_to(Pt p, int timeout_ms, lemlib::MoveToPointParams params = {}) {
	chassis.moveToPoint(mx(p.x), p.y, timeout_ms, params, false);
}
void back_to(Pt p, int timeout_ms, float max_speed = 127) {
	chassis.moveToPoint(mx(p.x), p.y, timeout_ms, {.forwards = false, .maxSpeed = max_speed}, false);
}
void face(Pt p, int timeout_ms) {
	chassis.turnToPoint(mx(p.x), p.y, timeout_ms, {}, false);
}
// Drive toward p and stop `reach` inches short of it, on the line from where we are.
void drive_to_reach(Pt p, float reach, int timeout_ms, float max_speed = 70) {
	lemlib::Pose pose = chassis.getPose();
	float dx = mx(p.x) - pose.x;
	float dy = p.y - pose.y;
	float d = std::hypot(dx, dy);
	if (d <= reach) return;
	float k = (d - reach) / d;
	chassis.moveToPoint(pose.x + dx * k, pose.y + dy * k, timeout_ms, {.maxSpeed = max_speed}, false);
}

// 2. Drive to our alliance goal and place the preload. The lift rises while we
//    drive so it is at height on arrival.
void score_preload() {
	status("to goal");
	if (!drive_only) lift_to(LIFT_ALLIANCE_GOAL_DEG);
	drive_to(LEAVE_WALL, 1000, {.forwards = !TOGGLE_PUSH_WITH_FRONT});
	if (estop_triggered()) return;
	face(ALLIANCE_GOAL, 1000);
	if (estop_triggered()) return;
	drive_to_reach(ALLIANCE_GOAL, CLAW_REACH, 1500);
	if (estop_triggered()) return;
	status("score");
	if (drive_only) {
		pros::delay(1000);   // stand-in for score_pin() so you can see where it stopped
	} else {
		score_pin(LIFT_ALLIANCE_GOAL_DEG);
	}
}

void run_odom_auton() {
	chassis.setPose(0, 0, START_HEADING);
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

	push_toggle();                                     // ~1.1 s red, ~2.2 s blue
	if (!estop_triggered() && time_left_ms() > SCORE_MIN_MS) score_preload(); // ~6 s

	// 3. End clear of the wall, Toggle and goals, with nothing moving.
	status("done");
	if (!estop_triggered() && time_left_ms() > 1000) back_to(END_POINT, 1200);
	chassis.cancelAllMotions();
	stop_drive();
}
#endif

// Fallback when LemLib odom is compiled out or the IMU wasn't found at startup.
// The Toggle pushes are time-based, so do those, then leave the wall so we are
// not touching the perimeter. Keeps the preload in the claw.
void run_fallback_auton() {
	status("FALLBACK");
	// BRAKE so each run-up stops where it should instead of coasting further back.
	left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	push_toggle();
	if (!estop_triggered()) drive_for(-TOWARD_WALL * 70, 600);
	left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);   // driver control default
	right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
}

} // namespace

void auton_init() {
#if USE_LEMLIB_ODOM
	if (odom_ready) return;
	// The IMU can take a moment to enumerate after power-on.
	bool imu_present = false;
	for (int i = 0; i < 20 && !imu_present; i++) {
		imu_present = pros::Device::get_plugged_type(imu.get_port()) == pros::DeviceType::imu;
		if (!imu_present) pros::delay(50);
	}
	if (imu_present) {
		pros::screen::print(pros::E_TEXT_MEDIUM, 6, "odom: calibrating IMU...          ");
		chassis.calibrate();   // blocks until the IMU is calibrated, then starts the odom task
		odom_ready = true;
		pros::screen::print(pros::E_TEXT_MEDIUM, 6, "odom: READY                       ");
	} else {
		pros::screen::print(pros::E_TEXT_MEDIUM, 6, "odom: NO IMU on port %d, FALLBACK ", imu.get_port());
	}
#else
	pros::screen::print(pros::E_TEXT_MEDIUM, 6, "odom: compiled out, FALLBACK      ");
#endif
	printf("DBG odom_ready=%d\n", odom_ready);
}

bool auton_odom_ready() { return odom_ready; }

void autonomous() {
	auton_start_ms = pros::millis();
	side = auton_selector_get();
	color = auton_color_get();
	status("start");
	estop_arm();   // L1 + R1 stops the run (test runs only, never in a match)

#if USE_LEMLIB_ODOM
	if (odom_ready) {
		run_odom_auton();
	} else {
		run_fallback_auton();
	}
#else
	run_fallback_auton();
#endif

	estop_disarm();
	mech_stop_all();
	status(estop_triggered() ? "E-STOPPED" : "end");
}

void auton_drive_test() {
	if (!odom_ready) {
		pros::screen::print(pros::E_TEXT_MEDIUM, 5, "drive test needs the IMU (odom not ready)  ");
		return;
	}
	drive_only = true;
	autonomous();
	drive_only = false;
}
