#include "tuning.h"
#include "main.h"
#include "robot-config.h"
#include "mechanisms.h"
#include "auton.h"
#include "estop.h"
#include <cmath>
#include <new>

namespace {

bool active = false;

constexpr int LINE_H = 20;
constexpr int BTN_ROW_Y[3] = {114, 156, 198};
constexpr int BTN_H = 38;
constexpr int BTN_W = 92;
constexpr int BTN_PITCH = 96;


constexpr int TUNE_X0 = 405, TUNE_Y0 = 2, TUNE_X1 = 475, TUNE_Y1 = 44;
struct Gains { float kP; float kI; float kD; };
Gains ang = {0, 0, 0};
Gains lat = {0, 0, 0};
bool gains_loaded = false;
bool editing_lat = false;  
constexpr float STEPS[4] = {0.1f, 0.5f, 1.0f, 5.0f};
int step_i = 1;
double left_zero_deg = 0;
double right_zero_deg = 0;
char result1[48] = "tap a test";
char result2[48] = "";
float wrap180(float a) {
	a = std::fmod(a + 180.0f, 360.0f);
	if (a < 0) a += 360.0f;
	return a - 180.0f;
}

double enc_to_in(double deg) {
	return deg / 360.0 * M_PI * drivetrain.wheelDiameter * (drivetrain.rpm / 600.0);
}
void apply_gains() {
	chassis.angularPID.~PID();
	new (&chassis.angularPID) lemlib::PID(ang.kP, ang.kI, ang.kD, angular_controller.windupRange, true);
	chassis.lateralPID.~PID();
	new (&chassis.lateralPID) lemlib::PID(lat.kP, lat.kI, lat.kD, lateral_controller.windupRange, true);
}
void clear_rect(int x0, int y0, int x1, int y1) {
	pros::screen::set_pen(pros::Color::black);
	pros::screen::fill_rect(x0, y0, x1, y1);
	pros::screen::set_pen(pros::Color::white);
}
void draw_btn(int col, int row, const char* label, bool highlight = false) {
	int x0 = 2 + col * BTN_PITCH;
	int y0 = BTN_ROW_Y[row];
	pros::screen::set_pen(highlight ? pros::Color::dark_green : pros::Color::black);
	pros::screen::fill_rect(x0, y0, x0 + BTN_W, y0 + BTN_H);
	pros::screen::set_pen(pros::Color::white);
	pros::screen::draw_rect(x0, y0, x0 + BTN_W, y0 + BTN_H);
	pros::screen::print(pros::E_TEXT_MEDIUM, x0 + 6, y0 + 10, "%s", label);
}
int hit(int x, int y) {
	for (int row = 0; row < 3; row++) {
		if (y < BTN_ROW_Y[row] || y > BTN_ROW_Y[row] + BTN_H) continue;
		for (int col = 0; col < 5; col++) {
			int x0 = 2 + col * BTN_PITCH;
			if (x >= x0 && x <= x0 + BTN_W) return row * 5 + col;
		}
	}
	return -1;
}

void draw_buttons() {
	char step_label[16];
	snprintf(step_label, sizeof(step_label), "STEP %.1f", STEPS[step_i]);
	draw_btn(0, 0, "TURN+90");
	draw_btn(1, 0, "TURN-90");
	draw_btn(2, 0, "DRV+24");
	draw_btn(3, 0, "DRV-24");
	draw_btn(4, 0, "ZERO");
	draw_btn(0, 1, editing_lat ? "EDIT LAT" : "EDIT ANG", true);
	draw_btn(1, 1, "kP -");
	draw_btn(2, 1, "kP +");
	draw_btn(3, 1, "kD -");
	draw_btn(4, 1, "kD +");
	draw_btn(0, 2, "DRIFT 5s");
	draw_btn(1, 2, "kI -");
	draw_btn(2, 2, "kI +");
	draw_btn(3, 2, step_label);
	draw_btn(4, 2, "EXIT");
}
void draw_readouts() {
	lemlib::Pose p = chassis.getPose();
	double l_in = enc_to_in(left_motors.get_position() - left_zero_deg);
	double r_in = enc_to_in(right_motors.get_position() - right_zero_deg);

	clear_rect(0, 0, 479, 5 * LINE_H + 12);
	pros::screen::print(pros::E_TEXT_MEDIUM, 4, 0 * LINE_H, "x %6.1f  y %6.1f  h %6.1f", p.x, p.y, p.theta);
	pros::screen::print(pros::E_TEXT_MEDIUM, 4, 1 * LINE_H, "imu %7.1f L %6.1f R %6.1f", imu.get_rotation(), l_in, r_in);
	pros::screen::print(pros::E_TEXT_MEDIUM, 4, 2 * LINE_H, "%sANG kP %5.2f kI %4.2f kD %5.2f",
	                    editing_lat ? "  " : "> ", ang.kP, ang.kI, ang.kD);
	pros::screen::print(pros::E_TEXT_MEDIUM, 4, 3 * LINE_H, "%sLAT kP %5.2f kI %4.2f kD %5.2f",
	                    editing_lat ? "> " : "  ", lat.kP, lat.kI, lat.kD);
	pros::screen::print(pros::E_TEXT_MEDIUM, 4, 4 * LINE_H, "%s", result1);
	// Both cascade motors (port 20 / port 19): they should read the same while the lift moves.
	std::vector<double> motor_pos = cascade.get_position_all();
	pros::screen::print(pros::E_TEXT_SMALL, 4, 5 * LINE_H, "%s   bat %.0f%%  lift %.0f / %.0f  rot %.0f",
	                    result2, pros::battery::get_capacity(),
	                    motor_pos.size() > 0 ? motor_pos[0] : 0.0, motor_pos.size() > 1 ? motor_pos[1] : 0.0,
	                    ::lift_pos());
}
void countdown(const char* name) {
	snprintf(result1, sizeof(result1), "%s in 1 s, hands off", name);
	result2[0] = '\0';
	draw_readouts();
	pros::delay(1000);
}
void turn_test(float deg) {
	countdown(deg > 0 ? "TURN+90" : "TURN-90");
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	float sign = deg > 0 ? 1.0f : -1.0f;
	float h0 = chassis.getPose().theta;
	uint32_t t0 = pros::millis();
	estop_arm();
	chassis.turnToHeading(h0 + deg, 3000, {}, true);
	pros::delay(20);
	float max_prog = 0;
	while (chassis.isInMotion()) {
		float prog = wrap180(chassis.getPose().theta - h0) * sign;
		if (prog > max_prog) max_prog = prog;
		pros::delay(10);
	}
	uint32_t t = pros::millis() - t0;
	estop_disarm();
	if (estop_triggered()) {
		snprintf(result1, sizeof(result1), "E-STOPPED (L1+R1)");
		result2[0] = '\0';
		return;
	}
	pros::delay(300);  
	float final_prog = wrap180(chassis.getPose().theta - h0) * sign;
	float err = std::fabs(deg) - final_prog;   // + = stopped short, - = past target
	float over = std::fmax(0.0f, max_prog - std::fabs(deg));
	snprintf(result1, sizeof(result1), "TURN err %+.1f ovr %.1f %lums", err, over, (unsigned long)t);
	snprintf(result2, sizeof(result2), "err + = short, - = past. timeout 3000ms");
}
void drive_test(float dist) {
	countdown(dist > 0 ? "DRV+24" : "DRV-24");
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	lemlib::Pose p0 = chassis.getPose();
	float th = p0.theta * (float)M_PI / 180.0f;
	float ux = std::sin(th), uy = std::cos(th);   // 0 deg faces +y, clockwise
	float sign = dist > 0 ? 1.0f : -1.0f;
	uint32_t t0 = pros::millis();
	estop_arm();
	chassis.moveToPoint(p0.x + ux * dist, p0.y + uy * dist, 4000, {.forwards = dist > 0}, true);
	pros::delay(20);
	float max_prog = 0;
	while (chassis.isInMotion()) {
		lemlib::Pose p = chassis.getPose();
		float prog = ((p.x - p0.x) * ux + (p.y - p0.y) * uy) * sign;
		if (prog > max_prog) max_prog = prog;
		pros::delay(10);
	}
	uint32_t t = pros::millis() - t0;
	estop_disarm();
	if (estop_triggered()) {
		snprintf(result1, sizeof(result1), "E-STOPPED (L1+R1)");
		result2[0] = '\0';
		return;
	}
	pros::delay(300);
	lemlib::Pose p = chassis.getPose();
	float along = ((p.x - p0.x) * ux + (p.y - p0.y) * uy) * sign;
	float side = (p.x - p0.x) * uy - (p.y - p0.y) * ux;   // + = drifted right
	float hdg = wrap180(p.theta - p0.theta);
	float err = std::fabs(dist) - along;
	float over = std::fmax(0.0f, max_prog - std::fabs(dist));
	snprintf(result1, sizeof(result1), "DRV err %+.2f ovr %.2f %lums", err, over, (unsigned long)t);
	snprintf(result2, sizeof(result2), "side %+.2fin  hdg %+.1fdeg  (+ = right)", side, hdg);
}

// Robot sitting still: how far the IMU heading wanders 
void drift_test() {
	countdown("DRIFT");
	snprintf(result1, sizeof(result1), "DRIFT: don't touch, 5 s");
	draw_readouts();
	double r0 = imu.get_rotation();
	pros::delay(5000);
	double drift = imu.get_rotation() - r0;
	snprintf(result1, sizeof(result1), "DRIFT %+.2f deg in 5 s", drift);
	snprintf(result2, sizeof(result2), "over ~0.5 deg = bad mount or recalibrate");
}

void zero() {
	chassis.setPose(0, 0, 0);
	left_zero_deg = left_motors.get_position();
	right_zero_deg = right_motors.get_position();
	snprintf(result1, sizeof(result1), "zeroed pose + encoders");
	result2[0] = '\0';
}

void bump(float& value, float delta) {
	value = std::fmax(0.0f, value + delta);
	apply_gains();
}

void handle(int b) {
	Gains& g = editing_lat ? lat : ang;
	float step = STEPS[step_i];
	switch (b) {
		case 0: turn_test(90); break;
		case 1: turn_test(-90); break;
		case 2: drive_test(24); break;
		case 3: drive_test(-24); break;
		case 4: zero(); break;
		case 5: editing_lat = !editing_lat; break;
		case 6: bump(g.kP, -step); break;
		case 7: bump(g.kP, step); break;
		case 8: bump(g.kD, -step); break;
		case 9: bump(g.kD, step); break;
		case 10: drift_test(); break;
		case 11: bump(g.kI, -step / 10.0f); break;
		case 12: bump(g.kI, step / 10.0f); break;
		case 13: step_i = (step_i + 1) % 4; break;
		case 14:
			active = false;
			chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
			clear_rect(0, 0, 479, 239);
			return;
	}
	draw_buttons();
	draw_readouts();
}

const char* tune_block_reason() {
	static char no_imu[16];
	snprintf(no_imu, sizeof(no_imu), "NO IMU %d", imu.get_port());
	if (!auton_odom_ready()) return no_imu;
	if (pros::competition::is_connected()) return "COMP";
	return nullptr;
}

void draw_tune_entry() {
	const char* reason = tune_block_reason();
	pros::screen::set_pen(reason ? pros::Color::dim_gray : pros::Color::dark_green);
	pros::screen::fill_rect(TUNE_X0, TUNE_Y0, TUNE_X1, TUNE_Y1);
	pros::screen::set_pen(pros::Color::white);
	pros::screen::draw_rect(TUNE_X0, TUNE_Y0, TUNE_X1, TUNE_Y1);
	pros::screen::print(pros::E_TEXT_MEDIUM, TUNE_X0 + 6, TUNE_Y0 + 12, "%s", reason ? reason : "TUNE");
}

void tuning_task_fn() {
	int32_t last_release = pros::screen::touch_status().release_count;
	int32_t last_press = pros::screen::touch_status().press_count;
	bool was_down = false;
	uint32_t last_draw = 0;

	while (true) {
		pros::screen_touch_status_s_t t = pros::screen::touch_status();
		// A new finger-down counts as a tap. Checked three ways (state edge, press
		// counter, release counter) so a quick tap between polls isn't missed.
		bool down = t.touch_status == pros::E_TOUCH_PRESSED || t.touch_status == pros::E_TOUCH_HELD;
		bool tapped = (down && !was_down) || t.press_count != last_press || t.release_count != last_release;
		was_down = down;
		last_press = t.press_count;
		last_release = t.release_count;

		if (!active) {
			if (pros::millis() - last_draw > 1000) {
				draw_tune_entry();
				last_draw = pros::millis();
			}
			bool on_tune = t.x >= TUNE_X0 - 25 && t.y <= TUNE_Y1 + 25;
			if (tapped) {
				pros::screen::set_pen(pros::Color::black);
				pros::screen::fill_rect(TUNE_X0 - 25, TUNE_Y1 + 2, 479, TUNE_Y1 + 18);
				pros::screen::set_pen(pros::Color::white);
				pros::screen::print(pros::E_TEXT_SMALL, TUNE_X0 - 20, TUNE_Y1 + 4, "tap %d,%d", t.x, t.y);
				if (on_tune && !auton_odom_ready() && !pros::competition::is_connected()) {
					pros::screen::set_pen(pros::Color::dim_gray);
					pros::screen::fill_rect(TUNE_X0, TUNE_Y0, TUNE_X1, TUNE_Y1);
					pros::screen::set_pen(pros::Color::white);
					pros::screen::print(pros::E_TEXT_MEDIUM, TUNE_X0 + 6, TUNE_Y0 + 12, "CAL...");
					auton_init();
				}
				if (on_tune) draw_tune_entry();
			}
			if (tapped && on_tune && !tune_block_reason()) {
				if (!gains_loaded) {
					ang = {angular_controller.kP, angular_controller.kI, angular_controller.kD};
					lat = {lateral_controller.kP, lateral_controller.kI, lateral_controller.kD};
					gains_loaded = true;
				}
				active = true;
				clear_rect(0, 0, 479, 239);
				draw_buttons();
				draw_readouts();
				pros::delay(300);
				pros::screen_touch_status_s_t now = pros::screen::touch_status();
				last_press = now.press_count;
				last_release = now.release_count;
				was_down = now.touch_status == pros::E_TOUCH_PRESSED || now.touch_status == pros::E_TOUCH_HELD;
			}
		} else {
			if (tapped) {
				int b = hit(t.x, t.y);
				if (b >= 0) handle(b);
				pros::screen_touch_status_s_t now = pros::screen::touch_status();
				last_press = now.press_count;
				last_release = now.release_count;
				was_down = now.touch_status == pros::E_TOUCH_PRESSED || now.touch_status == pros::E_TOUCH_HELD;
			}
			if (active && pros::millis() - last_draw > 100) {
				draw_readouts();
				last_draw = pros::millis();
			}
		}
		pros::delay(20);
	}
}


} // namespace

void tuning_start() {
	static pros::Task tuning_task(tuning_task_fn);
}

bool tuning_active() { return active; }
