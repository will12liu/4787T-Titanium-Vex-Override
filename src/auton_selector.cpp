#include "auton_selector.h"
#include "main.h"

namespace {

// Brain screen is 480x240. Rows 1-3 (top) are the port check, rows 5-6 are
// auton status. The four buttons live in the bottom strip:
// RED LEFT | RED RIGHT | BLUE LEFT | BLUE RIGHT
constexpr int BTN_Y0 = 150;
constexpr int BTN_Y1 = 230;
constexpr int BTN_W = 110;
constexpr int BTN_GAP = 8;
constexpr int BTN_X0 = 10;

AutonSide selected = AutonSide::LEFT;
AutonColor selected_color = AutonColor::RED;

int btn_x0(int i) { return BTN_X0 + i * (BTN_W + BTN_GAP); }

void draw_button(int i, const char* label, AutonColor color, bool active) {
	int x0 = btn_x0(i);
	int x1 = x0 + BTN_W;
	if (active) {
		pros::screen::set_pen(color == AutonColor::RED ? pros::Color::red : pros::Color::blue);
	} else {
		pros::screen::set_pen(pros::Color::black);
	}
	pros::screen::fill_rect(x0, BTN_Y0, x1, BTN_Y1);
	pros::screen::set_pen(color == AutonColor::RED ? pros::Color::red : pros::Color::blue);
	pros::screen::draw_rect(x0, BTN_Y0, x1, BTN_Y1);
	pros::screen::set_pen(pros::Color::white);
	pros::screen::print(pros::E_TEXT_MEDIUM, x0 + 10, BTN_Y0 + 30, "%s", label);
}

void redraw() {
	const char* labels[4] = {"RED L", "RED R", "BLUE L", "BLUE R"};
	for (int i = 0; i < 4; i++) {
		AutonColor c = i < 2 ? AutonColor::RED : AutonColor::BLUE;
		AutonSide s = i % 2 == 0 ? AutonSide::LEFT : AutonSide::RIGHT;
		draw_button(i, labels[i], c, c == selected_color && s == selected);
	}
	pros::screen::set_pen(pros::Color::white);
}

void selector_task_fn() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	AutonSide last_drawn = selected;
	AutonColor last_drawn_color = selected_color;
	bool first = true;

	while (true) {
		// Touch: only act on a fresh press so a held finger doesn't flicker.
		pros::screen_touch_status_s_t t = pros::screen::touch_status();
		if (t.touch_status == pros::E_TOUCH_PRESSED && t.y >= BTN_Y0 && t.y <= BTN_Y1) {
			for (int i = 0; i < 4; i++) {
				if (t.x >= btn_x0(i) && t.x <= btn_x0(i) + BTN_W) {
					selected_color = i < 2 ? AutonColor::RED : AutonColor::BLUE;
					selected = i % 2 == 0 ? AutonSide::LEFT : AutonSide::RIGHT;
				}
			}
		}

		// Controller D-pad, only while not actually running autonomous.
		if (!pros::competition::is_autonomous()) {
			if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) selected = AutonSide::LEFT;
			if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) selected = AutonSide::RIGHT;
			if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
				selected_color = selected_color == AutonColor::RED ? AutonColor::BLUE : AutonColor::RED;
			}
		}

		if (first || selected != last_drawn || selected_color != last_drawn_color) {
			redraw();
			master.print(1, 0, "AUTON: %-4s %-5s ", auton_color_name(selected_color), auton_side_name(selected));
			last_drawn = selected;
			last_drawn_color = selected_color;
			first = false;
		}
		pros::delay(50);
	}
}

} // namespace

void auton_selector_start() {
	static pros::Task selector_task(selector_task_fn);
}

AutonSide auton_selector_get() { return selected; }
AutonColor auton_color_get() { return selected_color; }

const char* auton_side_name(AutonSide side) {
	return side == AutonSide::LEFT ? "LEFT" : "RIGHT";
}

const char* auton_color_name(AutonColor color) {
	return color == AutonColor::RED ? "RED" : "BLUE";
}
