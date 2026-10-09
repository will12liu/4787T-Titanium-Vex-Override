#pragma once

// Pre-match autonomous selection: 4 autos (RED/BLUE x LEFT/RIGHT).
// Four buttons on the Brain screen (touch), or controller D-pad LEFT / RIGHT
// for side and X to switch colour. Selection is shown on both.
// Side: standing behind the robot looking into the field, which side our
// alliance goal is on. LEFT = side-wall quadrant, RIGHT = quadrant in front
// of our station.
enum class AutonSide { LEFT, RIGHT };
enum class AutonColor { RED, BLUE };

// Start the background selector task. Call once from initialize().
void auton_selector_start();

AutonSide auton_selector_get();
AutonColor auton_color_get();
const char* auton_side_name(AutonSide side);
const char* auton_color_name(AutonColor color);
