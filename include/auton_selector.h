#pragma once

// Pre-match autonomous selection: 4 autos (RED/BLUE x LEFT/RIGHT).
// Four buttons on the Brain screen (touch), or controller D-pad LEFT / RIGHT
// for side and X to switch colour. Selection is shown on both.
// Side: which side our alliance goal is on, as seen when you stand at the Toggle
// wall with your back to it, looking out into the field. Left hand = LEFT.
// Colour: our alliance colour. Not used by the auton yet (it always hits the
// Toggle twice), but selected and shown so the four routines can differ later.
enum class AutonSide { LEFT, RIGHT };
enum class AutonColor { RED, BLUE };

// Start the background selector task. Call once from initialize().
void auton_selector_start();

AutonSide auton_selector_get();
AutonColor auton_color_get();
const char* auton_side_name(AutonSide side);
const char* auton_color_name(AutonColor color);
