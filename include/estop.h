#pragma once

// Hold L1 + R1 to cancel practice runs, tuning motions, or driver-control macros.
// During competition autonomous, the field controls stopping instead.
void estop_start();
void estop_arm(bool force = false);  // Clear the stop; force enables it for a driver macro in a match.
void estop_disarm();
bool estop_triggered();
