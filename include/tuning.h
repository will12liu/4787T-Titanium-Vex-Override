#pragma once

// PID + sensor tuning mode on the Brain screen.
//
// Tap TUNE (top-right of the Brain) to open it. Only works with no competition
// switch / field control plugged in, and only once the IMU has calibrated.
// While it is open it owns the whole screen and the drivetrain; driver control
// keeps running the other mechanisms but stops driving.
//
// Gains changed here apply live to the chassis until the program restarts.
// Copy the numbers shown into robot-config.cpp to keep them.

// Start the tuning task. Call once at the end of initialize().
void tuning_start();

// True while the tuning screen is open.
bool tuning_active();
