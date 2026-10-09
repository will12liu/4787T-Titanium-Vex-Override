#pragma once

// Emergency stop for auton test runs: hold L1 + R1 (the two upper shoulder
// buttons) together. Cancels the LemLib motion, stops every motor, and the
// running routine bails out at its next step.
//
// Only armed when no competition switch / field control is plugged in. In a
// real match, touching the controller during autonomous is not allowed, and
// the field disables the robot at 15 s anyway.

// Start the watcher task. Call once from initialize().
void estop_start();

// Arm before a routine starts, disarm when it ends. arm() clears a previous stop.
void estop_arm();
void estop_disarm();

// True once L1 + R1 were pressed while armed. Routines check this between steps.
bool estop_triggered();
