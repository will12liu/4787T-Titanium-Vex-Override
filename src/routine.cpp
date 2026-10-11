#include "main.h"
#include "auton_cmds.h"

// =============================================================================
//                       WRITE YOUR AUTON ROUTINE HERE
//
// Commands run top to bottom, one after another. See include/auton_cmds.h for
// the full list. The main ones:
//
//   move(inches, speed)          drive straight, negative = backwards
//   turn(degrees, speed)         turn from where you face now, + = right, - = left
//   turn_to(degrees, speed)      turn to a heading measured from the auton start
//   drive_time(speed, ms)        drive straight for some time (for pushing things)
//   wait(ms)                     pause
//   move_pid(kP, kD)             change the drive PID for every move() after this
//   turn_pid(kP, kD)             change the turn PID for every turn() after this
//   lift_goal()  lift_down()  lift_wait()                (lift moves don't wait; add lift_wait())
//   claw_in()  claw_out()  claw_up()  claw_drop()          (claw: vertical / horizontal / slightly up / spin to drop)
//   intake(speed)  spin_claw(speed)
//   score()   = lift_goal, lift_wait, claw_out, claw_drop, claw_up in one go
//
// Speed is 0-127 (for drive_time it also sets the direction: negative = backwards).
// You can also set the PID for a single command: move(24, 100, 6.0, 0.3)
//
// LEFT / RIGHT: write the LEFT version. Wrap turns in mirror() and they flip
// automatically for RIGHT:   turn(mirror(-85));
// Or check on_left() / on_right() / is_red() / is_blue() yourself.
//
// START: robot centred on the Toggle, FRONT facing the Toggle wall, a few inches
// from it. Left / right = which side our goal is on, standing at the Toggle wall
// with your back to it looking out into the field.
// =============================================================================

void run_routine() {
	// ---- 1. Hit the Toggle twice -------------------------------------------
	drive_time(-50,400);
	drive_time(127, 900);
	wait(100);        // hit 1: drive in
	drive_time(-50, 400);        // back up
	wait(100);                   // let the Toggle stop moving
	drive_time(127, 900);        // hit 2: drive in
	drive_time(-95, 350);        // back off the Toggle a little

	// ---- 2. Go to our goal and score ---------------------------------------                // lift starts rising while we drive
	turn(mirror(-85), 100);      // turn so the BACK (claw side) faces our goal
	move(-127, 500);              // reverse toward the goal

	// ---- 3. Score: each step on its own, so you can reorder / retime them --
                // start the lift rising
                // wait until it is up
	claw_out();                  // claw vertical -> horizontal
	claw_drop();                 // claw spin (A button) to drop the pin
	claw_up();                   // claw slightly up (B's 2nd press)
}
