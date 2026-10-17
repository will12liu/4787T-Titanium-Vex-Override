
#include "main.h"
#include "auton_cmds.h"

void run_routine() {
	// 1. Hit the Toggle twice 
	back_to(0.0, -8.4,100);      // back up to y = -8.4 (no settle: the hit below lines it up)
	drive_time(127, 900);//forward
	set_pose(0, 0.7, 0);   
	wait(50); //wait
	back_to(0.0, -8.4, 100);      // back up to y = -8.4
	drive_time(127, 900);        // hit 2: drive in
	set_pose(0, 0.7, 0);         // pressed flat on the wall: exact spot (measured 10/13), wipes out wheel slip from the hits
	wait(50);
// 2. score pin 
	back_to(0, -17.4, 100);                // 1) straight back off the Toggle to the score spot's y
	face_heading(mirror(-87.1), 100);      // 2) turn so the BACK (claw side) faces our goal
	claw_out();                            // claw vertical -> horizontal
	back_to(mirror(21.2), -17.4, 100);     // 3) straight back to the score spot's x
	claw_drop();                           // claw spin (A button) to drop the pin
	claw_up();
	lift_secs(300,true); 
	                           // horizontal -> slanted, waits until it gets there
// 3. go to (21.9, -32.6) facing -50.3: every move is straight along one axis
	drive_to(mirror(13.9), -17.4, 100);    // 1) pull away from the goal (front first) to give room to turn
	back_to(mirror(13.9), -32.6, 100);     // 2) straight along y to the final y
	back_to(mirror(21.9), -32.6, 100);     // 3) straight along x to the final x
	face_heading(mirror(-50.3), 100);      // 4) turn to the final heading
}
