
#include "main.h"
#include "auton_cmds.h"

void run_routine() {
	// 1. Hit the Toggle twice 
	back_to(0, -8.4, 60);        // back up to y = -8.4
	drive_time(127, 900);//forward
	set_pose(0, 0.7, 0);   
	wait(150); //wait
	back_to(0, -8.4, 60);        // back up to y = -8.4
	drive_time(127, 950);        // hit 2: drive in
	set_pose(0, 0.7, 0);         // pressed flat on the wall: exact spot (measured 10/13), wipes out wheel slip from the hits
	wait(150);

	//2. score preloaded pin (LemLib). Spots measured on the TUNE screen 10/13, LEFT side:
	//   score spot (19.2, -12.9, -87.1)
	back_to(0, -17.4, 60);                 // 1) straight back off the Toggle to the score spot's y
	face_heading(mirror(-87.1), 100);      // 2) turn so the BACK (claw side) faces our goal
	claw_out();                            // claw vertical -> horizontal
	back_to(mirror(21.2), -17.4, 60);        // 3) straight back to the score spot's x
	claw_drop();                           // claw spin (A button) to drop the pin

	//3. claw up while driving to the next spot (-16.9, -35.8, -130.1)
	//claw_up_start();                       // B once: horizontal -> slanted, lifts while we drive
	//face_heading(mirror(-120.1), 100);     // turn to the angle first
	//drive_to(mirror(-16.9), -35.8, 100);   // then drive front first to the spot
}
