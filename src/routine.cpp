
#include "main.h"
#include "auton_cmds.h"
#include <cmath>

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
// 3. pull away, turn, then back down a straight slanted line
	// The turn is FACE_H (front heading). Backing up travels toward FACE_H + 180, so the end point
	// is worked out with trig on that heading, down to y = END_Y. Turn more right = bigger FACE_H.
	const float FACE_H = 326.8f;     // was 309.3, turned right ~17.5 deg
	const float END_Y = -29.9f;
	const float back_dir = (FACE_H + 180.0f) * 3.14159265f / 180.0f;
	const float EXTRA = 12.0f;       // it stopped ~12" short on the field, so aim this much farther down the line
	const float run = (END_Y - (-17.4f)) / std::cos(back_dir) + EXTRA;  // distance along that heading
	const float end_x = 5.4f + run * std::sin(back_dir);                // = x at END_Y
	drive_to(mirror(5.4), -17.4, 100);              // 1) pull away from the goal to x = 5.4
	lift_secs(300,false);
	face_heading(mirror(FACE_H), 100);              // 2) turn
	back_straight(mirror(end_x), -17.4f + run * std::cos(back_dir), 100);   // 3) straight line down that heading
	spin_claw(100);
	claw_down_45();
}
