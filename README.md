# Titanium VEX

PROS + LemLib robot program. Start with the lift fully down before powering on.

## Where to edit

| File | Purpose |
| --- | --- |
| `src/routine.cpp` | Autonomous distances, headings, lift/claw timing, and the steps in order |
| `include/auton_cmds.h` | Default drive and turn timeouts |
| `src/auton_cmds.cpp` | Driving commands, heading correction, and autonomous progress display |
| `include/mechanisms.h` | Lift limits, lift controller settings, and claw speeds |
| `src/mechanisms.cpp` | Lift controller, travel limits, and claw toggle behavior |
| `src/opcontrol.cpp` | Controller buttons and driver movement |
| `src/robot-config.cpp` | Motor/sensor ports, drivetrain measurements, and PID gains |
| `src/autonomous.cpp` | IMU startup, practice modes, and cleanup |
| `src/tuning.cpp` | Brain screen for PID and sensor tests |

## Autonomous

`run_routine()` selects the path, then:

1. Starts R1's claw grab and raises the lift while hitting the toggle. Backs up to Y = -6.4
   for the first hit, then to Y = -10.4 after the first hit, 4 inches farther back for the second hit.
   Hits at full power 127 for exactly 900 ms on each hit,
   with no added waits before, between, or after toggle hits.
   A second hit is skipped when time is short. The first goal approach then backs up to Y = -13.3.
2. Approaches the goal, clamps down for 900 ms, and starts opening the claw at full power.
   After 500 ms, pulls away without an alignment turn while keeping the claw open.
   Both scoring approaches gradually reduce drive power from 90 toward 25 over the last 8 inches.
   If progress stops for 250 ms in that zone, the drive stops instead of pushing until its timeout.
3. Drives forward from the first goal to the pull-away point (X = +3.4 or -3.4, Y = -14.4),
   2 inches farther from the goal before turning.
   Blue right / red left turn left 60 degrees; blue left / red right turn right 60 degrees.
   Immediately after the turn, switches to R1's grab power and drives backward while the claw closes.
   The fast leg's distance is calculated from the reverse
   travel heading for 16 inches toward -Y, then extended by 1 inch (2 inches longer than before).
   Lowers the cascade during this fast leg and waits for the bottom limit before slow intake.
   Lowering has a 4-second timeout measured from when it starts, including driving time.
4. Drives backward another 12.75 inches at power 35 while grabbing with R1's claw power.
   Keeps grabbing through the turn and second scoring approach until the final release.
5. Raises the lift to -550 first, waits for it to arrive, pauses for 100 ms, then drives backward
   3.5 inches along the current heading before turning. Adds 130 degrees
   to the measured heading for red left / blue right, or subtracts 135 degrees for red right / blue left.
   These are relative changes from the examples 70 -> 200 and -45 -> -180, not fixed heading targets.
   Immediately drives backward 26 inches after the turn with the pin raised until it reaches the target or stalls in the final
   8-inch slowdown zone. Waits 200 ms, commands the cascade to -250, waits 250 ms,
   and releases at full power for 1500 ms,
   then waits 500 ms before lifting clear and driving away.
   A drive timeout, E-STOP, missing odometry, or a lift wait over 2 seconds ends the step without releasing.
6. After the final release, raises the cascade 60 degrees from its current position, drives forward
   7 inches, lowers fully to the bottom limit, then drives forward another 41 inches along the same
   heading. Each step waits for the previous one; lowering has a 4-second timeout and each drive
   has a 6.67-second timeout.

Distances are inches. Heading 0 faces the starting direction; clockwise is positive.
A more negative lift rotation means a higher lift. Blue currently swaps the selected left/right path.
The left path reflects the right path's x coordinates and intake turn. The turn after slow intake
uses the separate relative heading changes described above.
The intake turn is relative to the heading after pulling away. With the nominal scoring headings,
the right-turn path faces 332.9 degrees and the left-turn path faces 27.1 degrees. Backward intake
moves toward -Y on both paths; its X displacement follows the measured heading and is mirrored.
The backward scoring approach reverses along the heading reached after that relative turn.
Y coordinates, distances, powers, lift targets, and timings are shared.
Toggle pushes use full power 127; other autonomous drive and turn power is capped at 90/127,
including heading correction. The slow intake stays at 35 and the scoring minimum stays at 25. Times tuned at power
100 are increased by approximately 100/90, rounded up to 10 ms, as an initial compensation.
Drive moves time out after 2.23 seconds (3 seconds for the slow intake; 4.45 seconds for each scoring approach).
Turns time out after 1.12 seconds (2.23 seconds for the intake and final scoring turns). The 14.8-second planning budget
controls the optional second hit. The field or practice simulation cuts autonomous off at 15 seconds.
On a practice cutoff, the controller shows the last active step; the terminal also logs the lift position.
Lift position waits use the controller's 14-degree restart band so an already settled lift does not time out
waiting for a tighter tolerance. The wait before slow intake requires reaching the bottom limit.
Early exits show `ERR lift timeout`, `ERR reverse timeout`,
`ERR forward timeout`, or `ERR NO ODOM` on the controller instead of `done`.
Use `pros terminal` to see timestamped steps, drive endpoints, scoring progress, and lift timeout
targets/positions when investigating an unexpected pause. The first goal approach is labeled
`goal approach`; reaching a goal can also be followed by the clamp and release pauses described above.

## Controller

| Input | Action |
| --- | --- |
| Left stick up/down, right stick left/right | Drive and turn; turning scales to 100/127 at full stick |
| D-pad Up | Reverse driving direction |
| L1 / L2 | Raise / lower lift; override the automatic intake target |
| R1 | Toggle claw grab; starting intake sends the cascade to 1 degree |
| R2 or A | Toggle claw release |
| D-pad Left / Right | Select autonomous side outside autonomous |
| X while disabled | Change alliance colour |
| D-pad Down, without field control | Run a 15-second practice autonomous |
| Hold B, press Y | Run the autonomous routine during driver control |
| Hold L1 + R1 | Cancel a practice run, macro, or tuning motion |

The cascade holds its 1-degree intake target until intake stops, claw release starts, or L1/L2 overrides it.
After a manual override, toggle intake off and on to resume the automatic target.

Tap TUNE on the Brain to adjust PID gains and run drive/turn tests without field control.
Copy successful gains into `src/robot-config.cpp` to keep them after restart.

## Build

With the PROS ARM toolchain installed:

```sh
make -j4
```
