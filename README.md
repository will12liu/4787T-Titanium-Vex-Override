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

1. Raises the lift while hitting the toggle. A second hit is skipped when time is short.
2. Approaches the goal, clamps down for 900 ms, and starts opening the claw with driver control's release power (85).
   Keeps opening through the 500 ms pause, then pulls away without raising the cascade.
   Pulls away without an alignment turn, returns the cascade to -70 degrees and waits for it to arrive,
   then starts R1's grab power and continues toward intake.
   Both scoring approaches gradually reduce drive power from 100 toward 25 over the last 8 inches.
   If progress stops for 250 ms in that zone, the drive stops instead of pushing until its timeout.
3. After pulling away and reaching intake height, turns and completes the original backward drive while grabbing.
4. Reverses another 11.75 inches slowly at power 35 while grabbing with R1's claw power.
   Keeps grabbing through the turn and second scoring approach until the final release.
5. Raises the lift to -450 first, waits for it to arrive, pauses for 100 ms, then turns 133 degrees
   (left on the right path, right on the left path)
   and drives backward 23 inches with the pin raised,
   then commands -365 once it reaches the target or stalls in the final 8-inch slowdown zone.
   Waits for -365, pauses for 100 ms, and starts continuous release at driver control's power.
   Waits 1500 ms before autonomous cleanup stops the mechanisms.
   A drive timeout, E-STOP, missing odometry, or a lift wait over 2 seconds ends the step without releasing.

Distances are inches. Heading 0 faces the starting direction; clockwise is positive.
A more negative lift rotation means a higher lift. Blue currently swaps the selected left/right path.
Drive moves time out after 2 seconds (3 seconds for the slow intake; 4 seconds for each scoring approach).
Turns time out after 1 second (2 seconds for the final scoring turn). The 14.8-second planning budget
controls the optional second hit. The field or practice simulation cuts autonomous off at 15 seconds.
On a practice cutoff, the controller shows the last active step; the terminal also logs the lift position.

## Controller

| Input | Action |
| --- | --- |
| Left stick up/down, right stick left/right | Drive and turn |
| D-pad Up | Reverse driving direction |
| L1 / L2 | Raise / lower lift |
| R1 | Toggle claw grab |
| R2 or A | Toggle claw release |
| D-pad Left / Right | Select autonomous side outside autonomous |
| X while disabled | Change alliance colour |
| D-pad Down, without field control | Run a 15-second practice autonomous |
| Hold B, press Y | Run the autonomous routine during driver control |
| Hold L1 + R1 | Cancel a practice run, macro, or tuning motion |

Tap TUNE on the Brain to adjust PID gains and run drive/turn tests without field control.
Copy successful gains into `src/robot-config.cpp` to keep them after restart.

## Build

With the PROS ARM toolchain installed:

```sh
make -j4
```
