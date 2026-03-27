# RobotCode2026
### drivetrain
- [ ] fix velocity (not getting high enough, presumably confidence constants are broken)
  - average ds command and current velocity for elsewhere
  - turret velocity 
- [ ] check if drivetrain speed limiter is working for shoot on the fly
- [ ] think about speed limiting while passing
- [ ] test to make sure max speeds are actually max speeds
-----
### turret
- [ ] regs. for passing, yaw
- [ ] ensure turret is correctly rotated relative to center of robot
  - currently, 5x farther away from center than actually, for easy testing
- [ ] turret abs. encoder (accurate zeroing)
- [ ] tune hood PID? never got changed after plates got recut, but might mess up shooting? be careful
- [ ] might need to tune turret PID, check if soft stop is working as intended
  - might want to just limit rotation rate
  - might want to think about stopping handoff while snapping back from softstop, low priority
- [ ] think about soft stop behavior while shooting
--- 
### misc.
- [ ] recalibrate limelights?
- [ ] fix deadzone being high on inputs 
- [ ] how to pass in starting pose for auton
- [ ] think about efficient calibration workflow at comp
- [ ] notes: in lib.rs, currently calling shooter tables instead of passing tables for bottom pass target. easy fix, just change shoot_to -> pass_to
- [ ] debouncer
- [ ] generational merge conflict
