Control mapping (teleop)

Overview
- This file lists which controller inputs map to which functionality and what runs continuously during teleop.

Controllers
- driverXbox (CommandXboxController) port: 0
- firstXbox (XboxController) port: 0
- secondXbox (XboxController) port: 1

Note: The current code constructs both `driverXbox` and `firstXbox` on port 0. That's probably accidental and should be corrected so each physical controller has a unique USB index (e.g., driverXbox=0, firstXbox=1, secondXbox=2).

Mappings (as implemented in code)
- Drive (field-oriented): controlled by `driverXbox` analog sticks via `SwerveInputStream` (left stick for translation, right stick for rotation). Drive default command: `drivebase.driveFieldOriented(...)`.
- Reset Limelight: `driverXbox.a()` — when pressed, runs `vision.hardReset("limelight")`.
- Zero gyro (no AprilTags): `driverXbox.start()` — runs `drivebase.zeroNoAprilTagsGyro()` once.

ManualControls (firstXbox & secondXbox)
- BallOut (shoot/feeder): `firstXbox.getRightTriggerAxis() > 0.5` — used by `SnotmCommand` and `FeederCommand` to run the feeder while true.
- Intake extend/roller: `firstXbox.getRightBumper()` — used by intake commands (GroundIntake) to run intake out.
- Climb up/down: `secondXbox.getRightY()` axis with threshold (>0.7 or < -0.7)
- Gyro zero (alternate): `secondXbox.getStartButton()` — exposed via `ManualControls.zeroNoAprilTagsGyro()` but RobotContainer uses driverXbox.start() for gyro zero.

Default/constantly-running behaviors during teleop
- Drivebase default command: `drivebase.driveFieldOriented(driveAngularVelocity)` — runs continuously and reads controller axis to drive.
- Snotm default command: `SnotmCommand` — calls `snotm.ShootBall()` (in periodic) and uses `controls.BallOut()` to feed. This computes aim/launch RPM and calls swerve align rotation and hood/turret goals.
- Feeder default command: `FeederCommand` — reads `controls.BallOut()` and runs/stops feeder motor accordingly.
- Flywheel subsystem periodic telemetry: reports current RPM, target RPM, whether at target, voltage, and current draw.
- SwerveSubsystem periodic/path-following and alignment commands: swerve odometry and path routines run as needed; `alignRotationCommand(...)` directly drives rotation output when called.
- Turret and Hood subsystems both run their control loops (MotionMagic/PositionVoltage) from their `periodic()`/setControl() calls to move toward set goals.

Notes & debugging hooks
- SmartDashboard telemetry added (Snotm, Sotm, Flywheel, Feeder) to show:
  - Flywheel: current RPM, target RPM, atTarget, voltage, current draw.
  - Feeder: boolean `Feeder/Is Feeding`.
  - Snotm/Sotm: robotPoseX/Y, dx/dy, robotHeadingDeg, launchAngle, launchRPM, finalRPM, finalVelocity, turretShootAngle, robotVx/Vy.

Suggested improvements
- Assign unique controller ports (avoid two controllers constructed on port 0).
- Consolidate feeder control so only one command is responsible for the feeder motor (either via default command or by SnotmCommand delegating to FeederCommand).
- Tune CAN IDs to match actual wiring and update `Constants.java` accordingly.

If you want, I can:
- Re-assign controller ports in `RobotContainer` to unique indices.
- Move more telemetry to Shuffleboard tabs with clearer labels.
- Make the feeder owned exclusively by `FeederCommand` (remove feeder calls from SnotmCommand), or vice versa—your choice.
