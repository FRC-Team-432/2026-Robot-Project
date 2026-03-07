# Autonomous Building Blocks Reference

Quick reference for the drive team when composing autonomous routines between matches.

All building blocks are methods on `AutoCommands` (drive blocks) or `Superstructure` (shooter blocks). Compose them in a `Commands.sequence(...)` inside `AutoRoutines`.

---

## Drive Blocks (AutoCommands)

### `driveTo(Pose2d pose)`
Drive to a specific field position and angle. Finishes when the path is complete.
```java
autoCommands.driveTo(Waypoints.SCORE_A)
```

### `driveBackwardUntilTag(LimelightSubsystem limelight, int[] tagIds, double speedMps, double timeoutSeconds)`
Drive backward (robot-relative) until any of the listed AprilTags is visible, or timeout.
```java
autoCommands.driveBackwardUntilTag(limelight, hubTagIds, 0.5, 6.0)
```

### `alignToHubTag(LimelightSubsystem limelight, int centerTagId, double timeoutSeconds)`
Rotate in place to center the robot on a specific hub tag. Uses P-control with camera offset correction and Debouncer confirmation (0.05s).
```java
autoCommands.alignToHubTag(limelight, centerTagId, 5.0)
```

### `spinToFindTag(LimelightSubsystem limelight, int[] tagIds, double spinRateRadS, double timeoutSeconds)`
Spin in place until any of the listed tags is visible, or timeout.
```java
autoCommands.spinToFindTag(limelight, climbTagIds, 0.6, 8.5)
```

### `blindSpin(double spinRateRadS, double durationSeconds)`
Spin for a fixed time with no exit condition. Use before `spinToFindTag` to clear false positives.
```java
autoCommands.blindSpin(0.6, 1.5)
```

### `driveToWithAction(Pose2d targetPose, Command parallelCommand)`
Drive to a position while running another command simultaneously.
```java
autoCommands.driveToWithAction(Waypoints.SCORE_A, superstructure.speakerCloseCommand())
```

### `driveToThenExecute(Pose2d targetPose, Command afterCommand)`
Drive to a position, then run a command after arriving.
```java
autoCommands.driveToThenExecute(Waypoints.SCORE_A, superstructure.shootCommand())
```

### `distanceCommand(double triggerDistance, Pose2d targetPose, Command command)`
Start a command when the robot gets within a distance of a target.
```java
autoCommands.distanceCommand(1.5, Waypoints.SCORE_A, superstructure.speakerCloseCommand())
```

### `resetPose(Pose2d pose)`
Set the robot's starting position on the field.
```java
autoCommands.resetPose(Waypoints.START_CENTER)
```

---

## Utility Blocks (AutoCommands)

### `waitSeconds(double seconds)`
Pause for a duration.
```java
autoCommands.waitSeconds(0.5)
```

### `log(String message)`
Print a message to the console for tracing auto execution.
```java
autoCommands.log("AUTO: Phase 1 complete")
```

---

## Shooter Blocks (Superstructure)

### `spinUpForAreaAndWaitCommand(DoubleSupplier areaSupplier)`
Spin up the shooter at area-based speed and wait until at target speed. Pass `limelight::getTargetArea`.
```java
superstructure.spinUpForAreaAndWaitCommand(limelight::getTargetArea).withTimeout(3.0)
```

### `speakerCloseAndWaitCommand()`
Spin up at fixed close-range speed, wait until ready.
```java
superstructure.speakerCloseAndWaitCommand().withTimeout(3.0)
```

### `shootCommand()`
Fire (run feeder for SHOOT_DURATION_SECONDS). Call after spinner is at speed.
```java
superstructure.shootCommand()
```

### `stowCommand()`
Stop the shooter.
```java
superstructure.stowCommand()
```

---

## Climb Block (Climb)

### `climb.climbUpCommand()`
Run the climb motor upward. Always add `.withTimeout()`.
```java
climb.climbUpCommand().withTimeout(4.0)
```

---

## Example: Full Auto Routine

```java
Commands.sequence(
    autoCommands.resetPose(Waypoints.START_CENTER),
    autoCommands.log("AUTO: driving backward"),
    autoCommands.driveBackwardUntilTag(limelight, hubTagIds, 0.5, 6.0),
    autoCommands.log("AUTO: aligning"),
    autoCommands.alignToHubTag(limelight, centerTagId, 5.0),
    autoCommands.log("AUTO: spinning up"),
    superstructure.spinUpForAreaAndWaitCommand(limelight::getTargetArea).withTimeout(3.0),
    autoCommands.log("AUTO: shooting"),
    superstructure.shootCommand(),
    superstructure.stowCommand(),
    autoCommands.log("AUTO: finding climb"),
    autoCommands.blindSpin(0.6, 1.5),
    autoCommands.spinToFindTag(limelight, climbTagIds, 0.6, 8.5),
    autoCommands.log("AUTO: climbing"),
    climb.climbUpCommand().withTimeout(4.0),
    autoCommands.log("AUTO: Complete")
)
```

**Remember:** Tag IDs must be resolved at enable time inside `Commands.defer()` using `DriverStation.getAlliance()`. See `AutoRoutines.java` for the full pattern.
