# Autonomous Building Blocks Reference

Quick reference for the drive team when creating new auto routines between matches.

---

## Drive Blocks (AutoCommands)

### `driveTo(Pose2d pose)`
Drive to a specific field position using path planning. Finishes when the robot arrives.
```java
autoCommands.driveTo(Waypoints.SCORE_A)
```

### `driveBackwardUntilTag(LimelightSubsystem limelight, int[] tagIds, double speedMps, double timeoutSeconds)`
Drive backward until any of the specified AprilTags is visible, or timeout expires.
```java
autoCommands.driveBackwardUntilTag(limelight, hubTagIds, 0.5, 6.0)
```

### `alignToHubTag(LimelightSubsystem limelight, int centerTagId, double timeoutSeconds)`
Rotate in place to center on a specific hub tag. Uses three-way logic: P-control on center tag, directional spin on side tags, fallback spin if no tag. Exits when centered for 0.05s.
```java
autoCommands.alignToHubTag(limelight, centerTagId, 5.0)
```

### `spinToFindTag(LimelightSubsystem limelight, int[] tagIds, double spinRateRadS, double timeoutSeconds)`
Spin in place until any of the specified tags is visible, or timeout.
```java
autoCommands.spinToFindTag(limelight, climbTagIds, 0.6, 8.5)
```

### `blindSpin(double spinRateRadS, double durationSeconds)`
Spin for a fixed duration with no exit condition. Use before `spinToFindTag` to rotate away from a known area.
```java
autoCommands.blindSpin(0.6, 1.5)
```

### `resetPose(Pose2d pose)`
Reset the robot's odometry to a known position. Always call at the start of auto.
```java
autoCommands.resetPose(Waypoints.START_CENTER)
```

### `driveToWithAction(Pose2d targetPose, Command parallelCommand)`
Drive to a location while running another command in parallel.
```java
autoCommands.driveToWithAction(Waypoints.SCORE_A, superstructure.speakerCloseCommand())
```

### `driveToThenExecute(Pose2d targetPose, Command afterCommand)`
Drive to a location, then run a command after arriving.
```java
autoCommands.driveToThenExecute(Waypoints.SCORE_A, superstructure.shootCommand())
```

### `distanceCommand(double triggerDistance, Pose2d targetPose, Command command)`
Start a command when the robot gets within a certain distance of a target.
```java
autoCommands.distanceCommand(1.5, Waypoints.SCORE_A, superstructure.speakerCloseCommand())
```

---

## Utility Blocks (AutoCommands)

### `waitSeconds(double seconds)`
Wait for a specified duration.
```java
autoCommands.waitSeconds(1.0)
```

### `log(String message)`
Print a message to the console for tracing auto execution.
```java
autoCommands.log("AUTO: Phase 1 complete")
```

---

## Shooter Blocks (Superstructure)

### `teleOpShootWithAreaCommand(DoubleSupplier areaSupplier)`
Teleop: run shooter at area-based speed + feeder simultaneously. Stops on release.
```java
superstructure.teleOpShootWithAreaCommand(limelight::getTargetArea)
```

### `spinUpForAreaAndWaitCommand(DoubleSupplier areaSupplier)`
Auto: spin up shooter based on tag area, wait until at speed.
```java
superstructure.spinUpForAreaAndWaitCommand(limelight::getTargetArea).withTimeout(3.0)
```

### `speakerCloseAndWaitCommand()`
Spin up shooter at fixed speed, wait until at speed (fallback).
```java
superstructure.speakerCloseAndWaitCommand().withTimeout(3.0)
```

### `shootCommand()`
Fire: run feeder for SHOOT_DURATION_SECONDS to push ball through shooter.
```java
superstructure.shootCommand()
```

### `stowCommand()`
Stop the shooter (safe state).
```java
superstructure.stowCommand()
```

---

## Climb Block

### `climb.climbUpCommand()`
Run climb motor upward. Always add `.withTimeout()`.
```java
climb.climbUpCommand().withTimeout(4.0)
```

---

## Example: Full Auto Routine

```java
Commands.sequence(
    autoCommands.resetPose(Waypoints.START_CENTER),
    autoCommands.log("Driving backward"),
    autoCommands.driveBackwardUntilTag(limelight, hubTagIds, 0.5, 6.0),
    autoCommands.log("Aligning"),
    autoCommands.alignToHubTag(limelight, centerTagId, 5.0),
    autoCommands.log("Spinning up"),
    superstructure.spinUpForAreaAndWaitCommand(limelight::getTargetArea).withTimeout(3.0),
    autoCommands.log("Shooting"),
    superstructure.shootCommand(),
    superstructure.stowCommand(),
    autoCommands.log("Finding climb"),
    autoCommands.blindSpin(0.6, 1.5),
    autoCommands.spinToFindTag(limelight, climbTagIds, 0.6, 8.5),
    autoCommands.log("Climbing"),
    climb.climbUpCommand().withTimeout(4.0),
    autoCommands.log("Done!")
)
```

**Remember:** Tag IDs must be resolved inside `Commands.defer()` at enable time, not at construction time. See `AutoRoutines.java` for the pattern.
