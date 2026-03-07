# Autonomous Building Blocks Reference

Quick reference for the drive team when creating new auto routines between matches.

All blocks are methods on `AutoCommands` (for drive/vision) or `Superstructure` (for shooter/feeder).

---

## Drive Blocks (AutoCommands)

### `driveTo(Pose2d pose)`
Drive to a specific field position using path planning. Finishes when the robot arrives.
```java
autoCommands.driveTo(Waypoints.SCORE_A)
```

### `driveToWithAction(Pose2d targetPose, Command parallelCommand)`
Drive to a position while running another command at the same time.
```java
autoCommands.driveToWithAction(Waypoints.SCORE_A, superstructure.speakerCloseCommand())
```

### `driveToThenExecute(Pose2d targetPose, Command afterCommand)`
Drive to a position, then run a command after arriving.
```java
autoCommands.driveToThenExecute(Waypoints.SCORE_A, superstructure.shootCommand())
```

### `distanceCommand(double triggerDistance, Pose2d targetPose, Command command)`
Start a command when the robot gets within a certain distance of a target.
```java
// Start spinning up shooter when 1.5m away from scoring position
Commands.parallel(
    autoCommands.driveTo(Waypoints.SCORE_A),
    autoCommands.distanceCommand(1.5, Waypoints.SCORE_A, superstructure.speakerCloseCommand())
)
```

### `resetPose(Pose2d pose)`
Set the robot's starting position on the field. Always call this first in auto.
```java
autoCommands.resetPose(Waypoints.START_CENTER)
```

---

## Vision-Drive Blocks (AutoCommands)

### `driveBackwardUntilTag(LimelightSubsystem limelight, int[] tagIds, double speedMps, double timeoutSeconds)`
Drive backward until the camera sees one of the specified AprilTags.
```java
autoCommands.driveBackwardUntilTag(limelight, hubTagIds, 0.5, 6.0)
```

### `alignToHubTag(LimelightSubsystem limelight, int centerTagId, double timeoutSeconds)`
Rotate in place to center on a specific hub tag. Uses P-control with camera offset correction. Exits when centered within 3 degrees for 0.05 seconds.
```java
autoCommands.alignToHubTag(limelight, centerTagId, 5.0)
```

### `spinToFindTag(LimelightSubsystem limelight, int[] tagIds, double spinRateRadS, double timeoutSeconds)`
Spin in place until the camera sees one of the specified tags.
```java
autoCommands.spinToFindTag(limelight, climbTagIds, 0.6, 8.5)
```

### `blindSpin(double spinRateRadS, double durationSeconds)`
Spin for a fixed time with no exit condition. Use before `spinToFindTag` to rotate away from a known area and prevent false exits.
```java
autoCommands.blindSpin(0.6, 1.5)
```

---

## Utility Blocks (AutoCommands)

### `waitSeconds(double seconds)`
Pause for a set amount of time.
```java
autoCommands.waitSeconds(0.5)
```

### `log(String message)`
Print a message to the console for debugging.
```java
autoCommands.log("AUTO: Phase 1 complete")
```

---

## Shooter Blocks (Superstructure)

### `teleOpShootWithAreaCommand(DoubleSupplier areaSupplier)`
Teleop: run shooter at area-based speed + feeder. Both stop when released.
```java
superstructure.teleOpShootWithAreaCommand(limelight::getTargetArea)
```

### `spinUpForAreaAndWaitCommand(DoubleSupplier areaSupplier)`
Auto: spin up shooter based on tag area, wait until at speed.
```java
superstructure.spinUpForAreaAndWaitCommand(limelight::getTargetArea).withTimeout(3.0)
```

### `speakerCloseAndWaitCommand()`
Spin up the shooter at fixed speed, wait until ready. Fallback if area-based is not working.
```java
superstructure.speakerCloseAndWaitCommand().withTimeout(3.0)
```

### `shootCommand()`
Fire a game piece (runs feeder for 0.5s). Call after spin-up is complete.
```java
superstructure.shootCommand()
```

### `stowCommand()`
Stop the shooter (safe state).
```java
superstructure.stowCommand()
```

### `teleOpShootCommand()`
Teleop: run shooter at fixed speed + feeder while held. Fallback if area-based is not working.
```java
superstructure.teleOpShootCommand()
```

---

## Example: Full Auto Routine

```java
Commands.sequence(
    autoCommands.resetPose(Waypoints.START_CENTER),
    autoCommands.log("Driving backward to find hub"),
    autoCommands.driveBackwardUntilTag(limelight, hubTagIds, 0.5, 6.0),
    autoCommands.log("Aligning to hub"),
    autoCommands.alignToHubTag(limelight, centerTagId, 5.0),
    autoCommands.log("Spinning up shooter"),
    superstructure.spinUpForAreaAndWaitCommand(limelight::getTargetArea).withTimeout(3.0),
    autoCommands.log("Shooting"),
    superstructure.shootCommand(),
    superstructure.stowCommand(),
    autoCommands.log("Finding climb tags"),
    autoCommands.blindSpin(0.6, 1.5),
    autoCommands.spinToFindTag(limelight, climbTagIds, 0.6, 8.5),
    autoCommands.log("Climbing"),
    climb.climbUpCommand().withTimeout(4.0),
    autoCommands.log("Done!")
)
```

---

## Tips

- **Always wrap in `Commands.defer()`** when using alliance-specific tag IDs so they resolve at enable time, not at construction.
- **Always add `.withTimeout()`** to any command that could hang (vision search, spin-up, climb).
- **Use `blindSpin` before `spinToFindTag`** to avoid instant exits when tags from the previous phase are still visible.
- **Tag IDs are in `VisionConstants`** — check there for blue/red hub and climb tag arrays.
- **Starting poses are in `Waypoints`** — START_LEFT, START_CENTER, START_RIGHT.
