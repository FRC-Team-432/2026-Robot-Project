# Autonomous Building Blocks Reference

Quick reference for the drive team when creating new auto routines between matches.

All blocks are methods on `AutoCommands` (drive blocks) or `Superstructure` (shooter blocks).

---

## Drive Blocks (AutoCommands)

### `driveTo(Pose2d pose)`
Drive to a specific field position using path planning. Finishes when the robot arrives.

```java
autoCommands.driveTo(Waypoints.SCORE_A)
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
Trigger a command when the robot gets within a certain distance of a target.

```java
// Start spinning up when 1.5m away from scoring position
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
Drive backward until any of the specified AprilTags is visible, or timeout.

```java
autoCommands.driveBackwardUntilTag(limelight, hubTagIds, 0.5, 6.0)
```

### `alignToHubTag(LimelightSubsystem limelight, int centerTagId, double timeoutSeconds)`
Rotate in place to center on a specific hub tag. Uses P-control with Debouncer confirmation.

```java
autoCommands.alignToHubTag(limelight, centerTagId, 5.0)
```

### `spinToFindTag(LimelightSubsystem limelight, int[] tagIds, double spinRateRadS, double timeoutSeconds)`
Spin in place until any of the specified tags is visible, or timeout.

```java
autoCommands.spinToFindTag(limelight, climbTagIds, 0.6, 8.5)
```

### `blindSpin(double spinRateRadS, double durationSeconds)`
Spin for a fixed duration with no exit condition. Use before `spinToFindTag` to avoid false exits.

```java
autoCommands.blindSpin(0.6, 1.5)
```

---

## Utility Blocks (AutoCommands)

### `waitSeconds(double seconds)`
Wait for a duration. Convenience wrapper.

```java
autoCommands.waitSeconds(0.5)
```

### `log(String message)`
Print a message to the console. Useful for tracing auto execution.

```java
autoCommands.log("AUTO: Phase 1 complete")
```

---

## Shooter Blocks (Superstructure)

### `speakerCloseAndWaitCommand()`
Spin up shooter at fixed speed and wait until at speed.

```java
superstructure.speakerCloseAndWaitCommand().withTimeout(3.0)
```

### `spinUpForAreaAndWaitCommand(DoubleSupplier areaSupplier)`
Spin up shooter at area-based speed (reads tag area once), wait until at speed.

```java
superstructure.spinUpForAreaAndWaitCommand(limelight::getTargetArea).withTimeout(3.0)
```

### `shootCommand()`
Fire the feeder to push a ball through the spinning shooter.

```java
superstructure.shootCommand()
```

### `stowCommand()`
Stop the shooter (safe state).

```java
superstructure.stowCommand()
```

### `teleOpShootWithAreaCommand(DoubleSupplier areaSupplier)`
Teleop: run shooter + feeder with area-based speed. For use with `.whileTrue()`.

```java
joystick.rightTrigger(0.1).whileTrue(superstructure.teleOpShootWithAreaCommand(limelight::getTargetArea))
```

---

## Climb Block (Climb subsystem)

### `climbUpCommand()`
Climb upward while active. Always use with `.withTimeout()`.

```java
climb.climbUpCommand().withTimeout(4.0)
```

---

## Complete Example: Vision Auto Routine

```java
Commands.sequence(
    autoCommands.resetPose(startPose),
    autoCommands.log("Phase 1 - drive backward"),
    autoCommands.driveBackwardUntilTag(limelight, hubTagIds, 0.5, 6.0),
    autoCommands.log("Phase 2 - align to hub"),
    autoCommands.alignToHubTag(limelight, centerTagId, 5.0),
    autoCommands.log("Phase 3 - spin up"),
    superstructure.spinUpForAreaAndWaitCommand(limelight::getTargetArea).withTimeout(3.0),
    autoCommands.log("Phase 4 - shoot"),
    superstructure.shootCommand(),
    superstructure.stowCommand(),
    autoCommands.log("Phase 5 - find climb"),
    autoCommands.blindSpin(0.6, 1.5),
    autoCommands.spinToFindTag(limelight, climbTagIds, 0.6, 8.5),
    autoCommands.log("Phase 6 - climb"),
    climb.climbUpCommand().withTimeout(4.0)
)
```

---

## Tips

- **Always wrap in `Commands.defer()`** when using alliance-dependent tag IDs so they resolve at enable time.
- **Always add `.withTimeout()`** to commands that could hang (vision searches, spin-ups).
- **Use `blindSpin` before `spinToFindTag`** to prevent false-positive exits when the target tags are already in view.
- **Tag IDs are in `VisionConstants`**: `BLUE_HUB_ALL_TAG_IDS`, `RED_HUB_ALL_TAG_IDS`, `BLUE_CLIMB_TAG_IDS`, `RED_CLIMB_TAG_IDS`.
- **Starting positions are in `Waypoints`**: `START_LEFT`, `START_CENTER`, `START_RIGHT`.
