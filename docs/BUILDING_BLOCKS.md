# Autonomous Building Blocks Reference

Quick reference for the drive team when creating new auto routines between matches.

All building blocks are methods on `AutoCommands` (driving) or `Superstructure` (shooting).

---

## Drive Blocks (AutoCommands)

### `driveTo(Pose2d pose)`
Drive to a specific field position using path planning. Finishes when the path is complete.

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
Start a command when the robot gets within a distance of a target.

```java
Commands.parallel(
    autoCommands.driveTo(shootPose),
    autoCommands.distanceCommand(1.5, shootPose, superstructure.speakerCloseCommand())
)
```

### `resetPose(Pose2d pose)`
Reset the robot's odometry to a known position. Use at the start of every auto.

```java
autoCommands.resetPose(Waypoints.START_CENTER)
```

---

## Vision-Drive Blocks (AutoCommands)

### `driveBackwardUntilTag(LimelightSubsystem limelight, int[] tagIds, double speedMps, double timeoutSeconds)`
Drive backward until any of the specified AprilTags is visible, or timeout expires.

```java
autoCommands.driveBackwardUntilTag(limelight, hubTagIds, 0.5, 6.0)
```

### `alignToHubTag(LimelightSubsystem limelight, int centerTagId, double timeoutSeconds)`
Rotate in place to center on the hub tag. Uses P-control with camera offset correction. Exits when centered for 0.05s.

```java
autoCommands.alignToHubTag(limelight, centerTagId, 5.0)
```

### `spinToFindTag(LimelightSubsystem limelight, int[] tagIds, double spinRateRadS, double timeoutSeconds)`
Spin in place until any of the specified tags is visible, or timeout expires.

```java
autoCommands.spinToFindTag(limelight, climbTagIds, 0.6, 8.5)
```

### `blindSpin(double spinRateRadS, double durationSeconds)`
Spin for a fixed time with no exit condition. Use before `spinToFindTag` to avoid false exits.

```java
autoCommands.blindSpin(0.6, 1.5)
```

---

## Utility Blocks (AutoCommands)

### `waitSeconds(double seconds)`
Wait for a duration.

```java
autoCommands.waitSeconds(0.5)
```

### `log(String message)`
Print a message to the console for tracing auto execution.

```java
autoCommands.log("AUTO: Phase 1 starting")
```

---

## Shooting Blocks (Superstructure)

### `speakerCloseAndWaitCommand()`
Spin up shooter at fixed speed and wait until at target speed.

### `spinUpForAreaAndWaitCommand(DoubleSupplier areaSupplier)`
Spin up shooter at area-based speed (reads tag area once), wait until at speed.

```java
superstructure.spinUpForAreaAndWaitCommand(limelight::getTargetArea).withTimeout(3.0)
```

### `shootCommand()`
Fire the feeder to push a ball through the shooter. Use after spin-up.

### `stowCommand()`
Stop the shooter.

### `teleOpShootWithAreaCommand(DoubleSupplier areaSupplier)`
Teleop: run shooter at area-based speed + feeder. For `.whileTrue()` binding.

---

## Climb Blocks (Climb)

### `climb.climbUpCommand().withTimeout(4.0)`
Climb up for a fixed duration. Always add a timeout.

---

## Example: Full Auto Routine

```java
Commands.sequence(
    autoCommands.resetPose(Waypoints.START_CENTER),
    autoCommands.log("Phase 1 - drive backward"),
    autoCommands.driveBackwardUntilTag(limelight, hubTagIds, 0.5, 6.0),
    autoCommands.log("Phase 2 - align"),
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
    climb.climbUpCommand().withTimeout(4.0),
    autoCommands.log("AUTO: Complete")
)
```

**Remember:** Tag IDs must be resolved inside `Commands.defer()` at enable time so the correct alliance is used. See `AutoRoutines.java` for the full pattern.
