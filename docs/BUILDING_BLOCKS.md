# Autonomous Building Blocks Reference

Quick reference for all available autonomous building blocks. Use these to compose new auto routines between matches.

---

## Drive Commands (AutoCommands)

### `driveTo(Pose2d pose)`
Drive to a specific field position using motion-profiled path following. Finishes when the robot arrives.

```java
autoCommands.driveTo(Waypoints.SCORE_A)
```

### `driveToWithAction(Pose2d targetPose, Command parallelCommand)`
Drive to a location while running another command in parallel. Both start at the same time.

```java
autoCommands.driveToWithAction(Waypoints.SCORE_A, superstructure.speakerCloseCommand())
```

### `driveToThenExecute(Pose2d targetPose, Command afterCommand)`
Drive to a location, then run a command after arriving.

```java
autoCommands.driveToThenExecute(Waypoints.SCORE_A, superstructure.shootCommand())
```

### `distanceCommand(double triggerDistance, Pose2d targetPose, Command command)`
Start a command when the robot gets within a distance of a target. Use in parallel with driveTo.

```java
Commands.parallel(
    autoCommands.driveTo(Waypoints.SCORE_A),
    autoCommands.distanceCommand(1.5, Waypoints.SCORE_A, superstructure.speakerCloseCommand())
)
```

### `resetPose(Pose2d pose)`
Reset the robot's odometry to a known starting position. Always call at the start of auto.

```java
autoCommands.resetPose(Waypoints.START_CENTER)
```

---

## Vision-Drive Blocks (AutoCommands)

### `driveBackwardUntilTag(LimelightSubsystem limelight, int[] tagIds, double speedMps, double timeoutSeconds)`
Drive backward (robot-relative) until any of the specified AprilTags is visible, or timeout.

```java
autoCommands.driveBackwardUntilTag(limelight, hubTagIds, 0.5, 6.0)
```

### `alignToHubTag(LimelightSubsystem limelight, int centerTagId, double timeoutSeconds)`
Rotate in place to center on a specific hub tag. Uses three-way logic: P-control on center tag, directional spin on side tags, fallback spin if no tag visible. Exits when centered within 3 degrees for 0.05s.

```java
autoCommands.alignToHubTag(limelight, centerTagId, 5.0)
```

### `spinToFindTag(LimelightSubsystem limelight, int[] tagIds, double spinRateRadS, double timeoutSeconds)`
Spin in place until any of the specified tags is visible, or timeout.

```java
autoCommands.spinToFindTag(limelight, climbTagIds, 0.6, 8.5)
```

### `blindSpin(double spinRateRadS, double durationSeconds)`
Spin for a fixed duration with no exit condition. Use before spinToFindTag to avoid false-positive exits.

```java
autoCommands.blindSpin(0.6, 1.5)
```

---

## Utility Blocks (AutoCommands)

### `waitSeconds(double seconds)`
Wait for a duration. Convenience wrapper around Commands.waitSeconds.

```java
autoCommands.waitSeconds(0.5)
```

### `log(String message)`
Print a message to the console. Useful for tracing auto execution in logs.

```java
autoCommands.log("AUTO: Phase 1 complete")
```

---

## Shooter Commands (Superstructure)

### `speakerCloseAndWaitCommand()`
Spin up shooter at fixed speed, wait until at target speed.

### `spinUpForAreaAndWaitCommand(DoubleSupplier areaSupplier)`
Spin up shooter at area-based speed (reads tag area once), wait until at target speed.

```java
superstructure.spinUpForAreaAndWaitCommand(limelight::getTargetArea).withTimeout(3.0)
```

### `shootCommand()`
Run feeder for 0.5s to fire a ball through the already-spinning shooter.

### `stowCommand()`
Stop the shooter motors.

### `teleOpShootWithAreaCommand(DoubleSupplier areaSupplier)`
Teleop: run shooter at area-based speed + feeder simultaneously. Bind with `.whileTrue()`.

---

## Climb Commands (Climb)

### `climb.climbUpCommand()`
Run climb motor upward. Brakes on release. Always use `.withTimeout()` in auto.

```java
climb.climbUpCommand().withTimeout(4.0)
```

### `climb.climbDownCommand()`
Run climb motor downward. Brakes on release.

---

## Example: Full Auto Routine

```java
Commands.sequence(
    autoCommands.resetPose(Waypoints.START_CENTER),
    autoCommands.log("Phase 1: drive backward"),
    autoCommands.driveBackwardUntilTag(limelight, hubTagIds, 0.5, 6.0),
    autoCommands.log("Phase 2: align"),
    autoCommands.alignToHubTag(limelight, centerTagId, 5.0),
    autoCommands.log("Phase 3: spin up"),
    superstructure.spinUpForAreaAndWaitCommand(limelight::getTargetArea).withTimeout(3.0),
    autoCommands.log("Phase 4: shoot"),
    superstructure.shootCommand(),
    superstructure.stowCommand(),
    autoCommands.log("Phase 5: find climb"),
    autoCommands.blindSpin(0.6, 1.5),
    autoCommands.spinToFindTag(limelight, climbTagIds, 0.6, 8.5),
    autoCommands.log("Phase 6: climb"),
    climb.climbUpCommand().withTimeout(4.0),
    autoCommands.log("Complete!")
)
```

---

## Tips

- Always wrap auto commands with `.withTimeout()` to prevent infinite hangs
- Use `Commands.defer()` with alliance tag ID resolution so the correct tags are used at enable time
- The `blindSpin` before `spinToFindTag` prevents instant exit when climb tags are already visible
- Area-based shooting reads tag area at the moment the command starts — align first, then spin up
