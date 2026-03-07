# Autonomous Building Blocks Reference

Quick reference for the drive team when composing autonomous routines between matches.

All building blocks are methods on `AutoCommands` or `Superstructure`. Compose them using `Commands.sequence(...)` and `Commands.parallel(...)`.

---

## Drive Blocks (AutoCommands)

### `driveTo(Pose2d pose)`
Drive to a specific field position using path planning. Finishes when the robot arrives.
```java
autoCommands.driveTo(Waypoints.SCORE_A)
```

### `resetPose(Pose2d pose)`
Set the robot's starting position. Use at the beginning of every auto.
```java
autoCommands.resetPose(Waypoints.START_CENTER)
```

### `driveToWithAction(Pose2d targetPose, Command parallelCommand)`
Drive to a location while doing something else simultaneously.
```java
autoCommands.driveToWithAction(Waypoints.SCORE_A, superstructure.speakerCloseCommand())
```

### `driveToThenExecute(Pose2d targetPose, Command afterCommand)`
Drive to a location, then do something after arriving.
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

---

## Vision-Drive Blocks (AutoCommands)

### `driveBackwardUntilTag(LimelightSubsystem limelight, int[] tagIds, double speedMps, double timeoutSeconds)`
Drive backward until any of the specified AprilTags is visible.
```java
autoCommands.driveBackwardUntilTag(limelight, hubTagIds, 0.5, 6.0)
```

### `alignToHubTag(LimelightSubsystem limelight, int centerTagId, double timeoutSeconds)`
Rotate in place to center on a specific hub tag. Uses three-way logic: P-control on center tag, directional spin on side tags, fallback spin when no tag visible. Exits when centered within 3 degrees for 0.05s.
```java
autoCommands.alignToHubTag(limelight, centerTagId, 5.0)
```

### `spinToFindTag(LimelightSubsystem limelight, int[] tagIds, double spinRateRadS, double timeoutSeconds)`
Spin in place until any of the specified tags is visible.
```java
autoCommands.spinToFindTag(limelight, climbTagIds, 0.6, 8.5)
```

### `blindSpin(double spinRateRadS, double durationSeconds)`
Spin for a fixed duration with no exit condition. Use before `spinToFindTag` to rotate away from the current area and prevent false-positive exits.
```java
autoCommands.blindSpin(0.6, 1.5)
```

---

## Utility Blocks (AutoCommands)

### `waitSeconds(double seconds)`
Wait for a specified duration.
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
Spin up the shooter at fixed speed and wait until at target speed.
```java
superstructure.speakerCloseAndWaitCommand().withTimeout(3.0)
```

### `spinUpForAreaAndWaitCommand(DoubleSupplier areaSupplier)`
Spin up the shooter at area-based speed (reads tag area once), then wait until at speed.
```java
superstructure.spinUpForAreaAndWaitCommand(limelight::getTargetArea).withTimeout(3.0)
```

### `shootCommand()`
Fire the feeder to push a ball through the spinning shooter wheels.
```java
superstructure.shootCommand()
```

### `stowCommand()`
Stop the shooter (safe state).
```java
superstructure.stowCommand()
```

### `teleOpShootWithAreaCommand(DoubleSupplier areaSupplier)`
Teleop: run shooter at area-based speed + feeder simultaneously. Both stop on release.
```java
superstructure.teleOpShootWithAreaCommand(limelight::getTargetArea)
```

---

## Climb Blocks

### `climb.climbUpCommand()`
Climb up while the command is active. Always use with `.withTimeout()`.
```java
climb.climbUpCommand().withTimeout(4.0)
```

### `climb.climbDownCommand()`
Climb down while the command is active. Always use with `.withTimeout()`.
```java
climb.climbDownCommand().withTimeout(1.5)
```

---

## Example: Full Vision Auto Routine

```java
Commands.sequence(
    autoCommands.resetPose(startPose),
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
    climb.climbUpCommand().withTimeout(4.0)
)
```

---

## Tips

- **Always add `.withTimeout()`** to any command that could run forever (drive, align, spin up).
- **Wrap alliance-dependent routines in `Commands.defer()`** so tag IDs are resolved at enable time, not at construction time.
- **Include `Set.of(drivetrain, climb)`** in the defer call to declare all subsystems used.
- **Use `autoCommands.log()`** liberally — it costs nothing and helps debug auto on the field.
- **Area-based speed values** are in `ShooterConstants.AREA_SPEED_MAP` — tune by reading `Limelight/TA` from SmartDashboard.
