# Autonomous Building Blocks Reference

Quick reference for the drive team when composing autonomous routines between matches.

---

## Drive Blocks (AutoCommands)

### `driveTo(Pose2d pose)`
Drive to a specific field position and heading using path planning.
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
Set the robot's starting position on the field (use at the start of auto).
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
Rotate in place to center on a specific hub tag using three-way P-control logic.
```java
autoCommands.alignToHubTag(limelight, 26, 5.0)  // Blue center tag
```

### `spinToFindTag(LimelightSubsystem limelight, int[] tagIds, double spinRateRadS, double timeoutSeconds)`
Spin in place until any of the specified tags is visible, or timeout.
```java
autoCommands.spinToFindTag(limelight, climbTagIds, 0.6, 8.5)
```

### `blindSpin(double spinRateRadS, double durationSeconds)`
Spin for a fixed duration with no exit condition. Use before tag searches to prevent false exits.
```java
autoCommands.blindSpin(0.6, 1.5)
```

---

## Shooter Blocks (Superstructure)

### `teleOpShootWithAreaCommand(DoubleSupplier areaSupplier)`
Teleop: run shooter at area-based speed + feeder. Both stop on release.
```java
superstructure.teleOpShootWithAreaCommand(limelight::getTargetArea)
```

### `spinUpForAreaAndWaitCommand(DoubleSupplier areaSupplier)`
Auto: spin up shooter based on tag area, wait until at speed.
```java
superstructure.spinUpForAreaAndWaitCommand(limelight::getTargetArea).withTimeout(3.0)
```

### `speakerCloseAndWaitCommand()`
Spin up shooter at fixed speed, wait until ready.
```java
superstructure.speakerCloseAndWaitCommand().withTimeout(3.0)
```

### `shootCommand()`
Fire the feeder to push a ball through the shooter (use after spin-up).
```java
superstructure.shootCommand()
```

### `stowCommand()`
Stop the shooter.
```java
superstructure.stowCommand()
```

---

## Climb Blocks (Climb)

### `climbUpCommand()`
Climb up while the command is active. Always use with `.withTimeout()`.
```java
climb.climbUpCommand().withTimeout(4.0)
```

### `climbDownCommand()`
Climb down while the command is active. Always use with `.withTimeout()`.
```java
climb.climbDownCommand().withTimeout(1.5)
```

---

## Utility Blocks (AutoCommands)

### `waitSeconds(double seconds)`
Wait for a specified duration.
```java
autoCommands.waitSeconds(0.5)
```

### `log(String message)`
Print a message to the console for tracing auto execution.
```java
autoCommands.log("AUTO: Phase 1 complete")
```

---

## Composing a Full Auto Routine

Use `Commands.sequence()` to chain blocks in order:

```java
Commands.sequence(
    autoCommands.resetPose(Waypoints.START_CENTER),
    autoCommands.log("Phase 1: Drive backward"),
    autoCommands.driveBackwardUntilTag(limelight, hubTagIds, 0.5, 6.0),
    autoCommands.log("Phase 2: Align"),
    autoCommands.alignToHubTag(limelight, centerTagId, 5.0),
    autoCommands.log("Phase 3: Spin up"),
    superstructure.spinUpForAreaAndWaitCommand(limelight::getTargetArea).withTimeout(3.0),
    autoCommands.log("Phase 4: Shoot"),
    superstructure.shootCommand(),
    superstructure.stowCommand(),
    autoCommands.log("Phase 5: Find climb"),
    autoCommands.blindSpin(0.6, 1.5),
    autoCommands.spinToFindTag(limelight, climbTagIds, 0.6, 8.5),
    autoCommands.log("Phase 6: Climb"),
    climb.climbUpCommand().withTimeout(4.0),
    autoCommands.log("Complete!")
)
```

Use `Commands.parallel()` to run blocks simultaneously:

```java
Commands.parallel(
    autoCommands.driveTo(shootPose),
    superstructure.speakerCloseCommand()
)
```

**Important**: Two commands in a `parallel()` must NOT require the same subsystem.
