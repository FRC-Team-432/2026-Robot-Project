# Autonomous Building Blocks Reference

Quick reference for the drive team when building new auto routines between matches.

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
// Start spinning shooter when 1.5m from scoring position
Commands.parallel(
    autoCommands.driveTo(Waypoints.SCORE_A),
    autoCommands.distanceCommand(1.5, Waypoints.SCORE_A, superstructure.speakerCloseCommand())
)
```

### `resetPose(Pose2d pose)`
Tell the robot where it is on the field. Use at the start of every auto.

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
Rotate in place to center on the hub's center AprilTag. Uses P-control with camera offset correction. Exits when centered within 3 degrees for 0.05 seconds.

```java
autoCommands.alignToHubTag(limelight, centerTagId, 5.0)
```

### `spinToFindTag(LimelightSubsystem limelight, int[] tagIds, double spinRateRadS, double timeoutSeconds)`
Spin in place until one of the specified tags is visible.

```java
autoCommands.spinToFindTag(limelight, climbTagIds, 0.6, 8.5)
```

### `blindSpin(double spinRateRadS, double durationSeconds)`
Spin for a fixed time with no exit condition. Use before `spinToFindTag` to avoid false-positive exits when tags from the previous step are still visible.

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
Print a message to the console for debugging.

```java
autoCommands.log("AUTO: Phase 1 starting")
```

---

## Shooter Blocks (Superstructure)

### `speakerCloseAndWaitCommand()`
Spin up shooter at fixed speed and wait until at target speed.

```java
superstructure.speakerCloseAndWaitCommand().withTimeout(3.0)
```

### `spinUpForAreaAndWaitCommand(DoubleSupplier areaSupplier)`
Spin up shooter at area-based speed (reads tag area once), wait until at speed.

```java
superstructure.spinUpForAreaAndWaitCommand(limelight::getTargetArea).withTimeout(3.0)
```

### `shootCommand()`
Fire a game piece (runs feeder for 0.5 seconds).

```java
superstructure.shootCommand()
```

### `stowCommand()`
Stop the shooter.

```java
superstructure.stowCommand()
```

### `teleOpShootWithAreaCommand(DoubleSupplier areaSupplier)`
Teleop: run shooter at area-based speed + feeder while held.

```java
superstructure.teleOpShootWithAreaCommand(limelight::getTargetArea)
```

---

## Climb Block (Climb)

### `climbUpCommand()`
Climb upward while active, brakes on release. Always use with `.withTimeout()`.

```java
climb.climbUpCommand().withTimeout(4.0)
```

---

## Complete Auto Example

```java
private Command visionDriveAndShoot(Pose2d startPose) {
    return Commands.defer(() -> {
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        int[] hubTagIds = isBlue ? VisionConstants.BLUE_HUB_ALL_TAG_IDS : VisionConstants.RED_HUB_ALL_TAG_IDS;
        int centerTagId = isBlue ? VisionConstants.BLUE_HUB_CENTER_TAG_IDS[0] : VisionConstants.RED_HUB_CENTER_TAG_IDS[0];
        int[] climbTagIds = isBlue ? VisionConstants.BLUE_CLIMB_TAG_IDS : VisionConstants.RED_CLIMB_TAG_IDS;

        return Commands.sequence(
            autoCommands.resetPose(startPose),
            autoCommands.driveBackwardUntilTag(limelight, hubTagIds, 0.5, 6.0),
            autoCommands.alignToHubTag(limelight, centerTagId, 5.0),
            superstructure.spinUpForAreaAndWaitCommand(limelight::getTargetArea).withTimeout(3.0),
            superstructure.shootCommand(),
            superstructure.stowCommand(),
            autoCommands.blindSpin(0.6, 1.5),
            autoCommands.spinToFindTag(limelight, climbTagIds, 0.6, 8.5),
            climb.climbUpCommand().withTimeout(4.0)
        );
    }, Set.of(drivetrain, climb));
}
```

**Important:** Always wrap alliance-dependent tag IDs inside `Commands.defer()` so they are resolved at enable time, not at robot startup.
