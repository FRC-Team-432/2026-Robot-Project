# Autonomous Building Blocks Reference
## FRC Team 432 - 2026 Season

Quick reference for the drive team when composing new auto routines between matches.

---

## Drive Blocks (AutoCommands)

### `driveTo(Pose2d pose)`
Drive to a specific field position using path planning. Finishes when the robot arrives.

```java
autoCommands.driveTo(Waypoints.SCORE_A)
```

### `resetPose(Pose2d pose)`
Tell the robot where it is on the field. Always call this first in auto.

```java
autoCommands.resetPose(Waypoints.START_CENTER)
```

### `driveToWithAction(Pose2d targetPose, Command parallelCommand)`
Drive to a location while doing something else at the same time.

```java
autoCommands.driveToWithAction(Waypoints.SCORE_A, superstructure.speakerCloseCommand())
```

### `driveToThenExecute(Pose2d targetPose, Command afterCommand)`
Drive to a location, then do something when you arrive.

```java
autoCommands.driveToThenExecute(Waypoints.SCORE_A, superstructure.shootCommand())
```

### `distanceCommand(double triggerDistance, Pose2d targetPose, Command command)`
Start an action when the robot gets within a certain distance of a target.

```java
Commands.parallel(
    autoCommands.driveTo(Waypoints.SCORE_A),
    autoCommands.distanceCommand(1.5, Waypoints.SCORE_A, superstructure.speakerCloseCommand())
)
```

---

## Vision-Drive Blocks (AutoCommands)

### `driveBackwardUntilTag(LimelightSubsystem limelight, int[] tagIds, double speedMps, double timeoutSeconds)`
Drive backward until the camera sees one of the specified AprilTags. Stops on tag detection or timeout.

```java
autoCommands.driveBackwardUntilTag(limelight, hubTagIds, 0.5, 6.0)
```

### `alignToHubTag(LimelightSubsystem limelight, int centerTagId, double timeoutSeconds)`
Rotate in place to center on a specific hub tag. Uses three-way logic: P-control when center tag is visible, spin toward side tags, or slow fallback spin. Exits when centered within 3 degrees for 0.05s.

```java
autoCommands.alignToHubTag(limelight, centerTagId, 5.0)
```

### `spinToFindTag(LimelightSubsystem limelight, int[] tagIds, double spinRateRadS, double timeoutSeconds)`
Spin in place until any of the specified tags is visible, or timeout expires.

```java
autoCommands.spinToFindTag(limelight, climbTagIds, 0.6, 8.5)
```

### `blindSpin(double spinRateRadS, double durationSeconds)`
Spin for a fixed duration with no exit condition. Use before `spinToFindTag` to rotate away from current tags.

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
Spin up shooter at fixed close-range speed, wait until ready.

```java
superstructure.speakerCloseAndWaitCommand().withTimeout(3.0)
```

### `shootCommand()`
Fire the feeder to push a ball through the shooter.

```java
superstructure.shootCommand()
```

### `stowCommand()`
Stop the shooter (safe state).

```java
superstructure.stowCommand()
```

---

## Climb Blocks

### `climb.climbUpCommand()`
Climb up while active, brakes on release. Always use with `.withTimeout()`.

```java
climb.climbUpCommand().withTimeout(4.0)
```

### `climb.climbDownCommand()`
Lower the robot while active. Always use with `.withTimeout()`.

```java
climb.climbDownCommand().withTimeout(1.5)
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
autoCommands.log("AUTO: Phase 1 complete")
```

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
    autoCommands.log("AUTO: Complete")
)
```

---

## Tuning Area-Based Shooting

The area-speed lookup table is in `ShooterConstants.AREA_SPEED_MAP`. To tune:

1. Drive to a shooting distance
2. Read `Limelight/TA` from SmartDashboard
3. Adjust shooter speed until shots score
4. Record the `{area, speed}` pair in the table
5. Repeat at 3-4 distances
