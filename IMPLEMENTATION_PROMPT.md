# COMPETITION-CRITICAL IMPLEMENTATION PROMPT
# Autonomous Building Blocks + Area-Based Shooting
# FRC Team 432 - 2026 Season

---

## READ THIS ENTIRE PROMPT BEFORE MAKING ANY CHANGES. DO NOT SKIP SECTIONS.

You have ONE chance to get this right. The robot competition is in 2 days. There is no time for debug cycles. Every change must compile and work on first deploy.

---

## PROJECT CONTEXT

FRC Team 432 robot code. Java 17, WPILib 2026, CTRE Phoenix 6, swerve drive with Limelight AprilTag vision. Command-based robot pattern.

The robot needs two features implemented:
1. **Area-based shooting** - Adjust shooter speed based on how large the AprilTag appears in the camera (larger tag = closer = slower speed needed)
2. **Autonomous building blocks** - Reusable command functions that compose like block code for autonomous routines

---

## STEP 0: READ THE CODEBASE FIRST

Before making ANY changes, read ALL of these files to understand the architecture, patterns, and existing code:

```
src/main/java/frc/robot/RobotContainer.java
src/main/java/frc/robot/subsystems/Superstructure.java
src/main/java/frc/robot/subsystems/shooter/Shooter.java
src/main/java/frc/robot/subsystems/shooter/Feeder.java
src/main/java/frc/robot/subsystems/vision/LimelightSubsystem.java
src/main/java/frc/robot/subsystems/intake/Intake.java
src/main/java/frc/robot/subsystems/climb/Climb.java
src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java
src/main/java/frc/robot/autonomous/AutoCommands.java
src/main/java/frc/robot/autonomous/AutoRoutines.java
src/main/java/frc/robot/autonomous/LinearPathRequest.java
src/main/java/frc/robot/constants/ShooterConstants.java
src/main/java/frc/robot/constants/AutoConstants.java
src/main/java/frc/robot/constants/VisionConstants.java
src/main/java/frc/robot/constants/Waypoints.java
src/main/java/frc/robot/commands/AlignToTagCommand.java
src/main/java/frc/robot/commands/FaceTagCommand.java
src/main/java/frc/robot/commands/DriveToTagCommand.java
src/main/java/frc/robot/commands/DriveAndLockCommand.java
src/main/java/frc/robot/utils/LimelightHelpers.java
```

Pay close attention to:
- How commands are created (`.withName()`, `.withTimeout()`, `.finallyDo()`)
- How subsystem requirements work (commands from SubsystemBase methods auto-require that subsystem)
- How `TalonFXUtil.applyConfigWithRetries()` is used for motor config
- The `@Logged` annotation pattern on subsystems
- Alliance-aware tag ID resolution using `Commands.defer()` and `DriverStation.getAlliance()`
- The `Debouncer` pattern in AutoRoutines for confirming alignment

---

## TASK 1: FIX CRITICAL BUG - driveTo() NEVER FINISHES

In `AutoCommands.java`, the `driveTo()` method is broken. It calls `drivetrain.applyRequest()` which creates a `run()` command that executes forever. The `LinearPathRequest` class has an `isFinished()` method, but nothing ever checks it.

**Fix**: Add `.until(() -> pathRequest.isFinished())` to the drive portion.

Current (BROKEN - command runs forever):
```java
public Command driveTo(Pose2d pose) {
    return drivetrain.runOnce(() -> pathRequest.reset(drivetrain.getPose(), drivetrain.getFieldSpeeds()))
        .andThen(drivetrain.applyRequest(() -> pathRequest.withTargetPose(pose)));
}
```

Fixed:
```java
public Command driveTo(Pose2d pose) {
    return drivetrain.runOnce(() -> pathRequest.reset(drivetrain.getPose(), drivetrain.getFieldSpeeds()))
        .andThen(
            drivetrain.applyRequest(() -> pathRequest.withTargetPose(pose))
                .until(() -> pathRequest.isFinished()))
        .withName("DriveTo");
}
```

Apply this same `.until(() -> pathRequest.isFinished())` pattern to ALL existing methods that use `driveTo()` internally (like `driveToWithAction`, `driveToThenExecute`).

---

## TASK 2: AREA-BASED SHOOTING SYSTEM

### The Math Problem

The original formula was: `speed = (1 - total_area) * max_velocity`

This is broken because:
- `LimelightHelpers.getTA()` returns 0-100 (percentage of image), NOT 0-1
- Even normalized to 0-1, AprilTags at typical FRC distances (1-5m) occupy only 0.5%-15% of the image, giving a speed range of only ~10% variation - far too narrow

### Correct Approach: Area-to-Speed Interpolation Table

Use the existing `InterpolatingDoubleTreeMap` (already in Shooter.java) with an area-based lookup table. This is:
- Easy to tune at competition (just change numbers between matches)
- Handles non-linear relationships naturally
- Clamps gracefully at extremes via interpolation

### Changes Required

#### A. `ShooterConstants.java` - Replace speed constants

Replace `DISTANCE_SPEED_MAP` with `AREA_SPEED_MAP`. Fix the speed value conflicts:

```java
// ==================== Area-Based Shooting ====================
// Maps Limelight target area (getTA(), 0-100 scale) to shooter speed (RPS).
// LARGER area = CLOSER to target = SLOWER speed.
// SMALLER area = FARTHER from target = FASTER speed.
//
// HOW TO TUNE AT COMPETITION:
//   1. Drive robot to a shooting distance
//   2. Read "Limelight/TA" from SmartDashboard (this is the area value)
//   3. Manually adjust shooter speed until shots score consistently
//   4. Record the {area, speed} pair below
//   5. Repeat at 3-4 different distances
//
// These are PLACEHOLDER values - you MUST tune them on the real robot.
public static final double[][] AREA_SPEED_MAP = {
    // { tagAreaPercent, shooterSpeedRPS }
    {0.0,  35.0},  // No/tiny target - use fallback speed
    {0.5,  45.0},  // Very far away - high speed
    {1.0,  40.0},  // Far
    {2.0,  35.0},  // Medium-far
    {5.0,  28.0},  // Medium
    {10.0, 22.0},  // Close
    {15.0, 18.0},  // Very close - low speed
};

// Speed when no tag is visible (safe medium value)
public static final double FALLBACK_SPEED_RPS = 35.0;

// Safety limits
public static final double MAX_SHOOTER_SPEED_RPS = 50.0;
public static final double MIN_SHOOTER_SPEED_RPS = 15.0;
```

Also fix these conflicts:
- `SHOOTER_SPEED_RPS` (fixed teleop speed): change from 200 to 35 (a reasonable default)
- `MOTION_MAGIC_CRUISE_VELOCITY`: change from 100 to 60 (must be >= MAX_SHOOTER_SPEED_RPS)

Keep all other ShooterConstants values (CAN IDs, feeder speed, PID, tolerances, shoot duration) unchanged.

#### B. `LimelightSubsystem.java` - Add area accessor

Add this method (it wraps getTA for clean access from commands):

```java
/** Returns the primary target area (0-100 scale), or 0.0 if no target visible. */
public double getTargetArea() {
    return LimelightHelpers.getTA(limelightName);
}
```

#### C. `Shooter.java` - Replace distance-based with area-based shooting

1. Rename `distanceSpeedMap` field to `areaSpeedMap`
2. Load from `ShooterConstants.AREA_SPEED_MAP` instead of `DISTANCE_SPEED_MAP`
3. Replace `spinAtDistanceWhileHeld(DoubleSupplier distanceMeters)` with:

```java
/**
 * Spin the shooter at a speed based on the current AprilTag area.
 * Larger area (closer) = slower. Smaller area (farther) = faster.
 * Recalculates speed every loop cycle. Stops on command cancel.
 *
 * For teleop use with .whileTrue() binding.
 *
 * @param areaSupplier Supplier for current tag area (0-100 from getTA())
 */
public Command spinAtAreaWhileHeld(DoubleSupplier areaSupplier) {
    return run(() -> {
            double area = areaSupplier.getAsDouble();
            double speed;
            if (area < 0.01) {
                speed = ShooterConstants.FALLBACK_SPEED_RPS;
            } else {
                speed = areaSpeedMap.get(area);
            }
            speed = MathUtil.clamp(speed,
                ShooterConstants.MIN_SHOOTER_SPEED_RPS,
                ShooterConstants.MAX_SHOOTER_SPEED_RPS);
            setVelocity(RotationsPerSecond.of(speed));
        })
        .finallyDo(() -> stop())
        .withName("ShooterSpinAtArea");
}
```

4. Add an instant command for autonomous (reads area once, sets speed, releases subsystem):

```java
/**
 * Set shooter speed based on current tag area, then release.
 * The TalonFX holds speed on its own. For use in auto sequences.
 *
 * @param areaSupplier Supplier for current tag area (0-100 from getTA())
 */
public Command spinUpForArea(DoubleSupplier areaSupplier) {
    return runOnce(() -> {
            double area = areaSupplier.getAsDouble();
            double speed;
            if (area < 0.01) {
                speed = ShooterConstants.FALLBACK_SPEED_RPS;
            } else {
                speed = areaSpeedMap.get(area);
            }
            speed = MathUtil.clamp(speed,
                ShooterConstants.MIN_SHOOTER_SPEED_RPS,
                ShooterConstants.MAX_SHOOTER_SPEED_RPS);
            setVelocity(RotationsPerSecond.of(speed));
        })
        .withName("ShooterSpinUpForArea");
}
```

5. Add `import edu.wpi.first.math.MathUtil;` if not already present.

6. Keep the existing `spinWhileHeld()` and `spinUpOnce()` as fallbacks (they use the fixed SHOOTER_SPEED_RPS).

#### D. `Superstructure.java` - Add area-based commands

Add these methods:

```java
/**
 * Teleop: run shooter at area-based speed + feeder simultaneously.
 * Both stop when the command ends (trigger released).
 */
public Command teleOpShootWithAreaCommand(DoubleSupplier areaSupplier) {
    return Commands.parallel(
            shooter.spinAtAreaWhileHeld(areaSupplier),
            feeder.feedWhileHeld())
        .withName("TeleOpShootWithArea");
}

/**
 * Auto: spin up shooter based on tag area, wait until at speed.
 * Reads area once at the moment this command starts.
 */
public Command spinUpForAreaAndWaitCommand(DoubleSupplier areaSupplier) {
    return shooter.spinUpForArea(areaSupplier)
        .andThen(Commands.waitUntil(() -> shooter.isAtTarget()))
        .withName("SpinUpForAreaAndWait");
}
```

Keep ALL existing methods (speakerClose, speakerFar, stow, shoot, teleOpShoot, etc.) as fallbacks.

#### E. `RobotContainer.java` - Wire area-based teleop shooting

Change the right trigger binding from fixed speed to area-based:

```java
// Right trigger - shoot with area-based speed adjustment
joystick.rightTrigger(0.1).whileTrue(superstructure.teleOpShootWithAreaCommand(limelight::getTargetArea));
```

Remove or comment out the old fixed-speed line and the old distance-based comment.

---

## TASK 3: AUTONOMOUS BUILDING BLOCKS

Add new building block methods to `AutoCommands.java`. These must be reusable, composable command factories.

Each block MUST:
- Have a `.withName()` for SmartDashboard identification
- Include safety timeouts where appropriate
- Take subsystem references as method parameters (NOT constructor fields)
- Follow the existing code patterns exactly

### Vision-Drive Blocks

These go in AutoCommands.java because they control the drivetrain:

```java
/**
 * Drive backward (robot-relative, -X) until any of the specified tags is visible.
 * Stops driving when a tag is found OR when timeout expires.
 *
 * @param limelight Vision subsystem to check for tags
 * @param tagIds Array of AprilTag IDs to look for
 * @param speedMps Backward driving speed in m/s (positive value, will be negated)
 * @param timeoutSeconds Maximum time to drive before giving up
 */
public Command driveBackwardUntilTag(LimelightSubsystem limelight, int[] tagIds,
    double speedMps, double timeoutSeconds)
```

```java
/**
 * Rotate in place to center on a specific hub tag.
 *
 * Three-way logic each loop cycle:
 *   a) Center tag is primary target -> P-control on TX with camera offset correction
 *   b) A different tag is primary (side tag) -> spin toward it using TX sign
 *   c) No tag visible -> slow fallback spin
 *
 * Exits when the center tag is centered within 3 degrees for 0.05 seconds (Debouncer).
 *
 * @param limelight Vision subsystem
 * @param centerTagId The center hub tag ID to align to (26 blue, 10 red)
 * @param timeoutSeconds Maximum alignment time
 */
public Command alignToHubTag(LimelightSubsystem limelight, int centerTagId,
    double timeoutSeconds)
```

```java
/**
 * Spin in place (no forward/backward movement) until any of the specified tags
 * is visible, or timeout expires.
 *
 * @param limelight Vision subsystem
 * @param tagIds Array of tag IDs to search for
 * @param spinRateRadS Rotation speed in rad/s (positive = CCW)
 * @param timeoutSeconds Maximum search time
 */
public Command spinToFindTag(LimelightSubsystem limelight, int[] tagIds,
    double spinRateRadS, double timeoutSeconds)
```

```java
/**
 * Blind spin (no exit condition other than time). Used to rotate away from
 * a known area before starting a tag search, preventing false-positive exits.
 *
 * @param spinRateRadS Rotation speed in rad/s (positive = CCW)
 * @param durationSeconds How long to spin
 */
public Command blindSpin(double spinRateRadS, double durationSeconds)
```

### The vision blocks need a RobotCentric SwerveRequest. Add this field to AutoCommands:

```java
private final SwerveRequest.RobotCentric robotCentric =
    new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);
```

Add the required imports for SwerveRequest, DriveRequestType, SteerRequestType, MathUtil, Debouncer, DebounceType, and VisionConstants.

### Implementation Notes for Vision Blocks

- `driveBackwardUntilTag`: Use `drivetrain.applyRequest(() -> robotCentric.withVelocityX(-speedMps))` with `.until(() -> limelight.hasSpecificTag(tagIds))` and `.withTimeout(timeoutSeconds)`.

- `alignToHubTag`: Use the EXACT same three-way rotation logic from the current `AutoRoutines.visionDriveAndShoot()` step 2 (lines ~113-137). This logic is proven to work. Copy it faithfully. Use `drivetrain.applyRequest(() -> { ... return robotCentric.withVelocityX(0).withRotationalRate(rotRate); })` with a Debouncer for exit.

- `spinToFindTag`: Simple `drivetrain.applyRequest(() -> robotCentric.withVelocityX(0).withRotationalRate(spinRateRadS))` with `.until(() -> limelight.hasSpecificTag(tagIds))` and `.withTimeout()`.

- `blindSpin`: Same pattern but with only `.withTimeout(durationSeconds)`, no exit condition.

### Utility Blocks

```java
/** Wait for a specified duration. Convenience wrapper. */
public Command waitSeconds(double seconds)

/** Print a message to console. Useful for tracing auto execution. */
public Command log(String message)
```

---

## TASK 4: COMPOSE COMPLETE AUTO ROUTINES

Rebuild `AutoRoutines.java` using the building blocks from Task 3.

### CRITICAL: Preserve What Works

The current vision auto strategy is proven to work on the robot. The new code must follow the EXACT same strategy, just using building blocks instead of inline code. Do NOT change:
- The backward-drive-then-search approach
- The three-way rotation logic (center tag P-control, side tag directional spin, fallback)
- The Debouncer confirmation pattern
- The mandatory blind spin before climb tag search
- The alliance-aware tag resolution at enable time via Commands.defer()
- The Set.of(drivetrain, climb) requirement declaration in defer()
- Starting poses from Waypoints (START_LEFT, START_CENTER, START_RIGHT)
- All timeout values (6s drive, 5s align, 3s spinup, 1.5s blind spin, 8.5s search, 4s climb)

### Target Structure

The `visionDriveAndShoot(Pose2d startPose)` method should become a clean sequence of building blocks:

```java
private Command visionDriveAndShoot(Pose2d startPose) {
    return Commands.defer(() -> {
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        int[] hubTagIds = isBlue ? VisionConstants.BLUE_HUB_ALL_TAG_IDS : VisionConstants.RED_HUB_ALL_TAG_IDS;
        int centerTagId = isBlue ? VisionConstants.BLUE_HUB_CENTER_TAG_IDS[0] : VisionConstants.RED_HUB_CENTER_TAG_IDS[0];
        int[] climbTagIds = isBlue ? VisionConstants.BLUE_CLIMB_TAG_IDS : VisionConstants.RED_CLIMB_TAG_IDS;

        return Commands.sequence(
            // Setup
            autoCommands.resetPose(startPose),
            autoCommands.log("AUTO: Phase 1 - driving backward until tag visible"),

            // Phase 1: Drive backward until hub tag visible
            autoCommands.driveBackwardUntilTag(limelight, hubTagIds, 0.5, 6.0),
            autoCommands.log("AUTO: Phase 2 - aligning to hub center tag"),

            // Phase 2: Rotate to center on hub tag
            autoCommands.alignToHubTag(limelight, centerTagId, 5.0),
            autoCommands.log("AUTO: Phase 3 - spinning up shooter"),

            // Phase 3: Spin up shooter at area-based speed, wait until ready
            superstructure.spinUpForAreaAndWaitCommand(limelight::getTargetArea).withTimeout(3.0),
            autoCommands.log("AUTO: Phase 4 - shooting"),

            // Phase 4: Fire and stow
            superstructure.shootCommand(),
            superstructure.stowCommand(),
            autoCommands.log("AUTO: Phase 5 - searching for climb tags"),

            // Phase 5: Mandatory blind spin then search for climb tags
            autoCommands.blindSpin(0.6, 1.5),
            autoCommands.spinToFindTag(limelight, climbTagIds, 0.6, 8.5),
            autoCommands.log("AUTO: Phase 6 - climbing"),

            // Phase 6: Climb
            climb.climbUpCommand().withTimeout(4.0),
            autoCommands.log("AUTO: Complete")
        );
    }, Set.of(drivetrain, climb));
}
```

Keep the three public methods `leftStartAuto()`, `centerStartAuto()`, `rightStartAuto()` that call `visionDriveAndShoot()` with the appropriate starting pose.

Keep the commented-out example autos at the bottom of the file - they are useful reference.

### AutoRoutines Constructor

The constructor signature should NOT change. It already receives everything needed:
```java
public AutoRoutines(AutoCommands autoCommands, Superstructure superstructure,
    CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight, Climb climb)
```

Remove the `driveBackward` and `driveAndAlign` SwerveRequest fields from AutoRoutines since that logic moves to AutoCommands.

---

## TASK 5: CREATE BUILDING BLOCKS DOCUMENTATION

Create a file `docs/BUILDING_BLOCKS.md` that lists every available building block with:
- Method signature
- One-line description
- Example usage showing how to compose it in a sequence

This document is for the drive team to reference when creating new auto routines between matches.

---

## VERIFICATION CHECKLIST

After ALL changes are complete, verify EACH of these. Do NOT skip any.

### Compilation
- [ ] Run `./gradlew build` and confirm it compiles with zero errors
- [ ] If there are errors, fix them immediately before doing anything else
- [ ] All imports are correct (no missing imports, no unused imports that cause warnings)

### WPILib Command Rules
- [ ] No two commands in a `Commands.parallel()` require the SAME subsystem
- [ ] Commands returned from SubsystemBase methods auto-require that subsystem - do NOT manually addRequirements
- [ ] `runOnce()` = instant (runs once, finishes). `run()` = continuous (runs every cycle until canceled)
- [ ] `startEnd(start, end)` = runs start on init, end on cancel/finish
- [ ] Every `.until()` condition will eventually become true (or has a `.withTimeout()` backup)
- [ ] Every auto command chain has a timeout preventing infinite hangs
- [ ] `Commands.defer()` includes ALL subsystems used in the deferred command in its Set.of()

### Shooter Verification
- [ ] MOTION_MAGIC_CRUISE_VELOCITY >= MAX_SHOOTER_SPEED_RPS (currently 60 >= 50)
- [ ] AREA_SPEED_MAP entries are sorted by area ascending
- [ ] Speed is clamped between MIN_SHOOTER_SPEED_RPS and MAX_SHOOTER_SPEED_RPS
- [ ] Fallback speed is used when area < 0.01 (no tag visible)
- [ ] Both leader AND follower motors receive the velocity command (check setVelocity method)
- [ ] The `isAtTarget()` tolerance check still works with new speed values

### Autonomous Verification
- [ ] driveTo() now has `.until(() -> pathRequest.isFinished())` so it actually terminates
- [ ] All building blocks have `.withName()` for SmartDashboard identification
- [ ] Alliance tag IDs are resolved inside Commands.defer(), not at construction time
- [ ] The alignToHubTag block uses the camera TX offset correction (VisionConstants.CAMERA_TX_OFFSET_DEG)
- [ ] The Debouncer pattern (0.05s rising) is preserved for alignment confirmation
- [ ] The mandatory blind spin (1.5s) happens BEFORE the climb tag search
- [ ] Starting poses match: START_LEFT, START_CENTER, START_RIGHT from Waypoints

### Integration
- [ ] RobotContainer creates AutoCommands with correct constructor args
- [ ] RobotContainer creates AutoRoutines with correct constructor args
- [ ] RobotContainer teleop right trigger uses area-based shooting
- [ ] SmartDashboard auto chooser still has all options (Center/Left/Right + PathPlanner backups)
- [ ] PathPlanner named commands ("shoot", "climbUp", "climbDown") still work
- [ ] No changes to files that should NOT be modified (TunerConstants, LinearPathRequest, Feeder, Climb, Intake, VisionConstants, Waypoints, command files)

### Final Sanity Check
- [ ] Read through each modified file one more time end-to-end
- [ ] Confirm no TODO items were accidentally deleted that are still relevant
- [ ] Confirm the code follows the same style as the rest of the codebase (spacing, naming, comments)
- [ ] Run `./gradlew build` one final time to confirm everything still compiles

---

## CRITICAL RULES

1. **Do NOT modify** these files: TunerConstants.java (generated), LinearPathRequest.java, Feeder.java, Climb.java, Intake.java, VisionConstants.java, Waypoints.java, AutoConstants.java, any file in commands/ directory, LimelightHelpers.java
2. **DO modify** these files: ShooterConstants.java, Shooter.java, Superstructure.java, LimelightSubsystem.java (add one method), AutoCommands.java, AutoRoutines.java, RobotContainer.java
3. **DO create** this file: docs/BUILDING_BLOCKS.md
4. **Preserve all existing functionality** - the fixed-speed teleop, PathPlanner autos, and all teleop bindings must continue to work
5. **Follow existing code patterns exactly** - same import style, same comment style, same naming conventions
6. **When in doubt, keep it simple** - a working simple solution beats a broken clever one
7. **Test compilation** - run `./gradlew build` after all changes

---

## FILES MODIFIED SUMMARY

| File | Changes |
|------|---------|
| `ShooterConstants.java` | Replace DISTANCE_SPEED_MAP with AREA_SPEED_MAP, fix SHOOTER_SPEED_RPS and CRUISE_VELOCITY |
| `Shooter.java` | Replace distance map with area map, add spinAtAreaWhileHeld() and spinUpForArea() |
| `LimelightSubsystem.java` | Add getTargetArea() method |
| `Superstructure.java` | Add teleOpShootWithAreaCommand() and spinUpForAreaAndWaitCommand() |
| `AutoCommands.java` | Fix driveTo bug, add vision blocks, utility blocks, SwerveRequest field |
| `AutoRoutines.java` | Rebuild using building blocks, remove inline drive requests |
| `RobotContainer.java` | Wire area-based teleop shooting |
| `docs/BUILDING_BLOCKS.md` | NEW - Document all building blocks for drive team |
