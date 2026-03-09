# Autonomous Building Blocks

This document contains every available command block for building autonomous routines.
Copy the blocks you want and paste them inside a `Commands.sequence(...)` in `AutoRoutines.java`.

---

## How to Build a Routine

Paste this template into `AutoRoutines.java` and fill in the blocks you want:

```java
public Command myAuto() {
    return Commands.sequence(
        // --- paste blocks here, one per line, separated by commas ---
        autoCommands.resetPose(Waypoints.START_CENTER),
        superstructure.speakerCloseAndWaitCommand(),
        superstructure.shootCommand(),
        autoCommands.driveTo(Waypoints.MIDFIELD_CENTER)
    );
}
```

> Add the method name to the `autoChooser` in `RobotContainer.java` so it shows up on the dashboard:
> ```java
> autoChooser.addOption("My Auto", autoRoutines.myAuto());
> ```

---

## BLOCK 1 — Reset Starting Position
*Always put this first so the robot knows where it starts.*

```java
// Tell the robot its starting position (pick one)
autoCommands.resetPose(Waypoints.START_LEFT)
autoCommands.resetPose(Waypoints.START_CENTER)
autoCommands.resetPose(Waypoints.START_RIGHT)

// Or use a custom position: resetPose(new Pose2d(x, y, Rotation2d.fromDegrees(angle)))
autoCommands.resetPose(new Pose2d(7.15, 4.05, Rotation2d.fromDegrees(0)))
```

---

## BLOCK 2 — Drive Commands

### Drive straight to a position
```java
// Drive to a named field position
autoCommands.driveTo(Waypoints.SCORE_A)
autoCommands.driveTo(Waypoints.SCORE_B)
autoCommands.driveTo(Waypoints.INTAKE_1)
autoCommands.driveTo(Waypoints.MIDFIELD_CENTER)

// Drive to a custom position: driveTo(new Pose2d(x, y, Rotation2d.fromDegrees(angle)))
autoCommands.driveTo(new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(0)))
```

### Drive while doing something at the same time
```java
// Drive to position AND spin up shooter at the same time
autoCommands.driveToWithAction(Waypoints.SCORE_A, superstructure.speakerCloseCommand())

// Drive to intake position AND run intake at the same time
autoCommands.driveToWithAction(Waypoints.INTAKE_1, intake.intake().withTimeout(3.0))

// Drive AND wait 0.5s then spin up (start spinning partway through drive)
autoCommands.driveToWithAction(Waypoints.SCORE_A,
    Commands.sequence(Commands.waitSeconds(0.5), superstructure.speakerCloseCommand()))
```

### Drive to position, then do something after arriving
```java
// Drive there, then shoot
autoCommands.driveToThenExecute(Waypoints.SCORE_A, superstructure.shootCommand())

// Drive there, then spin up and wait until ready
autoCommands.driveToThenExecute(Waypoints.SCORE_A, superstructure.speakerCloseAndWaitCommand())
```

### Trigger an action when close to a target
*Use inside a `Commands.parallel` alongside a `driveTo`.*
```java
// Start spinning up when within 1.5 meters of the scoring spot
Commands.parallel(
    autoCommands.driveTo(Waypoints.SCORE_A),
    autoCommands.distanceCommand(1.5, Waypoints.SCORE_A, superstructure.speakerCloseCommand())
)

// Start running intake when within 0.5 meters of a game piece
Commands.parallel(
    autoCommands.driveTo(Waypoints.INTAKE_1),
    autoCommands.distanceCommand(0.5, Waypoints.INTAKE_1, intake.intake().withTimeout(2.0))
)
```

---

## BLOCK 3 — Shooter Commands

### Spin up (instant — does not wait for shooter to reach speed)
```java
// Start spinning, command ends immediately, TalonFX holds speed
superstructure.speakerCloseCommand()    // standard shot speed
superstructure.speakerFarCommand()      // same speed (no arm angle difference now)
```

### Spin up and WAIT until shooter is at full speed
*Use before shootCommand() in sequences where timing matters.*
```java
// Start spinning AND hold until shooter reaches target speed before moving on
superstructure.speakerCloseAndWaitCommand()
superstructure.speakerFarAndWaitCommand()
```

### Fire (runs feeder to push ball through)
```java
// Runs feeder for SHOOT_DURATION_SECONDS (0.5s) — shooter must already be spinning
superstructure.shootCommand()
```

### Stop shooter
```java
// Stop shooter immediately
superstructure.stowCommand()

// Stop shooter and wait until fully stopped before continuing
superstructure.stowAndWaitCommand()
```

---

## BLOCK 4 — Intake Commands

```java
// Run intake until something interrupts it — use .withTimeout() in auto
intake.intake().withTimeout(2.0)        // run intake for up to 2 seconds
intake.intake().withTimeout(3.0)        // run intake for up to 3 seconds

// Eject a game piece
intake.eject().withTimeout(1.0)
```

---

## BLOCK 5 — Timing / Utility Commands

```java
// Wait for a number of seconds before continuing
Commands.waitSeconds(0.5)
Commands.waitSeconds(1.0)
Commands.waitSeconds(2.0)

// Print a message to the driver station console (useful for debugging)
Commands.print("Starting auto")
Commands.print("Arrived at scoring position")
Commands.print("Shot fired")
```

---

## BLOCK 6 — Command Combiners

These are not single commands — they wrap other blocks to change how they run.

### `Commands.sequence(...)` — run one at a time, in order
```java
// Each command runs after the previous one finishes
Commands.sequence(
    autoCommands.driveTo(Waypoints.SCORE_A),
    superstructure.speakerCloseAndWaitCommand(),
    superstructure.shootCommand(),
    superstructure.stowCommand()
)
```

### `Commands.parallel(...)` — run all at the same time, wait for all to finish
```java
// Drive and spin up happen simultaneously; sequence continues when BOTH finish
Commands.parallel(
    autoCommands.driveTo(Waypoints.SCORE_A),
    superstructure.speakerCloseCommand()
)
```

### `Commands.race(...)` — run all at the same time, stop when the FIRST one finishes
```java
// Drive to intake OR run intake for 3 seconds — whichever happens first
Commands.race(
    autoCommands.driveTo(Waypoints.INTAKE_1),
    intake.intake().withTimeout(3.0)
)
```

### `Commands.deadline(deadline, ...)` — run all until only the FIRST (deadline) finishes
```java
// Run intake the entire time we're driving, stop intake when drive finishes
Commands.deadline(
    autoCommands.driveTo(Waypoints.INTAKE_1),   // deadline — this one controls the end
    intake.intake()                              // runs until drive finishes
)
```

---

## BLOCK 7 — Waypoints Reference

All field positions available in `Waypoints.java`. Pass any of these to `driveTo()`.

### Starting Positions
```java
Waypoints.START_LEFT        // x=7.15, y=6.05,  heading=-120°
Waypoints.START_CENTER      // x=7.15, y=4.05,  heading=0°
Waypoints.START_RIGHT       // x=7.15, y=2.05,  heading=120°
```

### Game Piece Intake Positions
```java
Waypoints.INTAKE_1          // x=1.69, y=7.37,  heading=-60°
Waypoints.INTAKE_2          // x=1.28, y=7.07,  heading=-60°
Waypoints.INTAKE_3          // x=0.70, y=6.65,  heading=-60°

// Approach positions (safe stop before final intake)
Waypoints.INTAKE_1_APPROACH
Waypoints.INTAKE_2_APPROACH
Waypoints.INTAKE_3_APPROACH
```

### Scoring Positions
```java
Waypoints.SCORE_A           // x=3.16, y=4.20,  heading=0°
Waypoints.SCORE_B           // x=3.16, y=3.86,  heading=0°
Waypoints.SCORE_C           // x=3.67, y=2.95,  heading=60°
Waypoints.SCORE_D           // x=3.97, y=2.78,  heading=60°
Waypoints.SCORE_E           // x=5.02, y=2.95,  heading=120°
Waypoints.SCORE_F           // x=5.30, y=2.78,  heading=120°
Waypoints.SCORE_G           // x=5.83, y=3.86,  heading=180°
Waypoints.SCORE_H           // x=5.83, y=4.20,  heading=180°
Waypoints.SCORE_I           // x=5.30, y=5.10,  heading=-120°
Waypoints.SCORE_J           // x=5.02, y=5.26,  heading=-120°
Waypoints.SCORE_K           // x=3.97, y=5.26,  heading=-60°
Waypoints.SCORE_L           // x=3.67, y=5.10,  heading=-60°
```

### Navigation Waypoints
```java
Waypoints.INT_LEFT          // x=5.0, y=6.32 — safe pass-through on left side
Waypoints.INT_RIGHT         // x=5.0, y=1.83 — safe pass-through on right side
Waypoints.MIDFIELD_CENTER   // x=8.27, y=4.1 — center of field
```

### Custom Position
```java
// Build your own: new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(angle))
new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(0))
```

---

## Example Routines

These show how to assemble blocks into full autonomous routines.

### Leave Starting Zone
```java
Commands.sequence(
    autoCommands.resetPose(Waypoints.START_CENTER),
    autoCommands.driveTo(Waypoints.MIDFIELD_CENTER)
)
```

### Shoot and Leave
```java
Commands.sequence(
    autoCommands.resetPose(Waypoints.START_CENTER),
    superstructure.speakerCloseAndWaitCommand(),
    superstructure.shootCommand(),
    superstructure.stowCommand(),
    autoCommands.driveTo(Waypoints.MIDFIELD_CENTER)
)
```

### Shoot, Drive to Intake, Shoot Again
```java
Commands.sequence(
    autoCommands.resetPose(Waypoints.START_LEFT),
    // First shot — spin up while stationary, then fire
    superstructure.speakerCloseAndWaitCommand(),
    superstructure.shootCommand(),
    // Drive to intake position while running intake
    Commands.deadline(
        autoCommands.driveTo(Waypoints.INTAKE_1),
        intake.intake()
    ),
    Commands.waitSeconds(0.5),                      // brief pause to secure ball
    // Drive back to scoring, spin up on the way
    autoCommands.driveToWithAction(Waypoints.SCORE_A, superstructure.speakerCloseCommand()),
    Commands.waitSeconds(0.5),                      // let shooter finish spinning up
    superstructure.shootCommand(),
    superstructure.stowCommand()
)
```

### Spin Up While Driving (Time-Efficient)
```java
Commands.sequence(
    autoCommands.resetPose(Waypoints.START_CENTER),
    // Start spinning up immediately, drive and prep happen in parallel
    Commands.parallel(
        autoCommands.driveTo(Waypoints.SCORE_B),
        superstructure.speakerCloseCommand()
    ),
    Commands.waitSeconds(0.3),                      // let shooter finish if not quite there
    superstructure.shootCommand(),
    superstructure.stowCommand()
)
```

---

## BLOCK 8 — Vision Commands

> **Setup required** — Vision commands use `drivetrain` and `limelight`, which aren't in `AutoRoutines` by default.
> Add them to the constructor once, then all the blocks below will work:
>
> ```java
> // In AutoRoutines.java — add these two fields at the top:
> private final CommandSwerveDrivetrain drivetrain;
> private final LimelightSubsystem limelight;
>
> // Update the constructor signature:
> public AutoRoutines(AutoCommands autoCommands, Superstructure superstructure,
>                     CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight) {
>     this.autoCommands = autoCommands;
>     this.superstructure = superstructure;
>     this.drivetrain = drivetrain;
>     this.limelight = limelight;
> }
> ```
>
> Then update `RobotContainer.java` to pass them in:
> ```java
> autoRoutines = new AutoRoutines(autoCommands, superstructure, drivetrain, limelight);
> ```
>
> Add these imports at the top of `AutoRoutines.java`:
> ```java
> import frc.robot.commands.DriveAndLockCommand;
> import frc.robot.commands.DriveToTagCommand;
> import frc.robot.commands.FaceTagCommand;
> import frc.robot.subsystems.CommandSwerveDrivetrain;
> import frc.robot.subsystems.vision.LimelightSubsystem;
> ```

### Face the hub AprilTag (rotate in place)
```java
// Spin until the robot is facing the hub AprilTag — searchDirection: -1.0 = clockwise
new FaceTagCommand(drivetrain, limelight, -1.0).withTimeout(2.0)

// Search counter-clockwise instead
new FaceTagCommand(drivetrain, limelight, 1.0).withTimeout(2.0)
```

### Drive toward the nearest visible AprilTag
```java
// Drive toward the closest tag and stop at the configured approach distance
// Spins to search if no tag is visible — always use .withTimeout() in auto
new DriveToTagCommand(drivetrain, limelight).withTimeout(3.0)
```

### Wait until a tag is visible
```java
// Pause here until the limelight sees at least one AprilTag
Commands.waitUntil(() -> limelight.hasTarget())

// Same, but give up after 1.5 seconds if no tag appears
Commands.waitUntil(() -> limelight.hasTarget()).withTimeout(1.5)
```

### Shoot at auto-adjusted speed based on tag distance
```java
// Distance-based shooter speed — Limelight measures range and adjusts RPM automatically
// Use in place of speakerCloseAndWaitCommand() when using vision
superstructure.teleOpShootWithDistanceCommand(limelight::getAvgTagDistance).withTimeout(3.0)
```

### Align to hub tag, then shoot
```java
// Rotate to face hub, spin up once aligned, fire
Commands.sequence(
    new FaceTagCommand(drivetrain, limelight, -1.0).withTimeout(1.5),
    superstructure.speakerCloseAndWaitCommand(),
    superstructure.shootCommand(),
    superstructure.stowCommand()
)
```

### Drive toward tag, spin up on the way, then shoot
```java
// Approach the hub while spinning up, fire when close
Commands.sequence(
    Commands.parallel(
        new DriveToTagCommand(drivetrain, limelight).withTimeout(3.0),
        superstructure.speakerCloseCommand()
    ),
    Commands.waitSeconds(0.3),                      // let shooter finish spinning up
    superstructure.shootCommand(),
    superstructure.stowCommand()
)
```

---

## BLOCK 9 — Climb Commands

> **Setup required** — `climb` must be passed into `AutoRoutines` to use these blocks.
> Add it to the constructor the same way as `drivetrain`/`limelight` above:
>
> ```java
> // In AutoRoutines.java — add this field:
> private final Climb climb;
>
> // Update the constructor:
> public AutoRoutines(..., Climb climb) {
>     ...
>     this.climb = climb;
> }
> ```
>
> Update `RobotContainer.java`:
> ```java
> autoRoutines = new AutoRoutines(autoCommands, superstructure, drivetrain, limelight, climb);
> ```
>
> Add this import:
> ```java
> import frc.robot.subsystems.climb.Climb;
> ```

*Climb is an endgame mechanism. In auto, always add `.withTimeout()` so the command ends.*

```java
// Climb up for a set number of seconds
climb.climbUpCommand().withTimeout(2.0)

// Climb down for a set number of seconds
climb.climbDownCommand().withTimeout(1.5)
```

### Climb at the very end of auto
```java
// Drive to a safe position, then climb up in the final seconds
Commands.sequence(
    autoCommands.driveTo(Waypoints.MIDFIELD_CENTER),
    climb.climbUpCommand().withTimeout(2.0)
)
```
