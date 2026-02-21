# Robot Code Plan — FRC Team 432 (2026)

> **Status:** Approved, not yet implemented.
> No code has been changed. This is a reference document for the upcoming build sessions.

---

## What Needs to Be Done (Summary)

| Task | Session |
|------|---------|
| **Limelight debugging (confirm tags are being seen)** | **Session 1 (Sat) — first** |
| Delete Arm, Flywheel, Superstructure + their constants | Session 1 (Sat) |
| Create IntakeConstants, ShooterConstants, ClimberConstants | Session 1 (Sat) |
| Create Intake subsystem (1 motor) | Session 1 (Sat) |
| Create Shooter subsystem (3 motors) | Session 1 (Sat) |
| Create Climber subsystem (1 motor) | Session 1 (Sat) |
| Update RobotContainer — operator controller + new bindings | Session 1 (Sat) |
| Hardware install, deploy, first motor tests | Session 2 (Tue) |
| Tune shooter speed, intake speed, climber limits | Session 3 (Wed) |
| Snap-to-tag confirm + auto routines | Session 3 (Wed) |
| Full drive cycle, polish, stretch goals | Session 4 (Sat) |

---

## Session 1 — Saturday (9–3): Vision First, Then Code Sprint

**Goal:** Verify the Limelight is working since it's already connected to the robot. Then write all new subsystems, remove old ones, and wire up controllers.

### Step 1 — Limelight / Vision System (do this first — robot is available)

The Limelight is the only thing currently hooked up to the robot, so confirm it works before diving into code.

1. Deploy current code to the robot: `./gradlew deploy`
2. Open Shuffleboard — look for `Limelight/HasTarget`
3. Point the camera at an AprilTag manually — does `HasTarget` go `true`?
4. If yes: test driver **Left Bumper** → robot should rotate toward the tag
5. If no target is detected, check:
   - Camera power and cable connection
   - Limelight web UI at `http://limelight.local:5801` (or `http://10.4.32.11:5801`)
   - Pipeline is set to AprilTag mode (not retroreflective or neural)
   - Tag is within camera field of view and lit well enough

Once the Limelight is confirmed working (or diagnosed), move on to the code sprint below.

---

### Step 2 — Delete old code that doesn't match this robot

Files to delete:
- `src/main/java/frc/robot/subsystems/arm/Arm.java`
- `src/main/java/frc/robot/subsystems/arm/ArmSIM.java`
- `src/main/java/frc/robot/subsystems/flywheel/Flywheel.java`
- `src/main/java/frc/robot/subsystems/flywheel/FlywheelSIM.java`
- `src/main/java/frc/robot/subsystems/Superstructure.java`
- `src/main/java/frc/robot/constants/ArmConstants.java`
- `src/main/java/frc/robot/constants/FlywheelConstants.java`

Also: clean up the commented-out Arm/Flywheel/Superstructure lines in `RobotContainer.java`.

---

### Step 3 — Confirm CAN IDs with electrical team

> **Do this before writing constants.** Talk to whoever is wiring the robot.

Suggested CAN IDs (all on default CAN bus `""`):

| Device | CAN ID |
|--------|--------|
| Intake Motor | 36 |
| Shooter Top Motor | 37 |
| Shooter Bottom Motor | 38 |
| Shooter Feeder Motor | 39 |
| Climber Motor | 40 |

If any motor ends up on the `"canivore"` bus, update the bus string in that subsystem's constants.

---

### Step 4 — Create `IntakeConstants.java`

**Path:** `src/main/java/frc/robot/constants/IntakeConstants.java`

```java
public final class IntakeConstants {
  public static final int    CAN_ID              = 36;
  public static final String CAN_BUS             = "";
  public static final double INTAKE_SPEED_RPS    = 30.0; // tune on hardware
  public static final double EJECT_SPEED_RPS     = -20.0; // tune on hardware
  public static final double kS                  = 0.0;
  public static final double kV                  = 0.125;
  public static final double kP                  = 0.0;
  public static final double VELOCITY_TOLERANCE_RPS = 1.0;
}
```

---

### Step 5 — Create `ShooterConstants.java`

**Path:** `src/main/java/frc/robot/constants/ShooterConstants.java`

```java
public final class ShooterConstants {
  public static final int    TOP_MOTOR_CAN_ID    = 37;
  public static final int    BOTTOM_MOTOR_CAN_ID = 38;
  public static final int    FEEDER_MOTOR_CAN_ID = 39;
  public static final String CAN_BUS             = "";
  public static final double SHOOTING_SPEED_RPS  = 60.0; // tune on hardware
  public static final double FEEDER_SPEED_DUTY_CYCLE = 0.5; // 0.0–1.0, tune on hardware
  public static final double kS                  = 0.0;
  public static final double kV                  = 0.125;
  public static final double kP                  = 0.0;
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 120.0;
  public static final double MOTION_MAGIC_ACCELERATION    = 500.0;
  public static final double VELOCITY_TOLERANCE_RPS       = 1.0;
}
```

---

### Step 6 — Create `ClimberConstants.java`

**Path:** `src/main/java/frc/robot/constants/ClimberConstants.java`

```java
public final class ClimberConstants {
  public static final int    CAN_ID              = 40;
  public static final String CAN_BUS             = "";
  public static final double CLIMB_SPEED_PERCENT = 0.5;   // tune on hardware
  public static final double RETRACT_SPEED_PERCENT = -0.5; // tune on hardware
  // Set these after measuring how far the climber physically travels:
  public static final double FORWARD_SOFT_LIMIT_ROTATIONS = 50.0; // TODO: measure
  public static final double REVERSE_SOFT_LIMIT_ROTATIONS = 0.0;
}
```

---

### Step 7 — Create `Intake.java`

**Path:** `src/main/java/frc/robot/subsystems/intake/Intake.java`

Pattern to follow: `Flywheel.java` (single TalonFX, velocity control)

Key methods:
- `intakeCommand()` — spin at `INTAKE_SPEED_RPS` (whileTrue binding)
- `ejectCommand()` — spin at `EJECT_SPEED_RPS` (whileTrue binding)
- `stopCommand()` — stop motor
- `getVelocity()` — for telemetry

Use `TalonFXUtil.applyConfigWithRetries()` for motor config. Add `@Logged` annotation.

---

### Step 8 — Create `Shooter.java`

**Path:** `src/main/java/frc/robot/subsystems/shooter/Shooter.java`

Three motors:
- **Top shooter** — TalonFX, MotionMagic velocity control
- **Bottom shooter** — TalonFX, MotionMagic velocity control
- **Feeder** — TalonFX, DutyCycleOut (simple percent output)

Key methods:
- `spinUpCommand()` — spin top + bottom to `SHOOTING_SPEED_RPS` (does NOT run feeder yet)
- `runFeederCommand()` — run feeder at `FEEDER_SPEED_DUTY_CYCLE`
- `stopCommand()` — stop all three motors
- `isAtSpeed()` — true when both shooters are within `VELOCITY_TOLERANCE_RPS`
- `getTopVelocity()`, `getBottomVelocity()` — for telemetry

The **right trigger binding** in RobotContainer will use:
```java
operatorController.rightTrigger(0.1).whileTrue(
  Commands.sequence(
    shooter.spinUpCommand(),
    Commands.waitUntil(shooter::isAtSpeed),
    shooter.runFeederCommand()
  ).finallyDo(() -> shooter.stopAll())
);
```
This spins up wheels, waits until at speed, then feeds the ball. Releasing the trigger stops everything.

Add `@Logged` annotation.

---

### Step 9 — Create `Climber.java`

**Path:** `src/main/java/frc/robot/subsystems/climber/Climber.java`

Single TalonFX using `DutyCycleOut` (simple percent output — easiest for a first pass).

Key methods:
- `climbCommand()` — run at `CLIMB_SPEED_PERCENT`
- `retractCommand()` — run at `RETRACT_SPEED_PERCENT`
- `stopCommand()` — stop motor

Soft limits configured in motor config to prevent over-travel. Add `@Logged` annotation.

---

### Step 10 — Update `RobotContainer.java`

Changes needed:
1. **Remove** all Arm/Flywheel/Superstructure imports and references
2. **Add** operator controller: `CommandXboxController operatorController = new CommandXboxController(1);`
3. **Instantiate** new subsystems: `Intake intake`, `Shooter shooter`, `Climber climber`
4. **Add operator bindings:**

```
Left Trigger  (operator) → intake wheels run while held
Left Bumper   (operator) → intake wheels reverse (eject) while held
Right Trigger (operator) → shooter spins up, waits for speed, feeds ball while held
[TBD button]  (operator) → climber extends while held   ← TODO: assign when robot is built
```

5. **Keep** driver Left Bumper → DriveToTagCommand (snap-to-tag, already works)
6. **Keep** driver Start → reset pose

---

## Session 2 — Tuesday (1:30–8): Hardware Install + First Tests

**Goal:** Motors physically installed and connected. Confirm everything runs before tuning.

1. Run `./gradlew build` on laptop before arriving — fix any compile errors from session 1
2. Deploy to robot — check DriverStation for CAN faults (wrong IDs = yellow triangle)
3. **Test intake:** Left trigger → motor spins. If backwards, add `.withInverted(true)` in `Intake.java`
4. **Test shooter:** Right trigger → wheels spin up, then feeder runs. Adjust `SHOOTING_SPEED_RPS`
5. **Test climber:** Assigned button → motor extends. Set soft limits after measuring travel distance
6. **Limelight snap-to-tag:** Confirm driver Left Bumper → robot rotates toward tag (vision confirmed in session 1)

---

## Session 3 — Wednesday (1:30–8): Tuning

**Goal:** Get all mechanisms working reliably. Enable auto.

1. Tune `SHOOTING_SPEED_RPS` — increase until balls reach target distance consistently
2. Tune `INTAKE_SPEED_RPS` — verify reliable ball pickup without jamming
3. Tune `CLIMB_SPEED_PERCENT` and `FORWARD_SOFT_LIMIT_ROTATIONS` after measuring travel
4. Limelight snap-to-tag fine-tuning — adjust `DRIVE_TO_TAG_TURN_KP` in `VisionConstants.java` if robot over/under-rotates
5. Re-enable `AutoRoutines` in `RobotContainer` — write a simple mobility auto using `autoCommands.driveTo()`

---

## Session 4 — Saturday 2 (9–3): Polish + Stretch Goals

**Goal:** Full match practice, stretch features if time allows.

1. Full drive cycle — driver + operator run through match scenario start to finish
2. Assign climber button (should be decided by now)
3. **Stretch goal — Heading Lock while driving:**
   - New command: `TagHeadingLockCommand.java`
   - Driver can still translate (left stick) but robot stays pointed at tag (rotation = P controller on limelight TX)
   - Bind to driver Right Bumper (whileTrue)
4. Add shoot-and-drive auto sequence if mechanisms are reliable
5. Final cleanup — verify Shuffleboard looks correct, remove any debug logging

---

## Files Changed (Summary)

### Delete
- `subsystems/arm/Arm.java`
- `subsystems/arm/ArmSIM.java`
- `subsystems/flywheel/Flywheel.java`
- `subsystems/flywheel/FlywheelSIM.java`
- `subsystems/Superstructure.java`
- `constants/ArmConstants.java`
- `constants/FlywheelConstants.java`

### Create
- `constants/IntakeConstants.java`
- `constants/ShooterConstants.java`
- `constants/ClimberConstants.java`
- `subsystems/intake/Intake.java`
- `subsystems/shooter/Shooter.java`
- `subsystems/climber/Climber.java`
- *(stretch)* `commands/TagHeadingLockCommand.java`

### Modify
- `RobotContainer.java` — remove old, add operator controller + new subsystems + bindings
- `autonomous/AutoRoutines.java` — remove Superstructure dependency, add new subsystem auto commands

---

## Controller Layout (Final)

### Driver (Port 0 — Xbox)
| Input | Action |
|-------|--------|
| Left Stick | Drive (field-centric) |
| Right Stick X | Rotate |
| Left Bumper | Snap to AprilTag (while held) |
| Start | Reset pose to (0,0) |
| *(stretch)* Right Bumper | Heading lock on tag while driving |

### Operator (Port 1 — Xbox)
| Input | Action |
|-------|--------|
| Left Trigger | Run intake (while held) |
| Left Bumper | Reverse intake / eject (while held) |
| Right Trigger | Shoot (spin up → wait for speed → feed ball) |
| **TBD** | Climber extend (while held) |

---

## Key Notes

- **CAN IDs 36–40 are suggestions** — confirm with electrical before session 2 deploy
- **Motor direction** will likely need `.withInverted(true)` on some motors — quick fix during session 2 testing
- **Feeder is "wait for speed"** — wheels reach `SHOOTING_SPEED_RPS` before feeder activates
- **Limelight debug:** `Limelight/HasTarget` on Shuffleboard is the first thing to check
- **`TalonFXUtil.applyConfigWithRetries()`** must be used for all motor configs (project standard)
- **`@Logged` annotation** goes on every new subsystem class for automatic telemetry
