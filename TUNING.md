# Robot Tuning & Competition-Day Setup Guide

This document tracks every value in the codebase that needs to be set, verified, or tuned
once real hardware is available. It is organized by priority — work top to bottom.

---

## STEP 1 — Hardware Setup (do before powering any motor)

These are IDs and names that must match the physical hardware. Wrong values here will
cause the robot to control the wrong motor or not find devices at all.

### CAN IDs

| Constant | File | Current Value | What to do |
|---|---|---|---|
| `SHOOTER_LEADER_ID` | `ShooterConstants.java` | `20` | Set to actual CAN ID of shooter motor 1 |
| `SHOOTER_FOLLOWER_ID` | `ShooterConstants.java` | `21` | Set to actual CAN ID of shooter motor 2 |
| `FEEDER_ID` | `ShooterConstants.java` | `22` | Set to actual CAN ID of the feeder motor |
| `CLIMB_MOTOR_ID` | `ClimbConstants.java` | `30` | Set to actual CAN ID of the climb motor |

> **Note:** All shooter, feeder, and climb motors are on the **RoboRIO CAN bus**.
> The swerve drivetrain is on the **CANivore** bus and is configured separately via Tuner X.

### Limelight Camera Name

| Location | Current Value | What to do |
|---|---|---|
| `RobotContainer.java` | `"limelight"` | Must exactly match the hostname set in the Limelight web UI. |

---

## STEP 2 — Climb Tuning

The climb uses open-loop duty cycle control — no PID needed. Brake mode holds position
when buttons are released.

**File:** `ClimbConstants.java`

| Constant | Current Value | What to do |
|---|---|---|
| `CLIMB_UP_SPEED` | `0.5` (50%) | Increase if climb is too slow, decrease if it's too fast or strains the motor |
| `CLIMB_DOWN_SPEED` | `0.3` (30%) | Usually lower than up speed — gravity helps on the way down |

> **Controls:** Y button = climb up, A button = climb down. Motor brakes and holds
> position the moment either button is released.

---

## STEP 3 — Shooter Tuning

### Shooter Speeds
**File:** `ShooterConstants.java`

| Constant | Current Value | What to do |
|---|---|---|
| `SHOOTER_SPEED_RPS` | `25.0` | Start low (~15 RPS), increase until balls reach target consistently |
| `FEEDER_SPEED_PERCENT` | `0.5` | Run feeder until balls feed smoothly without jamming or slipping |

### Shooter PID Gains
**File:** `ShooterConstants.java`

| Constant | Current Value | Suggested Start | What it does |
|---|---|---|---|
| `kV` | `0.125` | Keep as-is initially | Velocity feedforward — predicts voltage for a given speed |
| `kS` | `0.0` | `0.1–0.3` | Static friction — helps shooter spin up from rest |
| `kP` | `0.0` | `0.1–0.5` | Corrects speed errors — raise if shooter consistently runs slow/fast |

**Tuning order:** `kV` is usually sufficient to start. Add `kS` if the shooter stalls at low speeds, then `kP` if speed is inconsistent under load.

### Auto Fire Duration
| Constant | File | Current Value | What to do |
|---|---|---|---|
| `SHOOT_DURATION_SECONDS` | `ShooterConstants.java` | `0.5 s` | How long the feeder runs per shot in auto. Shorten if fast, lengthen if ball doesn't fully exit |

### Shooter Speed Tolerance
| Constant | File | Current Value | Notes |
|---|---|---|---|
| `VELOCITY_TOLERANCE_RPS` | `ShooterConstants.java` | `1.0` | Tighten for more consistent shots, loosen if auto sequences stall waiting for shooter |

---

## STEP 4 — Distance-Based Shooting (Bonus Feature)

**File:** `ShooterConstants.java` — `DISTANCE_SPEED_MAP`

Currently all placeholder values. Tune after basic shooting is working.

| Distance | Current Speed | How to tune |
|---|---|---|
| 1.0 m | 20.0 RPS | Place robot 1 m from target. Increase speed until ball reaches. |
| 2.0 m | 25.0 RPS | Place robot 2 m from target. Tune until ball reaches. |
| 3.0 m | 30.0 RPS | Repeat at each distance. |
| 4.0 m | 35.0 RPS | Add more rows for better accuracy across the field. |
| 5.0 m | 40.0 RPS | |

> To switch from fixed-speed to distance-based shooting, swap the commented line in
> `RobotContainer.java` inside `configureBindings()`.

---

## STEP 5 — Vision (Limelight)

### Limelight Web UI Configuration
These are set in the Limelight web interface (not in code), but must be correct for
AprilTag detection and distance measurements to work.

| Setting | What to configure |
|---|---|
| **Camera name** | Must match `"limelight"` in `RobotContainer.java` |
| **Pipeline** | Set pipeline 0 to AprilTag detection mode |
| **Camera tilt angle** | Enter the physical mounting angle of the camera (affects distance accuracy) |
| **Camera height** | Enter the physical mounting height in meters (affects distance accuracy) |
| **AprilTag layout** | Make sure the field layout matches the current season |
| **Network settings** | Assign a static IP; confirm it's reachable at `limelight.local` |

### Vision Trust Constants
**File:** `VisionConstants.java`

These control how much the drivetrain trusts the Limelight over wheel odometry.

| Constant | Current Value | Notes |
|---|---|---|
| `BASE_XY_STD_DEV` | `0.5 m` | Lower = trust camera more for X/Y. Raise if robot position jumps erratically. |
| `BASE_THETA_STD_DEV` | `5 rad` | Lower = trust camera more for rotation. Usually left high (gyro handles rotation well). |
| `MAX_TAG_AMBIGUITY` | `0.7` | Lower if you get ghost readings from a single tag. Raise if valid readings are being rejected. |

---

## STEP 6 — Autonomous

**File:** `AutoConstants.java`

| Constant | Current Value | What to do |
|---|---|---|
| `ROBOT_MASS_LBS` | `125.0` | Update to actual weight **after weigh-in** |
| `MOMENT_OF_INERTIA_KG_M2` | `6.0` | Estimate based on robot geometry; leave at 6 if unknown |
| `MAX_LINEAR_VELOCITY_MPS` | `3.0 m/s` | Raise for faster autos, lower if robot skips or misses waypoints |
| `MAX_LINEAR_ACCELERATION_MPS2` | `3.0 m/s²` | Lower if robot tips or wheels slip during acceleration |
| `X_CONTROLLER_KP` | `10.0` | Lower if robot oscillates around waypoints during auto |
| `Y_CONTROLLER_KP` | `10.0` | Same as above |
| `THETA_CONTROLLER_KP` | `7.0` | Lower if robot spins past heading targets |

---

## Quick-Reference Checklist (Competition Day)

Copy and use this as a checklist before each event:

```
[ ] Verify all CAN IDs match physical hardware (Tuner X → Devices tab)
[ ] Verify Limelight name matches RobotContainer ("limelight")
[ ] Limelight pipeline is set to AprilTag mode
[ ] Limelight can see and identify tags at competition field
[ ] Climb holds position when Y/A buttons released (brake mode working)
[ ] Climb goes up with Y, down with A — check direction is correct
[ ] Shooter spins up to target speed (check in AdvantageScope or SmartDashboard)
[ ] Feeder moves balls into shooter without jamming
[ ] Right trigger fires both shooter wheels and feeder simultaneously
[ ] Releasing trigger stops all three motors immediately
[ ] Auto routine drives the correct path (run on practice field first)
[ ] Robot weight entered in AutoConstants after weigh-in
[ ] Distance-speed map tested if using distance-based shooting
```

---

## File Map

| What you're tuning | File to edit |
|---|---|
| Shooter speeds, CAN IDs, feeder speed, distance table | `src/main/java/frc/robot/constants/ShooterConstants.java` |
| Climb CAN ID and speeds | `src/main/java/frc/robot/constants/ClimbConstants.java` |
| Vision trust levels, ambiguity filter | `src/main/java/frc/robot/constants/VisionConstants.java` |
| Auto speed limits, robot mass | `src/main/java/frc/robot/constants/AutoConstants.java` |
| Controller bindings, distance-mode toggle | `src/main/java/frc/robot/RobotContainer.java` |
| Limelight name | `src/main/java/frc/robot/RobotContainer.java` |
