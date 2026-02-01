# FRC Team 432 - Student Guide

**Everything you need to know to upload, test, and run the robot code.**

---

## Table of Contents

1. [Quick Start](#quick-start)
2. [Running Simulation](#running-simulation)
3. [Deploying to Robot](#deploying-to-robot)
4. [Testing Checklist](#testing-checklist)
5. [What Works in Simulation](#what-works-in-simulation)
6. [Controller Layout](#controller-layout)
7. [Common Issues](#common-issues)

---

## Quick Start

### Prerequisites

1. **Install WPILib** - Download from [WPILib Installation Guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
2. **Install VS Code with WPILib Extension** - Comes with WPILib installer
3. **Install Git** - [git-scm.com](https://git-scm.com/)

### Clone the Repository

```bash
# Open terminal/command prompt
git clone https://github.com/your-team/2026-Robot-Project.git
cd 2026-Robot-Project
```

### Build the Code

```bash
# On Mac/Linux:
./gradlew build

# On Windows:
gradlew.bat build
```

**Expected output:**
```
BUILD SUCCESSFUL in Xs
```

### Common Build Errors

| Error | Solution |
|-------|----------|
| `JAVA_HOME not set` | Install WPILib (includes Java) |
| `Could not resolve dependencies` | Check internet connection, run `./gradlew --refresh-dependencies` |
| `Compilation failed` | Check error message for line number, fix syntax error |
| `Permission denied` (Mac/Linux) | Run `chmod +x gradlew` first |

---

## Running Simulation

Simulation lets you test code **without the physical robot**. This is great for:
- Testing logic before going to the shop
- Debugging at home
- Practicing driver controls

### Start Simulation

```bash
./gradlew simulateJava
```

This opens:
1. **Robot Simulation GUI** - Shows robot state
2. **Glass/SmartDashboard** - Shows telemetry data

### Using the Simulation

#### Driver Station Simulator

1. Open the **FRC Driver Station** (or use the simulation GUI)
2. Set the robot to **Teleop Enabled**
3. Connect a controller OR use keyboard

#### Keyboard Controls (No Controller)

In the Simulation GUI:
1. Go to **DS** → **Keyboard 0** → **Edit**
2. Map keys to joystick axes

#### What to Watch

Open **SmartDashboard** or **Shuffleboard** to see:
- `Drivetrain/Pose` - Robot position on field
- `Arm/Position` - Current arm angle
- `Shooter/Velocity` - Flywheel speed
- `AprilTagTracker/tagVisible` - Vision target status

### Simulation Tips

- **Reset pose**: Press Start button to reset robot position
- **Test buttons**: Make sure each button does what you expect
- **Check telemetry**: Watch for unexpected values
- **Test edge cases**: What happens when you release buttons?

---

## Deploying to Robot

### Step 1: Connect to Robot

1. **Turn on the robot** (main breaker)
2. **Wait for RoboRIO to boot** (~30 seconds)
3. **Connect to robot WiFi**
   - Network name: `432` or your team number
   - Password: (ask your mentor)

### Step 2: Verify Connection

Open a terminal and ping the RoboRIO:

```bash
ping roboRIO-432-FRC.local
```

Or use the IP address: `ping 10.4.32.2`

### Step 3: Deploy

```bash
./gradlew deploy
```

**Expected output:**
```
> Task :deployfrcJava
  Deploying to roboRIO at 10.4.32.2
  Deploy complete!

BUILD SUCCESSFUL
```

### Step 4: Enable Robot

1. Open **FRC Driver Station**
2. Wait for green connection status
3. Select **TeleOp** mode
4. Click **Enable**

### Deployment Troubleshooting

| Issue | Solution |
|-------|----------|
| `Connection refused` | Check WiFi, make sure robot is on |
| `Target not found` | Reboot RoboRIO, check network settings |
| `Deploy failed` | Check for code errors with `./gradlew build` first |
| `Robot doesn't respond` | Check Driver Station for errors, re-deploy |

### Emergency Stop!

**If something goes wrong:**
1. Press **SPACE** on the Driver Station (E-Stop)
2. OR press **Enter** to disable
3. OR flip the main breaker on the robot

---

## Testing Checklist

After deploying, test everything systematically:

### Drivetrain
- [ ] Left stick moves robot forward/backward
- [ ] Left stick strafes left/right
- [ ] Right stick rotates robot
- [ ] Slow mode works (hold RT)
- [ ] Vision lock works (hold LT) - *requires AprilTag in view*

### Targeting
- [ ] LB switches to Blue alliance targeting
- [ ] RB switches to Red alliance targeting
- [ ] Start button resets gyro heading
- [ ] Back button toggles field/robot centric

### Mechanisms (Operator Controller)
- [ ] LT runs intake IN
- [ ] LB runs intake OUT (eject)
- [ ] RT spins up shooter
- [ ] RB feeds balls to shooter

### Climb (STUBS - won't do anything yet)
- [ ] Y button - Flip Up (no response expected)
- [ ] A button - Flip Down (no response expected)
- [ ] X button - Lift Robot (no response expected)
- [ ] B button - Drop Robot (no response expected)

---

## What Works in Simulation

| Feature | Simulation Status | Notes |
|---------|------------------|-------|
| Swerve driving | ✅ **Works** | Full physics simulation |
| Arm movement | ✅ **Works** | ArmSIM class provides physics |
| Intake motor | ✅ **Works** | IntakeSIM shows velocity |
| Shooter flywheels | ✅ **Works** | ShooterSIM shows RPM |
| Feeder motor | ✅ **Works** | FeederSIM shows state |
| **Vision/AprilTags** | ❌ **NO** | No camera in simulation |
| **Climb** | ❌ **NO** | ClimbSIM not yet created |
| **Real PID response** | ⚠️ Approximate | Simulated physics differ from real |

### Important Notes

1. **Vision doesn't work in simulation**
   - `tracker.isTagVisible()` always returns `false`
   - Vision lock won't rotate the robot
   - Test vision features on the real robot only

2. **Climb is a stub**
   - Climb buttons are wired but do nothing
   - Implementation requires hardware

3. **PID tuning must be done on real robot**
   - Simulation physics are approximate
   - Use Phoenix Tuner X on the actual robot

---

## Controller Layout

### DRIVER (Port 0) - Controls Movement

```
         ┌──────────────────────────────┐
         │     [LB]           [RB]      │
         │   Blue Hub       Red Hub     │
         │                              │
   [LT]  │                              │  [RT]
  Vision │     [≡]           [☰]       │  Slow
  Lock   │   Start         Back         │  Mode
         │  Reset Gyro   Toggle F/R     │
         │                              │
         │    ┌───┐           ┌───┐     │
         │    │ L │  [Y]      │ R │     │
         │    │   │  Aim  [X] │   │     │
         │    └───┘       Dist└───┘     │
         │   Strafe  [A]      Rotate    │
         │         [B]                  │
         └──────────────────────────────┘

Left Stick: Move robot (forward/backward/strafe)
Right Stick: Rotate robot
LT: Vision Lock (auto-aim while driving)
RT: Slow mode (50% speed)
LB: Target Blue alliance hub
RB: Target Red alliance hub
Y: Auto-aim at tag (standalone)
X: Drive to medium distance from tag
Start: Reset gyro heading
Back: Toggle field-centric / robot-centric
```

### OPERATOR (Port 1) - Controls Mechanisms

```
         ┌──────────────────────────────┐
         │     [LB]           [RB]      │
         │  Intake OUT      Feeder      │
         │                              │
   [LT]  │                              │  [RT]
  Intake │     [≡]           [☰]       │  Shooter
   IN    │                              │  Spin Up
         │                              │
         │    ┌───┐           ┌───┐     │
         │    │   │  [Y]      │   │     │
         │    │   │ Flip Up   │   │     │
         │    └───┘ [X]   [B] └───┘     │
         │         Lift  Drop           │
         │          [A]                 │
         │        Flip Down             │
         │                              │
         │    D-pad: Tag ID cycling     │
         │    ↑ Next Tag  ↓ Prev Tag    │
         └──────────────────────────────┘

LT: Intake IN (pick up game pieces)
LB: Intake OUT (eject)
RT: Spin up shooter (hold before feeding!)
RB: Feed to shooter
Y/A/X/B: Climb controls (STUB)
D-pad Up/Down: Cycle tag IDs
```

---

## Common Issues

### "Robot won't move!"

1. Check Driver Station shows green communication
2. Verify robot is **Enabled** (not disabled)
3. Check joystick is detected in Driver Station
4. Try re-deploying code

### "Mechanism won't respond!"

1. Check CAN bus wiring
2. Look for error codes on motor controllers (blinking lights)
3. Check SmartDashboard for motor values
4. Verify correct CAN ID in code matches physical device

### "Vision lock doesn't rotate!"

1. Make sure an AprilTag is visible to the camera
2. Check `AprilTagTracker/tagVisible` in SmartDashboard
3. Verify Limelight is powered and connected
4. Vision does NOT work in simulation!

### "Build fails with weird errors!"

```bash
# Clean and rebuild:
./gradlew clean build

# If that doesn't work, delete gradle cache:
rm -rf ~/.gradle/caches
./gradlew build
```

### "Code deployed but robot acts weird!"

1. Check for runtime exceptions in Driver Station console
2. Re-deploy and watch for warnings
3. Check all motor CAN IDs match physical wiring
4. Verify motor directions are correct

---

## Next Steps

After mastering this guide:

1. **Read CONTROLLER_LAYOUT.md** - Detailed button reference
2. **Read SIMULATION_GUIDE.md** - More simulation tips
3. **Read PID_TUNING_GUIDE.md** - Before tuning the arm!
4. **Read TROUBLESHOOTING.md** - When things go wrong

---

*Last updated: 2026 Season*
*FRC Team 432 - The Final Countdown*
