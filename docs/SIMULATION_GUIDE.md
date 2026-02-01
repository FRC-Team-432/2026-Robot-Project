# Simulation Guide

**How to test your robot code without the physical robot.**

---

## What is Simulation?

WPILib provides a **physics simulation** that lets you run robot code on your computer. The simulated robot responds to joystick inputs and shows telemetry data just like the real thing.

### Why Simulate?

- **Test at home** - No need to be at the shop
- **Faster iteration** - No deploy time, instant testing
- **Debug safely** - Can't break anything!
- **Practice driving** - Train muscle memory

---

## Starting Simulation

### Basic Command

```bash
./gradlew simulateJava
```

This opens several windows:
1. **Robot Simulation** - Main control window
2. **Glass/SmartDashboard** - Telemetry display
3. **Field2d** - Optional field visualization

### What You'll See

```
┌─────────────────────────────────────────────┐
│         Robot Simulation Window             │
├─────────────────────────────────────────────┤
│                                             │
│  Robot State: [Disabled ▼]                  │
│                                             │
│  DS: ● Connected                            │
│                                             │
│  ┌──────────────────┐                       │
│  │   Joysticks      │                       │
│  │   [0] Xbox       │  ← Your controller    │
│  │   [1] Xbox       │                       │
│  └──────────────────┘                       │
│                                             │
│  Timing: 20ms loop                          │
│                                             │
└─────────────────────────────────────────────┘
```

---

## Simulation Feature Matrix

### What Works

| Feature | Status | Notes |
|---------|--------|-------|
| **Swerve Drivetrain** | ✅ Full | CTRE Phoenix 6 simulation |
| **Arm** | ✅ Full | ArmSIM provides physics model |
| **Intake** | ✅ Full | IntakeSIM shows motor velocity |
| **Shooter** | ✅ Full | ShooterSIM shows flywheel RPM |
| **Feeder** | ✅ Full | FeederSIM shows feeding state |
| **Joystick Input** | ✅ Full | Physical controller or keyboard |
| **Field Position** | ✅ Full | Odometry updates correctly |
| **Telemetry/Logging** | ✅ Full | All @Logged fields visible |

### What Does NOT Work

| Feature | Status | Why |
|---------|--------|-----|
| **AprilTag Vision** | ❌ None | No camera in simulation |
| **Limelight** | ❌ None | No camera hardware |
| **Vision Lock** | ❌ None | Depends on vision |
| **Climb** | ❌ None | No ClimbSIM class exists |
| **Real PID Response** | ⚠️ Approximate | Simulated physics differ |

### Important Limitations

```
┌─────────────────────────────────────────────────────────────┐
│                    ⚠️  WARNING  ⚠️                           │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  VISION DOES NOT WORK IN SIMULATION                         │
│                                                             │
│  • tracker.isTagVisible() always returns FALSE              │
│  • Vision lock (LT) will NOT rotate the robot               │
│  • AprilTag distance/angle data unavailable                 │
│                                                             │
│  To test vision: You MUST use the real robot with a         │
│  Limelight camera and actual AprilTags!                     │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## Using Controllers in Simulation

### Physical Xbox Controller

1. Plug in your Xbox controller via USB
2. Start simulation: `./gradlew simulateJava`
3. Controller should appear as "Joystick 0" or "Joystick 1"
4. Enable the robot and test!

### Keyboard as Controller

If you don't have a controller:

1. In the simulation window, go to **DS** → **Keyboard 0**
2. Click **Edit** to map keys to axes/buttons

**Suggested Keyboard Mapping:**

| Key | Maps To | Function |
|-----|---------|----------|
| W/S | Left Y Axis | Forward/Backward |
| A/D | Left X Axis | Strafe Left/Right |
| Q/E | Right X Axis | Rotate |
| Space | Button A | (varies) |
| Enter | Enable/Disable | Robot state |

### Multiple Controllers

For full two-controller testing:
- **Joystick 0** = Driver controller
- **Joystick 1** = Operator controller

You can use one physical controller and one keyboard, or two physical controllers.

---

## Viewing Telemetry

### SmartDashboard

Open **SmartDashboard** (comes with WPILib):
```
Tools → FRC SmartDashboard
```

Or use **Shuffleboard** for better layouts:
```
Tools → Shuffleboard
```

### Key Telemetry Values

| Path | What It Shows |
|------|---------------|
| `Drivetrain/pose` | Robot X, Y, rotation on field |
| `Drivetrain/speeds` | Current velocity |
| `Arm/position` | Arm angle in degrees |
| `Arm/velocity` | Arm movement speed |
| `Shooter/velocity` | Flywheel RPM |
| `Intake/velocity` | Intake roller speed |
| `AprilTagTracker/tagVisible` | Always false in sim |
| `AprilTagTracker/horizontalOffset` | 0 in sim |

### Glass (Advanced Visualization)

Glass provides more detailed views:

1. In simulation window: **NetworkTables** → **Open Glass**
2. Add plots for real-time graphing
3. Use Field2d widget for robot position

---

## Testing Strategy

### What to Test in Simulation

1. **Button bindings** - Every button does what you expect
2. **Drive response** - Smooth, no jerking
3. **Mechanism timing** - Shooter spins up, feeder works
4. **Command logic** - Commands start/stop correctly
5. **Autonomous** - Path following (without vision)

### What to Test on Real Robot

1. **Vision targeting** - Requires camera + tags
2. **PID tuning** - Simulation physics differ
3. **Motor directions** - Verify correct rotation
4. **Sensor readings** - Limit switches, encoders
5. **Full system integration** - Everything together

### Testing Checklist

```
SIMULATION TESTING CHECKLIST
═══════════════════════════════════════

DRIVETRAIN
[ ] Left stick moves robot smoothly
[ ] Right stick rotates robot
[ ] Slow mode (RT) reduces speed
[ ] Field-centric feels intuitive
[ ] Robot-centric toggle works

MECHANISMS
[ ] Intake runs when LT held
[ ] Intake reverses with LB
[ ] Shooter spins up with RT
[ ] Feeder runs with RB
[ ] Shooter + Feeder together works

TELEMETRY
[ ] Pose updates while driving
[ ] Arm position shows correctly
[ ] Motor velocities display
[ ] No error messages in console

BUTTONS
[ ] All driver buttons respond
[ ] All operator buttons respond
[ ] Start resets gyro heading
```

---

## Simulation Tips

### 1. Reset Between Tests

Press **Start** button to reset robot heading, or restart simulation for full reset.

### 2. Watch the Console

Error messages appear in the terminal where you ran `./gradlew simulateJava`. Keep an eye on it!

### 3. Use Print Statements

Add `System.out.println()` to debug:

```java
@Override
public void execute() {
    System.out.println("Command running, value: " + someValue);
    // ...
}
```

### 4. Test Edge Cases

- What happens when you release a button mid-action?
- What if you press two conflicting buttons?
- What if values are at limits (0, max)?

### 5. Simulate Failures

- What if a motor doesn't respond? (Comment it out temporarily)
- What if a sensor reads wrong? (Hardcode a bad value)

---

## Troubleshooting Simulation

### "Simulation won't start"

```bash
# Clean and retry
./gradlew clean simulateJava

# Check for errors in build
./gradlew build
```

### "Controller not detected"

1. Plug in controller BEFORE starting simulation
2. Check controller works in OS settings
3. Try a different USB port

### "Robot doesn't respond to inputs"

1. Make sure robot is **Enabled** in DS panel
2. Check joystick index matches code (0 for driver, 1 for operator)
3. Verify button mappings in code

### "Weird physics behavior"

Simulation physics are approximate. If something looks wrong in sim but works on the real robot, trust the real robot. Report major discrepancies.

### "Glass/SmartDashboard won't open"

```bash
# Run Glass directly (in WPILib tools folder)
# Or restart simulation - it should auto-open
```

---

## Advanced: Simulation Classes

Our codebase uses the "SIM pattern" for hardware abstraction:

```java
// In RobotContainer.java:
public final Arm arm = RobotBase.isSimulation() ? new ArmSIM() : new Arm();
```

This automatically uses:
- **Arm.java** on real robot (talks to actual motors)
- **ArmSIM.java** in simulation (models physics)

### Available SIM Classes

| Subsystem | Real Class | SIM Class |
|-----------|-----------|-----------|
| Arm | Arm.java | ArmSIM.java |
| Intake | Intake.java | IntakeSIM.java |
| Shooter | Shooter.java | ShooterSIM.java |
| Feeder | Feeder.java | FeederSIM.java |
| Climb | Climb.java | ❌ Not yet! |

---

*FRC Team 432 - The Final Countdown*
