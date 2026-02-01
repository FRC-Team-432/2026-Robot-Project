# PID Tuning Guide

**How to tune the arm (and other mechanisms) using Phoenix Tuner X.**

---

## CRITICAL WARNING

```
╔═══════════════════════════════════════════════════════════════════╗
║                                                                   ║
║   ⚠️  THE ARM PID VALUES ARE ALL 0.0 - IT WILL NOT MOVE!  ⚠️      ║
║                                                                   ║
║   Location: src/main/java/frc/robot/constants/ArmConstants.java   ║
║                                                                   ║
║   Current values:                                                 ║
║     kG = 0.0  (gravity compensation - NEEDS TUNING)               ║
║     kS = 0.0  (static friction - NEEDS TUNING)                    ║
║     kP = 0.0  (proportional gain - NEEDS TUNING)                  ║
║     kD = 0.0  (derivative gain - NEEDS TUNING)                    ║
║                                                                   ║
║   The arm WILL NOT MOVE until you tune these values!              ║
║                                                                   ║
╚═══════════════════════════════════════════════════════════════════╝
```

---

## What is PID?

PID stands for **Proportional-Integral-Derivative**. It's a control algorithm that smoothly moves a mechanism to a target position.

### The Problem PID Solves

```
Without PID:
"Go to position 45°!" → Motor turns full speed → SLAM! (overshoot)
                                               → Oscillates wildly

With PID:
"Go to position 45°!" → Motor turns fast at first
                      → Slows down as it approaches
                      → Stops smoothly at 45°
```

### PID Terms Explained

| Term | Symbol | What It Does | Too Low | Too High |
|------|--------|--------------|---------|----------|
| **Proportional** | kP | Correction strength based on error | Slow, never reaches target | Overshoot, oscillation |
| **Integral** | kI | Fixes steady-state error (usually 0) | Small persistent error | Overshoot, instability |
| **Derivative** | kD | Dampens oscillation, smooths motion | Overshoot | Slow response, vibration |

### Feedforward Terms (For Arms)

Arms fighting gravity need extra help:

| Term | Symbol | What It Does |
|------|--------|--------------|
| **Gravity** | kG | Counteracts gravity - holds arm in place |
| **Static Friction** | kS | Overcomes motor stiction (minimum power to move) |

```
ARM FORCE DIAGRAM:

      Target ↗        Without kG:        With kG:
            │         Arm sags down      Arm holds position
     ┌──────┼───┐     because gravity    because motor
     │      ●   │     wins               fights gravity
     │    ╱     │
     │  ╱       │     ↓↓↓ gravity       kG ↑↑↑ vs gravity
     └╱─────────┘
```

---

## Tools You Need

### 1. Phoenix Tuner X

Download from: [CTRE Phoenix Tuner X](https://store.ctr-electronics.com/software/)

Phoenix Tuner X lets you:
- See real-time motor data
- Change PID values on-the-fly
- Run characterization routines
- Test mechanisms safely

### 2. A Safe Testing Environment

**Before tuning:**
- Put the robot on blocks (wheels off ground)
- Remove game pieces that could fly out
- Have someone ready at the E-stop
- Clear the area around the mechanism

---

## Step-by-Step Arm Tuning

### Step 1: Connect to Robot

1. Turn on robot, connect to WiFi
2. Open Phoenix Tuner X
3. Select the arm motor (CAN ID 31)
4. Go to the **Configs** tab

### Step 2: Set Motion Magic Limits (FIRST!)

Before tuning PID, limit how fast the arm can move:

```
In Phoenix Tuner X → Configs → Motion Magic:

Cruise Velocity: 2.0 rotations/second   (start slow!)
Acceleration: 4.0 rotations/second²     (gentle start)
Jerk: 0 (unlimited, or try 40)
```

Then update ArmConstants.java:
```java
public static final double MOTION_MAGIC_CRUISE_VELOCITY = 2.0;
public static final double MOTION_MAGIC_ACCELERATION = 4.0;
```

### Step 3: Find kG (Gravity Compensation)

**Goal:** Find the motor output that holds the arm in place against gravity.

1. In Phoenix Tuner X → Control tab
2. Set control mode to **Duty Cycle** (manual power)
3. Move arm to horizontal position (hardest angle for gravity)
4. Slowly increase output until arm JUST holds its position
   - Too low: arm falls
   - Too high: arm rises
   - Just right: arm floats in place

**Typical value:** 0.05 to 0.30 depending on arm weight

```
ARM HORIZONTAL TEST:
───────────────────
          ↓ Gravity
    ┌─────●─────┐
    │           │
    │    ARM    │
    │           │
    ├───────────┤
    │   MOTOR   │ ← Apply power until arm floats
    └───────────┘
```

### Step 4: Find kS (Static Friction)

**Goal:** Find the minimum output to make the arm START moving.

1. With arm at horizontal, apply kG (from step 3)
2. Slowly add more power until arm JUST starts moving up
3. kS = (power to move) - kG

**Typical value:** 0.02 to 0.15

### Step 5: Tune kP (Proportional)

**Goal:** Make the arm respond to position errors.

1. Set kG and kS to values found above
2. In Phoenix Tuner X → Configs → Slot 0 (PID)
3. Start with kP = 50 (conservative)
4. Command the arm to a position
5. Observe behavior:
   - If slow/doesn't reach target → increase kP
   - If overshoots/oscillates → decrease kP

**Typical starting value:** 50-200 (depends on motor gearing)

### Step 6: Tune kD (Derivative)

**Goal:** Smooth out the motion and prevent overshoot.

1. With good kP set, add kD starting at kP/10
2. If kP = 100, try kD = 10
3. Command arm to move
4. Observe behavior:
   - If still overshooting → increase kD
   - If movement jerky/slow → decrease kD

**Typical ratio:** kD = kP/3 to kP/10

### Step 7: Test and Refine

1. Command arm to various positions
2. Watch for:
   - Overshoot (arm goes past target then back)
   - Oscillation (arm wobbles at target)
   - Slow approach (arm creeps toward target)
   - Steady-state error (arm stops before reaching target)

3. Adjust gains as needed

---

## Quick Reference: Starting Values

Copy these to ArmConstants.java as starting points:

```java
// STARTING VALUES - ADJUST ON REAL ROBOT!
public static final double kG = 0.15;  // Gravity compensation
public static final double kS = 0.05;  // Static friction
public static final double kP = 100.0; // Proportional gain
public static final double kD = 10.0;  // Derivative gain

public static final double MOTION_MAGIC_CRUISE_VELOCITY = 2.0;  // rot/s
public static final double MOTION_MAGIC_ACCELERATION = 4.0;     // rot/s²
```

---

## Tuning Troubleshooting

### Arm Falls When Enabled

**Cause:** kG too low
**Fix:** Increase kG until arm holds position

### Arm Rises When Trying to Hold

**Cause:** kG too high
**Fix:** Decrease kG

### Arm Oscillates at Target

**Cause:** kP too high and/or kD too low
**Fix:** Decrease kP, increase kD

### Arm Moves Slowly

**Cause:** kP too low
**Fix:** Increase kP

### Arm Overshoots Then Returns

**Cause:** kD too low
**Fix:** Increase kD (or decrease kP)

### Arm Never Reaches Target

**Cause:** kP too low, or kS too low
**Fix:** Increase kP. If still not moving, increase kS.

### Arm Jerks/Vibrates

**Cause:** kD too high, or Motion Magic too aggressive
**Fix:** Decrease kD, or lower acceleration

---

## Safety Tips

1. **Start with low gains** - You can always increase them
2. **Use Motion Magic limits** - Caps maximum speed
3. **Have E-stop ready** - Things can go wrong fast
4. **Test incrementally** - Change one value at a time
5. **Log everything** - Note what values you tried
6. **Put robot on blocks** - Wheels off ground for arm testing

---

## Phoenix Tuner X Cheat Sheet

| Location | What to Set |
|----------|-------------|
| Configs → Motor Output | Inverted, Neutral Mode |
| Configs → Feedback | Sensor type, direction |
| Configs → Slot 0 | kP, kI, kD gains |
| Configs → Slot 0 → Feedforward | kG, kS, kV, kA |
| Configs → Motion Magic | Cruise velocity, acceleration |
| Control → Position | Test target positions |
| Self-Test → Snapshot | Verify configuration |

---

## Other Mechanisms

### Shooter Flywheel

Flywheels use **velocity control** (maintain RPM), not position control.

Key values:
- **kV** (velocity feedforward) - Most important for flywheels
- **kP** - Corrects velocity errors
- **kS** - Overcomes friction

### Intake Roller

Simple mechanisms often use **duty cycle** (direct power) with no PID needed.

---

## Further Reading

- [CTRE Documentation](https://v6.docs.ctr-electronics.com/)
- [WPILib PID Tutorial](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html)
- [Motion Magic Documentation](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/motion-magic.html)

---

*FRC Team 432 - The Final Countdown*
