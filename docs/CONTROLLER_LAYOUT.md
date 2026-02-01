# Controller Layout Reference

**Complete button mapping for both controllers.**

---

## Overview

We use **two Xbox controllers** for better control division:

| Controller | Port | Role |
|------------|------|------|
| **Driver** | 0 | Robot movement, vision targeting, alliance selection |
| **Operator** | 1 | Mechanisms (intake, shooter, feeder, climb) |

---

## Driver Controller (Port 0)

The driver focuses on **WHERE the robot goes**.

### Visual Layout

```
                    DRIVER CONTROLLER
    ┌──────────────────────────────────────────────┐
    │                                              │
    │      [LB]                        [RB]        │
    │    BLUE HUB                    RED HUB       │
    │    Target blue                Target red     │
    │    alliance tags             alliance tags   │
    │                                              │
    │  [LT]                              [RT]      │
    │  VISION LOCK                    SLOW MODE    │
    │  Auto-aim while                 50% speed    │
    │  driving (hold)                for precision │
    │                                              │
    │         ╔═══════════════════╗                │
    │         ║ [BACK]   [START]  ║                │
    │         ║ Toggle    Reset   ║                │
    │         ║ F/R Centric Gyro  ║                │
    │         ╚═══════════════════╝                │
    │                                              │
    │    ╭─────╮                    ╭─────╮        │
    │    │     │         [Y]       │     │        │
    │    │  L  │       AUTO-AIM    │  R  │        │
    │    │     │                   │     │        │
    │    ╰──┬──╯    [X]     [B]    ╰──┬──╯        │
    │       │      DRIVE             │            │
    │    STRAFE   TO DIST    [A]  ROTATE          │
    │    Move                                      │
    │    left/right                               │
    │    and fwd/back                             │
    │                                              │
    └──────────────────────────────────────────────┘
```

### Button Reference

| Button | Action | Details |
|--------|--------|---------|
| **Left Stick** | Strafe/Move | Forward, backward, left, right movement |
| **Right Stick** | Rotate | Spin the robot clockwise/counter-clockwise |
| **Left Trigger (LT)** | Vision Lock | Hold to auto-aim at alliance hub while driving |
| **Right Trigger (RT)** | Slow Mode | Hold for 50% speed (precision control) |
| **Left Bumper (LB)** | Target Blue | Set vision to track blue alliance hub |
| **Right Bumper (RB)** | Target Red | Set vision to track red alliance hub |
| **Y Button** | Auto-Aim | Standalone rotation to center on tag |
| **X Button** | Drive to Distance | Drive to medium distance from tag |
| **Start** | Reset Gyro | Make current direction "forward" |
| **Back** | Toggle Drive Mode | Switch field-centric / robot-centric |

### Drive Modes Explained

**Field-Centric (Default)**
- "Forward" always means toward the **far end of the field**
- Robot can face any direction, controls stay the same
- **Recommended for competition**

**Robot-Centric**
- "Forward" means the **front of the robot**
- If robot is facing right, pushing up moves right
- Useful for precise maneuvers

---

## Operator Controller (Port 1)

The operator focuses on **WHAT the robot does**.

### Visual Layout

```
                   OPERATOR CONTROLLER
    ┌──────────────────────────────────────────────┐
    │                                              │
    │      [LB]                        [RB]        │
    │   INTAKE OUT                    FEEDER       │
    │   Eject game                  Feed balls     │
    │   pieces                     to shooter      │
    │                                              │
    │  [LT]                              [RT]      │
    │  INTAKE IN                     SHOOTER       │
    │  Pick up game                 Spin up        │
    │  pieces                      flywheels       │
    │                                              │
    │         ╔═══════════════════╗                │
    │         ║ [BACK]   [START]  ║                │
    │         ║ (unused) (unused) ║                │
    │         ╚═══════════════════╝                │
    │                                              │
    │    ╭─────╮                    ╭─────╮        │
    │    │     │         [Y]       │     │        │
    │    │     │       FLIP UP     │     │        │
    │    │     │       (STUB)      │     │        │
    │    ╰─────╯    [X]     [B]    ╰─────╯        │
    │              LIFT    DROP                    │
    │             (STUB)  (STUB)                   │
    │                 [A]                          │
    │              FLIP DOWN                       │
    │               (STUB)                         │
    │                                              │
    │         ┌───────────────┐                    │
    │         │   D-PAD       │                    │
    │         │  ↑ Next Tag   │                    │
    │         │ ←     →       │                    │
    │         │  ↓ Prev Tag   │                    │
    │         └───────────────┘                    │
    │                                              │
    └──────────────────────────────────────────────┘
```

### Button Reference

| Button | Action | Details |
|--------|--------|---------|
| **Left Trigger (LT)** | Intake IN | Hold to run intake, pick up game pieces |
| **Left Bumper (LB)** | Intake OUT | Hold to eject game pieces |
| **Right Trigger (RT)** | Shooter | Hold to spin up flywheels |
| **Right Bumper (RB)** | Feeder | Hold to feed balls to shooter |
| **Y Button** | Flip Up | Climb arm reaches up (STUB) |
| **A Button** | Flip Down | Climb arm retracts (STUB) |
| **X Button** | Lift Robot | Winch up / climb (STUB) |
| **B Button** | Drop Robot | Lower / descend (STUB) |
| **D-pad Up** | Next Tag | Cycle to next AprilTag ID |
| **D-pad Down** | Previous Tag | Cycle to previous AprilTag ID |

### Shooting Workflow

**To score a ball:**

1. **Spin up shooter** (hold RT)
2. **Wait 1 second** for flywheels to reach speed
3. **Feed ball** (press RB while holding RT)
4. **Keep holding RT** for additional shots
5. **Release RT** when done

```
Timeline:
────────────────────────────────────────────────
  [Hold RT]
  ↓ Shooter spinning up...
  ↓ ↓ ↓ Ready!
  ↓ ↓ ↓ [Press RB] → BALL FIRES!
  ↓ ↓ ↓ [Press RB] → BALL FIRES!
  [Release RT]
────────────────────────────────────────────────
```

---

## Climb Controls (STUB Warning)

**The climb buttons (Y, A, X, B) don't do anything yet!**

The climb subsystem is a placeholder waiting for hardware. When implemented:

| Button | Intended Action |
|--------|----------------|
| Y | Flip climb arm up to reach bar |
| A | Flip climb arm down to retract |
| X | Winch up / lift robot body |
| B | Lower / let robot down |

---

## Simultaneous Controls

### Driver Combos

| Combo | Effect |
|-------|--------|
| Left Stick + Right Stick | Move and rotate simultaneously |
| LT + Left Stick | Drive while auto-aiming at tag |
| LT + RT | Vision lock at slow speed |
| Any + RT | Any action at 50% speed |

### Operator Combos

| Combo | Effect |
|-------|--------|
| RT + RB | Spin shooter AND feed (shoot!) |
| LT + RT | Intake while shooter spins (continuous cycling) |

---

## Quick Reference Card

Print this and tape it to your driver station!

```
╔════════════════════════════════════════════════════════════╗
║                    DRIVER (PORT 0)                         ║
╠════════════════════════════════════════════════════════════╣
║  L-Stick: Move          R-Stick: Rotate                    ║
║  LT: Vision Lock        RT: Slow Mode                      ║
║  LB: Blue Hub           RB: Red Hub                        ║
║  Y: Auto-Aim            X: Drive to Distance               ║
║  Start: Reset Gyro      Back: Toggle F/R Centric           ║
╠════════════════════════════════════════════════════════════╣
║                   OPERATOR (PORT 1)                        ║
╠════════════════════════════════════════════════════════════╣
║  LT: Intake IN          RT: Shooter Spin                   ║
║  LB: Intake OUT         RB: Feeder                         ║
║  Y/A: Flip Up/Down      X/B: Lift/Drop (STUB)              ║
║  D-pad ↑↓: Cycle Tag IDs                                   ║
╚════════════════════════════════════════════════════════════╝
```

---

*FRC Team 432 - The Final Countdown*
