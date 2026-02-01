# Dual Camera Vision System

**How to use two Limelight cameras for 360° AprilTag tracking.**

---

## Overview

Our robot has **two Limelight cameras**:
- **Front Camera** (`limelight-front`) - Faces forward, primary targeting
- **Back Camera** (`limelight-back`) - Faces backward, rear visibility

This gives us visibility in both directions without turning the robot!

```
        FRONT CAMERA
            ┌───┐
            │ 📷 │ ← Sees tags in front
            └───┘
       ┌───────────┐
       │           │
       │   ROBOT   │
       │           │
       └───────────┘
            ┌───┐
            │ 📷 │ ← Sees tags behind
            └───┘
        BACK CAMERA
```

---

## How It Works in Code

### Both Cameras Run Simultaneously

Each `AprilTagTracker` is an independent subsystem that reads from its own camera every 20ms:

```java
// In RobotContainer.java - both trackers created and running
public final AprilTagTracker frontTracker = new AprilTagTracker("limelight-front");
public final AprilTagTracker backTracker = new AprilTagTracker("limelight-back");
```

**Key Point:** Both cameras are ALWAYS collecting data. You just need to decide which one to USE.

### Reading From Both Cameras

You can check both cameras at any time:

```java
// Check what each camera sees
boolean frontSeesTag = frontTracker.isTagVisible();
boolean backSeesTag = backTracker.isTagVisible();

// Get data from each
double frontDistance = frontTracker.getDistanceMeters();
double backDistance = backTracker.getDistanceMeters();

double frontOffset = frontTracker.getHorizontalOffset();
double backOffset = backTracker.getHorizontalOffset();

int frontTagId = frontTracker.getVisibleTagId();
int backTagId = backTracker.getVisibleTagId();
```

---

## Decision Strategies

### Strategy 1: Use Whichever Camera Sees a Tag

**Best for:** General targeting when you don't care which direction

```java
public AprilTagTracker getAvailableTracker() {
  if (frontTracker.isTagVisible()) {
    return frontTracker;
  } else if (backTracker.isTagVisible()) {
    return backTracker;
  }
  return frontTracker; // Default when neither sees anything
}
```

### Strategy 2: Use the Closest Tag

**Best for:** When multiple tags are visible and you want the nearest one

```java
public AprilTagTracker getClosestTracker() {
  boolean frontSees = frontTracker.isTagVisible();
  boolean backSees = backTracker.isTagVisible();

  // Only one camera sees a tag - use that one
  if (frontSees && !backSees) return frontTracker;
  if (backSees && !frontSees) return backTracker;

  // Both see tags - use the closer one
  if (frontSees && backSees) {
    return frontTracker.getDistanceMeters() <= backTracker.getDistanceMeters()
        ? frontTracker
        : backTracker;
  }

  // Neither sees anything
  return frontTracker;
}
```

### Strategy 3: Prefer Front Camera (With Fallback)

**Best for:** When you usually want to face the target, but want backup vision

```java
public AprilTagTracker getPreferredTracker() {
  // Always prefer front if it sees a tag
  if (frontTracker.isTagVisible()) {
    return frontTracker;
  }
  // Fall back to rear only if front can't see anything
  if (backTracker.isTagVisible()) {
    return backTracker;
  }
  return frontTracker;
}
```

### Strategy 4: Track Specific Tag Across Cameras

**Best for:** When you need to find a specific tag ID regardless of which camera sees it

```java
public AprilTagTracker findTrackerWithTag(int targetTagId) {
  // Check if front camera sees our target
  if (frontTracker.isTagVisible() && frontTracker.getVisibleTagId() == targetTagId) {
    return frontTracker;
  }
  // Check if back camera sees our target
  if (backTracker.isTagVisible() && backTracker.getVisibleTagId() == targetTagId) {
    return backTracker;
  }
  // Target not visible on either camera
  return null;
}

// Usage:
AprilTagTracker tracker = findTrackerWithTag(5);
if (tracker != null) {
  System.out.println("Found tag 5 on " + tracker.getLimelightName());
}
```

### Strategy 5: Best Aiming Angle

**Best for:** When you want the camera with the smallest angle adjustment needed

```java
public AprilTagTracker getBestAimingTracker() {
  boolean frontSees = frontTracker.isTagVisible();
  boolean backSees = backTracker.isTagVisible();

  if (frontSees && !backSees) return frontTracker;
  if (backSees && !frontSees) return backTracker;

  if (frontSees && backSees) {
    // Use whichever has smaller offset (less rotation needed)
    double frontAngle = Math.abs(frontTracker.getHorizontalOffset());
    double backAngle = Math.abs(backTracker.getHorizontalOffset());
    return frontAngle <= backAngle ? frontTracker : backTracker;
  }

  return frontTracker;
}
```

---

## Practical Code Snippets

### Snippet 1: Smart Vision Lock Command

A command that automatically uses the best available camera:

```java
public class SmartVisionLockCommand extends Command {
  private final RobotContainer robot;
  private final CommandSwerveDrivetrain drivetrain;

  public SmartVisionLockCommand(RobotContainer robot) {
    this.robot = robot;
    this.drivetrain = robot.drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    // Get the best tracker dynamically each loop
    AprilTagTracker activeTracker = robot.getActiveTracker();

    if (activeTracker.isTagVisible()) {
      double offset = activeTracker.getHorizontalOffset();
      // Calculate rotation to center the tag...
      double rotation = -offset * 0.03; // Simple P control

      // Apply rotation
      drivetrain.setControl(/* ... */);
    }
  }
}
```

### Snippet 2: Display Which Camera is Active

Show the driver which camera is being used:

```java
// In a periodic method or command
public void updateCameraDisplay() {
  boolean frontActive = frontTracker.isTagVisible();
  boolean backActive = backTracker.isTagVisible();

  String status;
  if (frontActive && backActive) {
    status = "BOTH - using " + getActiveTracker().getLimelightName();
  } else if (frontActive) {
    status = "FRONT";
  } else if (backActive) {
    status = "BACK";
  } else {
    status = "NONE";
  }

  SmartDashboard.putString("Active Camera", status);
}
```

### Snippet 3: Auto-Rotate to Face Tag

Rotate to face whichever direction has a visible tag:

```java
public Command autoFaceTagCommand() {
  return run(() -> {
    AprilTagTracker tracker = getActiveTracker();

    if (!tracker.isTagVisible()) {
      // No tag - stop rotating
      drivetrain.setControl(stopRequest);
      return;
    }

    // Determine if we need to rotate 180° (tag is behind us)
    boolean usingBackCamera = tracker == backTracker;
    double offset = tracker.getHorizontalOffset();

    if (usingBackCamera) {
      // Tag is behind - we need to turn around!
      // Add 180° worth of rotation to face the tag
      System.out.println("Tag behind us - turning around!");
      // Rotate at fixed speed until front camera sees it
      drivetrain.setControl(rotateRequest.withRotationalRate(2.0));
    } else {
      // Tag is in front - normal aiming
      double rotation = -offset * 0.03;
      drivetrain.setControl(rotateRequest.withRotationalRate(rotation));
    }
  });
}
```

### Snippet 4: Distance Check From Either Camera

Check if we're at scoring distance regardless of which camera sees the target:

```java
public boolean isAtScoringDistance() {
  double targetDistance = 1.5; // meters
  double tolerance = 0.1; // meters

  // Check front camera
  if (frontTracker.isTagVisible()) {
    if (Math.abs(frontTracker.getDistanceMeters() - targetDistance) < tolerance) {
      return true;
    }
  }

  // Check back camera
  if (backTracker.isTagVisible()) {
    if (Math.abs(backTracker.getDistanceMeters() - targetDistance) < tolerance) {
      return true;
    }
  }

  return false;
}
```

### Snippet 5: Combined Camera Data for Better Accuracy

If both cameras see tags, you could average or combine their data:

```java
public double getCombinedDistance() {
  boolean frontSees = frontTracker.isTagVisible();
  boolean backSees = backTracker.isTagVisible();

  if (frontSees && backSees) {
    // Average the two readings for potentially better accuracy
    return (frontTracker.getDistanceMeters() + backTracker.getDistanceMeters()) / 2.0;
  } else if (frontSees) {
    return frontTracker.getDistanceMeters();
  } else if (backSees) {
    return backTracker.getDistanceMeters();
  }

  return 0.0; // No data available
}
```

---

## Button Binding Ideas

### Option A: Separate Buttons Per Camera

```java
// Y = aim using front camera only
driverController.y().whileTrue(new AimAtTagCommand(frontTracker, drivetrain));

// A = aim using back camera only
driverController.a().whileTrue(new AimAtTagCommand(backTracker, drivetrain));
```

### Option B: Smart Auto-Selection

```java
// Y = aim using whichever camera has best view
driverController.y().whileTrue(
    new AimAtTagCommand(getActiveTracker(), drivetrain));
```

**Problem:** `getActiveTracker()` is evaluated once when command starts!

**Solution:** Create a command that checks each loop:

```java
driverController.y().whileTrue(
    run(() -> {
      AprilTagTracker active = getActiveTracker();
      if (active.isTagVisible()) {
        // Do aiming with active tracker
      }
    }).withName("SmartAim"));
```

### Option C: Toggle Between Cameras

```java
private boolean useBackCamera = false;

// Press B to toggle camera preference
driverController.b().onTrue(runOnce(() -> {
  useBackCamera = !useBackCamera;
  System.out.println("Now using: " + (useBackCamera ? "BACK" : "FRONT") + " camera");
}));

// Y aims with selected camera
driverController.y().whileTrue(
    new AimAtTagCommand(useBackCamera ? backTracker : frontTracker, drivetrain));
```

---

## Important Considerations

### 1. Camera Orientation Matters

The back camera sees things "backwards" - when a tag is to the LEFT in the back camera's view, it's actually to the RIGHT of the robot!

```java
// If using back camera, you may need to invert the offset
double offset = tracker.getHorizontalOffset();
if (tracker == backTracker) {
  offset = -offset; // Invert for back camera
}
```

### 2. Alliance Targeting Must Be Set on BOTH Cameras

```java
// When changing alliance, update both!
public void setAlliance(Alliance alliance) {
  frontTracker.setAlliance(alliance);
  backTracker.setAlliance(alliance);
}
```

### 3. Simulation Limitations

Neither camera works in simulation - `isTagVisible()` always returns false. Test camera selection logic on the real robot!

### 4. Network Bandwidth

Two cameras = more network traffic. If you experience lag:
- Reduce camera resolution in Limelight settings
- Increase compression
- Only enable the camera you're actively using

---

## Quick Reference

| Need | Solution |
|------|----------|
| Any visible tag | `frontTracker.isTagVisible() \|\| backTracker.isTagVisible()` |
| Closest tag | Compare `getDistanceMeters()` from both |
| Specific tag ID | Check `getVisibleTagId()` on both |
| Best aiming angle | Compare `Math.abs(getHorizontalOffset())` |
| Tag behind robot | `backTracker.isTagVisible() && !frontTracker.isTagVisible()` |

---

## Student Exercises

1. **Modify `getActiveTracker()`** to prefer the camera that's been tracking longer (add a "tracking time" counter)

2. **Create a "search mode"** that slowly rotates until any camera finds a tag

3. **Add LED feedback** (if you have addressable LEDs) to show green when front camera sees tag, blue when back camera sees tag

4. **Implement "sticky tracking"** - once a camera starts tracking, keep using it until it loses the tag for 0.5 seconds

5. **Create autonomous logic** that uses the back camera to approach a tag while driving backwards

---

*FRC Team 432 - The Final Countdown*
