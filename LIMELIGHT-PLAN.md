# Limelight Vision System — Planning & Troubleshooting Guide

> **For Session 1 (Saturday).** Written specifically for Team 432's setup: Limelight + MegaTag + DriveToTagCommand, swerve drive.

---

## How the Limelight Works (Quick Primer)

The Limelight is a camera + compute unit mounted on the robot. It runs a vision pipeline internally and publishes results to NetworkTables, which your robot code reads.

For AprilTags specifically, two things happen:

1. **Target tracking (TX/TY):** The Limelight reports the horizontal angle (TX) and vertical angle (TY) to the nearest tag. `DriveToTagCommand` uses TX to rotate the robot toward the tag and distance to drive toward it.

2. **MegaTag pose estimation:** The Limelight uses the tag's known position on the field (from a built-in field map) plus the robot's gyro heading to estimate the robot's position on the field. `LimelightSubsystem` feeds this into the drivetrain's Kalman filter to correct odometry drift.

Your code filters out bad estimates using two thresholds:
- **`MAX_TAG_AMBIGUITY = 0.7`** — ignores measurements where the camera isn't sure which orientation the tag is at (single-tag ambiguity problem)
- **Distance-scaled standard deviations** — the further away the tag is, the less the filter trusts the vision measurement

---

## Physical Mounting

### Orientation
- **Face the camera toward where the tags are.** For most FRC games, tags are on the scoring structures and field walls. If you're scoring at one end, face the camera forward (toward that end).
- If you want to see tags behind you (e.g., for auto starting pose), a second camera is needed — but you only have one Limelight, so pick the most important direction.

### Tilt Angle
- **Tilt the camera upward 15–30 degrees** from horizontal. FRC AprilTags are typically mounted 1.5–6 feet off the ground (varies by game). If the camera points straight ahead and the robot is 12" tall, you may not see tags at head height 10 feet away.
- A steeper upward tilt sees taller/farther tags but loses close tags. A shallow tilt works better close-up. **15–20 degrees up is a good starting point.**
- After mounting, check in the Limelight web UI that tags appear near the center of the frame at your typical scoring distance.

### Height
- **Higher is generally better** — it reduces the chance of the camera being blocked by the bumpers or other robots, and it sees tags at a better angle.
- Mount it above the bumper zone (above 7.5") but within robot frame perimeter if possible.

### Avoid
- Pointing at a surface that reflects bright arena lights directly into the lens
- Placing it near motors or belt systems with high vibration — motion blur kills detection at a distance
- Letting wires block the field of view

---

## AprilTag Detection — Range and Size

### Tag Size
FRC uses **6.5-inch (165mm)** AprilTags. You do not control this — the field tags are already printed and placed. Your own practice tags (for testing in the shop) should be **the same 6.5-inch size** printed at high quality and mounted flat.

> **Printing tip:** Print on cardstock or glue to foam board. Warped or glossy tags reduce detection rate. Measure the printed tag — it should be exactly 6.5" outer border to outer border.

### Detection Range
Reliable detection and pose estimation vary by Limelight model:

| Model | Reliable AprilTag range (6.5" tag) |
|-------|-------------------------------------|
| Limelight 2 | ~8–12 feet |
| Limelight 2+ | ~12–18 feet |
| Limelight 3 | ~18–25 feet |
| Limelight 3G | ~25+ feet |

- **Below ~3 feet:** Extremely reliable but ambiguity can spike if the tag is at an extreme angle
- **At typical scoring range (5–10 ft):** Should be solid for DriveToTagCommand
- **Beyond 15–20 ft (model dependent):** Pose estimation accuracy degrades; your `BASE_XY_STD_DEV = 0.5` scaling will reduce trust appropriately

### Angle to Tag
- Best detection is **straight-on (0–30 degrees off perpendicular)**
- At 45+ degrees, ambiguity rises and MegaTag may reject the measurement (caught by `MAX_TAG_AMBIGUITY = 0.7`)
- At 75+ degrees, detection often fails entirely

---

## Limelight Web Dashboard Setup

Access at **`http://limelight.local:5801`** (or `http://10.4.32.11:5801` if hostname doesn't resolve).

### Pipeline Settings (must be correct before any code testing)
1. Go to the **Pipelines** tab
2. Select or create a pipeline of type **"Fiducial / AprilTag"** — NOT retroreflective, NOT neural
3. Set the **Tag Family** to `36h11` (the FRC standard)
4. Set the **Tag Size** to `0.1651` meters (6.5 inches)
5. Set **Solver Type** to `SQPNP` (most accurate)
6. Set the pipeline as the **active pipeline** (pipeline index 0 is default)

### Field Map (required for MegaTag pose estimation)
MegaTag needs to know where each tag is on the field in order to estimate the robot's position.
- Go to the **3D** tab in the web dashboard
- Upload the **field map JSON** for the 2026 game. This file is distributed by WPILib/FIRST or included in Limelight firmware updates.
- If the map isn't uploaded, MegaTag will not produce pose estimates (TX/TY for DriveToTagCommand still works without it).

### LED Mode
- AprilTag detection does **not need LEDs** — it uses ambient light and works in typical arena conditions
- In the web dashboard, set **LED Mode to "off"** to save power and avoid blinding drive team

### Camera Exposure
- If detection is unreliable indoors, try **increasing exposure** slightly in the pipeline settings
- If detection fails when the robot is moving, try **decreasing exposure** to reduce motion blur

---

## Tomorrow's Test Procedure (Step by Step)

### 1. Connect to the robot
- Robot on, connect laptop to robot WiFi
- Confirm ping: `ping limelight.local`

### 2. Verify Limelight web UI is reachable
- Open `http://limelight.local:5801` in a browser
- Confirm the camera feed is visible
- Check pipeline type — must be AprilTag/Fiducial

### 3. Deploy current code
```
./gradlew deploy
```

### 4. Open Shuffleboard
- Look for the `Limelight` table in NetworkTables
- Key values to watch:
  - `Limelight/HasTarget` — goes `true` when a tag is visible
  - `Limelight/TX` — horizontal angle to tag (should be ~0 when centered)
  - `Limelight/TY` — vertical angle
  - `Limelight/TagDistance` — distance in meters
  - `Limelight/TagID` — which tag is being tracked

### 5. Hold a tag in front of the camera
- `HasTarget` should go `true` within ~1 second
- TX should move toward 0 as you center the tag

### 6. Enable the robot (Teleop)
- Hold the **driver Left Bumper** — the robot should rotate toward the tag
- If it rotates the wrong way: `DRIVE_TO_TAG_TURN_KP = 0.03` in `VisionConstants.java` — negate it (set to `-0.03`)
- If it rotates but overshoots wildly: reduce `DRIVE_TO_TAG_TURN_KP`
- If it barely moves: increase `DRIVE_TO_TAG_TURN_KP`

### 7. Test at scoring distance
- Place a tag ~5–8 feet in front of the robot
- Confirm `TagDistance` reads approximately correctly
- Release and hold Left Bumper — robot should drive toward the tag and stop at ~1 meter (`DRIVE_TO_TAG_STOP_DISTANCE_METERS`)

---

## Contingencies and Problems

### Problem: Can't reach `limelight.local`
**Symptoms:** Browser times out, ping fails
**Causes & Fixes:**
- Limelight is not powered → check wiring and breaker
- Not on robot WiFi → connect to robot's access point first
- IP conflict → try the static IP directly: `10.4.32.11` (common Limelight default), or scan `10.4.32.1`–`10.4.32.50`
- mDNS not working on Windows → try the IP directly instead of hostname

---

### Problem: `HasTarget` never goes `true`
**Symptoms:** Camera feed visible in web UI, but no target detected in Shuffleboard
**Causes & Fixes:**
- **Wrong pipeline type** → web dashboard, switch to Fiducial/AprilTag
- **Wrong tag family** → must be `36h11`
- **Tag is too small or printed wrong** → confirm 6.5" outer dimension, no gloss
- **Tag is warped or at too steep an angle** → lay it flat, face camera more perpendicular to it
- **Lighting too dim** → bring the tag into better light (no LEDs needed but it must be adequately lit)
- **Camera looking in wrong direction entirely** → physically check camera field of view in web dashboard

---

### Problem: `HasTarget` flickers on and off
**Symptoms:** Unstable detection, target appears and disappears
**Causes & Fixes:**
- Tag at edge of frame → center robot/camera better
- Motion blur from vibration → check camera mounting for looseness
- Lighting inconsistent (flickering fluorescent lights) → adjust exposure in pipeline settings
- Ambiguity threshold rejecting some frames → raise `MAX_TAG_AMBIGUITY` in `VisionConstants.java` temporarily to `0.85` to see if measurements stabilize

---

### Problem: Robot spins the wrong way toward the tag
**Symptoms:** Left Bumper held, robot rotates away from tag instead of toward it
**Fix:** In `VisionConstants.java`, negate `DRIVE_TO_TAG_TURN_KP`:
```java
public static final double DRIVE_TO_TAG_TURN_KP = -0.03; // was 0.03
```

---

### Problem: Robot rotates but never stops oscillating
**Symptoms:** Robot rocks back and forth past the tag
**Fix:** Reduce `DRIVE_TO_TAG_TURN_KP` (try `0.015`) or increase `DRIVE_TO_TAG_TX_TOLERANCE_DEG` (try `2.0` degrees):
```java
public static final double DRIVE_TO_TAG_TURN_KP = 0.015;
public static final double DRIVE_TO_TAG_TX_TOLERANCE_DEG = 2.0;
```

---

### Problem: Robot drives toward tag but stops too far or too close
**Symptoms:** Stops at wrong distance from tag
**Fix:** Adjust `DRIVE_TO_TAG_STOP_DISTANCE_METERS` in `VisionConstants.java`. Currently `1.0` meter — measure where the robot actually stops vs. where you want it.
- Too far: decrease the value
- Too close: increase the value

---

### Problem: MegaTag pose estimates are corrupting odometry (robot teleports on field map)
**Symptoms:** Robot position on Shuffleboard field widget jumps around or is clearly wrong
**Causes & Fixes:**
- **Field map not uploaded** → upload field JSON to Limelight web dashboard
- **Standard deviations too low (too much trust)** → increase `BASE_XY_STD_DEV` in `VisionConstants.java` (try `1.0` or `2.0`)
- **Ambiguity threshold too loose** → lower `MAX_TAG_AMBIGUITY` (try `0.5`)
- **Tag at extreme angle** → ambiguity will be high; filter will usually catch it, but if not, lower the threshold
- **Quick fix:** Comment out the `addVisionMeasurement` call in `LimelightSubsystem` temporarily — DriveToTagCommand will still work, you'll just lose Kalman filter correction

---

### Problem: No pose estimates from MegaTag at all (TX/TY works but no odometry correction)
**Symptoms:** `HasTarget` is true, robot rotates toward tag, but field position on dashboard never updates from vision
**Causes & Fixes:**
- Field map not loaded on Limelight (most likely)
- MegaTag2 requires a valid gyro heading — if the Pigeon2 is not connected/calibrated, MegaTag will not solve
- Check DataLog/Shuffleboard for the `Limelight/PoseX` and `Limelight/PoseY` values — if they're 0,0 the Limelight isn't returning a pose

---

### Problem: "Search spin" never finds the tag
**Symptoms:** No target, robot spins continuously but doesn't find the tag
**Fix:** `DRIVE_TO_TAG_SEARCH_SPEED_RAD_S = 1.0` — this is the spin speed when no target is seen. The tag might be behind the robot or out of vertical view. Physically orient the robot so the camera faces the general direction of the tag before enabling.

---

### Problem: Vision works in the shop but not at a competition field
**Causes & Fixes:**
- **Brighter arena lights** → adjust camera exposure in pipeline
- **Tags at different heights than practice tags** → re-check camera tilt angle
- **Different tag IDs than expected** → your code logs `TagID` to Shuffleboard — confirm it's seeing the right tag
- **Network congestion from other robots** → Limelight traffic is local to robot, should be unaffected

---

## Key Constants Reference (VisionConstants.java)

| Constant | Current Value | What to adjust if... |
|---|---|---|
| `MAX_TAG_AMBIGUITY` | `0.7` | Flickering detection → raise to `0.85`; bad pose estimates → lower to `0.5` |
| `BASE_XY_STD_DEV` | `0.5` | Odometry jumps → raise to `1.0`–`2.0` |
| `BASE_THETA_STD_DEV` | `5` | Leave this high — gyro is more trusted than vision for rotation |
| `DRIVE_TO_TAG_STOP_DISTANCE_METERS` | `1.0` | Robot stops too far/close → measure and adjust |
| `DRIVE_TO_TAG_TURN_KP` | `0.03` | Wrong direction → negate; oscillation → halve |
| `DRIVE_TO_TAG_DRIVE_KP` | `1.0` | Approach too fast/slow → adjust |
| `DRIVE_TO_TAG_TX_TOLERANCE_DEG` | `1.0` | Can't settle → raise to `2.0` |

---

---

## Code-Level Diagnostics (Based on Current Codebase)

These are specific issues found by reading the actual code — distinct from generic hardware problems above.

---

### Issue 1 — Limelight Name Mismatch (MOST LIKELY CAUSE)

**The code hardcodes the name `"limelight"` in `RobotContainer.java`:**
```java
public final LimelightSubsystem limelight = new LimelightSubsystem("limelight", drivetrain);
```

Every NetworkTables call in `LimelightSubsystem` and `LimelightHelpers` uses this name as the NT table prefix. If the physical Limelight is named anything other than `"limelight"` — for example `"limelight2"`, `"limelight3g"`, `"ll"`, etc. — **every call returns 0 or false silently**. The code will never see a target even when the camera is working perfectly.

**How to check:**
- Open the Limelight web dashboard (`http://limelight.local:5801`)
- Go to **Settings** (gear icon) → look for the **Name** field
- The name there must be exactly `limelight` (all lowercase, no spaces)

**Fix option A — rename the device (recommended):**
- In the web dashboard Settings, change the name to `limelight`
- Reboot the Limelight

**Fix option B — change the code to match the device's actual name:**
- In [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java), change `"limelight"` to whatever the device is actually named

---

### Issue 2 — `HasTarget` True but Robot Still Searches (DriveToTagCommand falls through)

Even when `hasTarget()` returns true, `DriveToTagCommand` has a second check at [DriveToTagCommand.java:45](src/main/java/frc/robot/commands/DriveToTagCommand.java#L45):

```java
double distance = limelight.getNearestTagDistance();
if (distance < 0) {
    // falls into SEARCHING mode even though hasTarget() == true
```

`getNearestTagDistance()` returns `-1` if `getRawFiducials()` returns an empty array. This happens when:
- The pipeline is not in **Fiducial/AprilTag** mode (a retroreflective or neural pipeline does not populate the `rawfiducials` NT key)
- The tag is visible to `tv` (used by `hasTarget()`) but the fiducial solver isn't running

**How to confirm:** Check `DriveToTag/Status` on Shuffleboard while holding Left Bumper with a tag visible. If it says `SEARCHING` while `Limelight/HasTarget` is `true`, this is the problem.

**Fix:** In the Limelight web dashboard, confirm the active pipeline type is **Fiducial** (AprilTag), not any other type.

---

### Issue 3 — MegaTag Pose Estimation Needs a Field Map (Separate from Basic Detection)

The code in [LimelightSubsystem.java:107](src/main/java/frc/robot/subsystems/vision/LimelightSubsystem.java#L107) calls:
```java
LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
```

This is **MegaTag1**, which requires the 2026 field layout to be uploaded to the Limelight in order to produce a `tagCount > 0`. Without the field map, `tagCount` stays 0 and no pose measurements get fed to the Kalman filter.

**Importantly:** This does NOT affect `hasTarget()` or `DriveToTagCommand` — those use `getTV()` and `getRawFiducials()` independently. So the robot can snap-to-tag just fine without a field map. Only odometry correction is broken without it.

**Fix for pose estimation:** Upload the 2026 game field map JSON to the Limelight via the web dashboard → **3D** tab → field map upload.

---

### Issue 4 — Ambiguity Crashes If poseEstimate Has Zero Fiducials (Code Bug)

In [LimelightSubsystem.java:114](src/main/java/frc/robot/subsystems/vision/LimelightSubsystem.java#L114):
```java
ambiguity = poseEstimate.rawFiducials[0].ambiguity;
```

This line runs inside `if (poseEstimate != null && poseEstimate.tagCount > 0)`, but `rawFiducials` could still be empty (length 0) if MegaTag returns a non-null estimate with `tagCount > 0` but no fiducial data in the array. This would throw an `ArrayIndexOutOfBoundsException` and crash the subsystem's `periodic()`, which would stop all vision processing silently.

**How to check:** Look at the DriverStation console for a stack trace mentioning `LimelightSubsystem.java:114`.

**Fix:** Add a bounds check before reading `rawFiducials[0]`:
```java
if (poseEstimate.rawFiducials != null && poseEstimate.rawFiducials.length > 0) {
    ambiguity = poseEstimate.rawFiducials[0].ambiguity;
} else {
    ambiguity = 0.0;
}
```

---

### Diagnostic Checklist — Work Through in Order

1. **Check the Limelight name** in the web dashboard Settings. Must be exactly `limelight`.
2. **Check `Limelight/HasTarget`** on Shuffleboard with a tag in view. If false → name mismatch or wrong pipeline.
3. **Check `DriveToTag/Status`** on Shuffleboard while holding Left Bumper. If `SEARCHING` while `HasTarget` is `true` → pipeline not returning fiducial data (Issue 2).
4. **Check DriverStation console** for any exception mentioning `LimelightSubsystem` → likely Issue 4.
5. **Check `Limelight/TagCount`** — if `HasTarget` is true but `TagCount` is 0 → no field map loaded (Issue 3, only affects odometry correction).

---

## Quick Reference — Is It Working?

| Check | Expected |
|---|---|
| `ping limelight.local` responds | Camera is powered and on the network |
| Web UI shows camera feed | Camera hardware is good |
| `HasTarget = true` with tag in view | Pipeline is set correctly |
| TX moves toward 0 as you center tag | TX reporting is working |
| Left Bumper → robot rotates toward tag | DriveToTagCommand is working |
| Robot position updates on field widget | MegaTag pose estimation is working |
