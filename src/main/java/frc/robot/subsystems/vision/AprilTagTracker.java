// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AllianceConstants;
import frc.robot.constants.AllianceConstants.Alliance;
import frc.robot.constants.AprilTagConstants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.RawFiducial;

/**
 * AprilTag Tracker subsystem - Detects and tracks AprilTags using a Limelight camera.
 *
 * <h2>What is This Subsystem?</h2>
 * <p>This subsystem is a <b>dedicated, single-purpose tracker</b> for AprilTags.
 * It's designed to be simple and easy to understand - perfect for learning!
 *
 * <p>Think of it like a "tag finder" that constantly looks for AprilTags and tells
 * you where they are. Other parts of your code can ask questions like:
 * <ul>
 *   <li>"Can you see any tags?" → {@link #isTagVisible()}
 *   <li>"How far left/right is it?" → {@link #getHorizontalOffset()}
 *   <li>"How far away is it?" → {@link #getDistanceMeters()}
 *   <li>"Which tag is it?" → {@link #getVisibleTagId()}
 * </ul>
 *
 * <h2>Why Create a Separate Subsystem?</h2>
 * <p>You might wonder: "We already have LimelightSubsystem - why make another one?"
 *
 * <p>Great question! There are TWO patterns for adding vision features:
 *
 * <h3>Pattern 1: Dedicated Subsystem (this class)</h3>
 * <p><b>When to use:</b> The feature is complex enough to deserve its own home,
 * or you want to keep things simple and focused.
 * <p><b>Pros:</b>
 * <ul>
 *   <li>Single responsibility - does ONE thing well</li>
 *   <li>Easier to understand - all tag tracking code in one place</li>
 *   <li>Can have its own periodic() loop for updates</li>
 *   <li>Cleaner testing - test tag tracking separately from pose estimation</li>
 * </ul>
 * <p><b>Cons:</b>
 * <ul>
 *   <li>More files to maintain</li>
 *   <li>Data might be duplicated (both subsystems reading from same camera)</li>
 * </ul>
 *
 * <h3>Pattern 2: Extend Existing Subsystem (see LimelightSubsystem)</h3>
 * <p><b>When to use:</b> The feature is small and closely related to existing
 * functionality - just add a few methods.
 * <p><b>Pros:</b>
 * <ul>
 *   <li>Less code duplication</li>
 *   <li>Shared data and state</li>
 * </ul>
 * <p><b>Cons:</b>
 * <ul>
 *   <li>Subsystem can get large and complex</li>
 *   <li>Multiple responsibilities in one class</li>
 * </ul>
 *
 * <p><b>This subsystem uses Pattern 1</b> to show you how a clean, focused
 * vision tracker should look. Check LimelightSubsystem to see Pattern 2.
 *
 * <h2>How AprilTag Detection Works</h2>
 * <p>Every 20ms (50 times per second), the {@link #periodic()} method runs:
 * <ol>
 *   <li>Asks the Limelight for ALL visible AprilTags (getRawFiducials)</li>
 *   <li>Searches through the list for our target tag ID</li>
 *   <li>If found, saves the tag's position data</li>
 *   <li>Commands can then read this data to control the robot</li>
 * </ol>
 *
 * <pre>
 *   LIMELIGHT CAMERA
 *   ┌──────────────────────┐
 *   │ Sees: Tag 1, Tag 3   │ ← Camera sees multiple tags
 *   └──────────┬───────────┘
 *              │
 *              ▼
 *   ┌──────────────────────┐
 *   │ AprilTagTracker      │
 *   │ Target: Tag 1        │ ← We only care about Tag 1
 *   │ Found: YES!          │
 *   │ tx: -5.2°            │ ← Tag is 5.2° to the LEFT
 *   │ distance: 1.5m       │ ← Tag is 1.5 meters away
 *   └──────────────────────┘
 * </pre>
 *
 * <h2>Key Data Explained</h2>
 * <ul>
 *   <li><b>tx (horizontal offset)</b>: How many degrees left/right the tag is
 *       from the camera center. Negative = left, Positive = right.</li>
 *   <li><b>distToRobot</b>: Distance from the robot to the tag in meters.
 *       The Limelight calculates this using the known tag size.</li>
 *   <li><b>tag ID</b>: The unique number printed on each AprilTag (1-20+).</li>
 * </ul>
 *
 * @see AprilTagConstants for configuration values
 * @see frc.robot.commands.vision.AimAtTagCommand for using this data to aim
 * @see frc.robot.commands.vision.DriveToTagDistanceCommand for distance control
 */
@Logged
public class AprilTagTracker extends SubsystemBase {

  // ==================== Configuration ====================
  // Settings that control which tag we're looking for

  /** The name of the Limelight camera (matches the name in the Limelight web interface). */
  private final String limelightName;

  /**
   * Which AprilTag ID we want to track (single tag mode).
   *
   * <p>Set to {@link AprilTagConstants#ANY_TAG_ID} (-1) to track any visible tag.
   * Otherwise, set to a specific tag ID (1, 2, 3, etc.) to only track that tag.
   */
  private int targetTagId = AprilTagConstants.DEFAULT_TARGET_TAG_ID;

  /**
   * Array of tag IDs to track (multi-tag mode for alliance targeting).
   *
   * <p>When targeting an alliance hub, this contains all the tag IDs on that hub.
   * The tracker will lock onto the first visible tag from this list.
   *
   * <p>Set to null to use single-tag mode (targetTagId).
   */
  private int[] targetTagIds = null;

  /**
   * The current alliance we're targeting.
   *
   * <p>This determines which fuel hub's tags we look for.
   */
  private Alliance currentAlliance = AllianceConstants.DEFAULT_ALLIANCE;

  // ==================== Cached Detection Data ====================
  // These values are updated every periodic() cycle and cached for quick access
  //
  // WHY CACHE? Reading from the Limelight network tables every time would be:
  // 1. Slower (network communication)
  // 2. Inconsistent (data might change mid-calculation)
  // By caching, all code in a single loop sees the same consistent data.

  /**
   * Whether we can currently see the target tag.
   *
   * <p>This is the first thing to check before using any other data!
   * If this is false, the offset and distance values are meaningless.
   */
  private boolean tagVisible = false;

  /**
   * Horizontal offset from camera center to the tag (degrees).
   *
   * <p>This is the "tx" value from the Limelight:
   * <ul>
   *   <li>NEGATIVE = tag is to the LEFT of center</li>
   *   <li>ZERO = tag is centered in the camera view</li>
   *   <li>POSITIVE = tag is to the RIGHT of center</li>
   * </ul>
   *
   * <p>Used for aiming - rotate the robot to make this value close to 0.
   */
  private double horizontalOffset = 0.0;

  /**
   * Distance from the robot to the tag (meters).
   *
   * <p>The Limelight calculates this using the known physical size of AprilTags.
   * Used for distance control - drive forward/backward to reach target distance.
   */
  private double distanceMeters = 0.0;

  /**
   * The ID of the tag we're actually seeing.
   *
   * <p>This might differ from targetTagId when targetTagId is set to ANY_TAG_ID,
   * in which case this tells you which tag was detected.
   */
  private int visibleTagId = -1;

  // ==================== Constructor ====================

  /**
   * Creates a new AprilTagTracker subsystem.
   *
   * <p>The tracker will start looking for tags immediately when the robot code starts.
   * By default, it looks for tag ID 1 (see {@link AprilTagConstants#DEFAULT_TARGET_TAG_ID}).
   *
   * <h3>Example Usage:</h3>
   * <pre>
   * // In RobotContainer.java:
   * public final AprilTagTracker tagTracker = new AprilTagTracker("limelight");
   *
   * // To track a specific tag:
   * tagTracker.setTargetTagId(3);  // Now tracks tag 3
   *
   * // To track any tag:
   * tagTracker.setTargetTagId(AprilTagConstants.ANY_TAG_ID);  // Tracks first visible tag
   * </pre>
   *
   * @param limelightName The name of the Limelight camera (e.g., "limelight", "limelight-front")
   */
  public AprilTagTracker(String limelightName) {
    this.limelightName = limelightName;
  }

  // ==================== Periodic Update ====================

  /**
   * Called every robot loop (approximately every 20ms).
   *
   * <p>This is where we:
   * <ol>
   *   <li>Fetch the latest tag data from the Limelight</li>
   *   <li>Search for our target tag in the list of visible tags</li>
   *   <li>Cache the position data for commands to use</li>
   * </ol>
   *
   * <p>The @Logged annotation on this class automatically logs all the
   * cached values (tagVisible, horizontalOffset, distanceMeters) to
   * SmartDashboard/NetworkTables for debugging.
   */
  @Override
  public void periodic() {
    // Get ALL visible AprilTags from the Limelight
    // This returns an array of RawFiducial objects, one per visible tag
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);

    // Reset our state - assume no tag visible until we find one
    tagVisible = false;
    horizontalOffset = 0.0;
    distanceMeters = 0.0;
    visibleTagId = -1;

    // No tags visible? We're done.
    if (fiducials == null || fiducials.length == 0) {
      return;
    }

    // Search through all visible tags for our target
    for (RawFiducial fiducial : fiducials) {
      // Check if this is a tag we want
      boolean isTarget = false;

      if (targetTagIds != null) {
        // Multi-tag mode (alliance targeting) - check if tag is in our list
        for (int id : targetTagIds) {
          if (fiducial.id == id) {
            isTarget = true;
            break;
          }
        }
      } else if (targetTagId == AprilTagConstants.ANY_TAG_ID) {
        // Any tag mode - accept any tag
        isTarget = true;
      } else {
        // Single tag mode - only accept exact match
        isTarget = (fiducial.id == targetTagId);
      }

      if (isTarget) {
        // Found our target! Save its data.
        tagVisible = true;
        visibleTagId = fiducial.id;
        horizontalOffset = fiducial.txnc; // txnc = TX No Crosshair (raw offset in degrees)
        distanceMeters = fiducial.distToRobot; // Distance calculated by Limelight

        // Stop searching - we found what we were looking for
        break;
      }
    }
  }

  // ==================== State Query Methods ====================
  // These let commands and other code check the tracker's current state

  /**
   * Checks if the target tag is currently visible.
   *
   * <p><b>ALWAYS check this first before using offset or distance!</b>
   * If this returns false, the other values are meaningless (they'll be 0).
   *
   * <h3>Example Usage:</h3>
   * <pre>
   * if (tracker.isTagVisible()) {
   *   double offset = tracker.getHorizontalOffset();
   *   // Use offset to control robot rotation
   * } else {
   *   // No tag visible - maybe stop rotating or search for tags
   * }
   * </pre>
   *
   * @return true if we can see the target tag, false otherwise
   */
  public boolean isTagVisible() {
    return tagVisible;
  }

  /**
   * Gets the horizontal offset from camera center to the tag (degrees).
   *
   * <p>This is the main value used for aiming:
   * <ul>
   *   <li><b>Negative</b> = tag is to the LEFT → rotate LEFT to center it</li>
   *   <li><b>Zero</b> = tag is centered → stop rotating</li>
   *   <li><b>Positive</b> = tag is to the RIGHT → rotate RIGHT to center it</li>
   * </ul>
   *
   * <p>Note: Always check {@link #isTagVisible()} first!
   *
   * @return Horizontal offset in degrees, or 0.0 if no tag visible
   */
  public double getHorizontalOffset() {
    return horizontalOffset;
  }

  /**
   * Gets the distance from the robot to the tag (meters).
   *
   * <p>Use this for distance-based control:
   * <ul>
   *   <li>If distance > target → drive FORWARD</li>
   *   <li>If distance < target → drive BACKWARD</li>
   *   <li>If distance ≈ target → stop driving</li>
   * </ul>
   *
   * <p>Note: Always check {@link #isTagVisible()} first!
   *
   * @return Distance to tag in meters, or 0.0 if no tag visible
   */
  public double getDistanceMeters() {
    return distanceMeters;
  }

  /**
   * Gets which tag ID we're configured to track.
   *
   * <p>Returns {@link AprilTagConstants#ANY_TAG_ID} (-1) if tracking any tag,
   * or a specific tag ID (1, 2, 3, etc.) if tracking a specific tag.
   *
   * @return The target tag ID we're looking for
   */
  public int getTargetTagId() {
    return targetTagId;
  }

  /**
   * Sets which tag ID to track.
   *
   * <p>Call this to change which tag the tracker looks for:
   * <ul>
   *   <li>Use a specific ID (1, 2, 3, etc.) to track only that tag</li>
   *   <li>Use {@link AprilTagConstants#ANY_TAG_ID} (-1) to track any visible tag</li>
   * </ul>
   *
   * <h3>Example - Cycling through tags:</h3>
   * <pre>
   * // In a button command:
   * int current = tracker.getTargetTagId();
   * if (current == AprilTagConstants.ANY_TAG_ID) {
   *   tracker.setTargetTagId(1);  // Back to tag 1
   * } else if (current >= 3) {
   *   tracker.setTargetTagId(AprilTagConstants.ANY_TAG_ID);  // Any tag
   * } else {
   *   tracker.setTargetTagId(current + 1);  // Next tag
   * }
   * </pre>
   *
   * @param tagId The tag ID to track, or ANY_TAG_ID for any tag
   */
  public void setTargetTagId(int tagId) {
    this.targetTagId = tagId;
  }

  /**
   * Gets the ID of the tag we're actually seeing right now.
   *
   * <p>This is different from {@link #getTargetTagId()}:
   * <ul>
   *   <li><b>getTargetTagId()</b> = what we WANT to find</li>
   *   <li><b>getVisibleTagId()</b> = what we ACTUALLY found</li>
   * </ul>
   *
   * <p>These are the same when tracking a specific tag. But when tracking
   * ANY_TAG_ID, this tells you which tag was actually detected.
   *
   * @return The ID of the visible tag, or -1 if no tag visible
   */
  public int getVisibleTagId() {
    return visibleTagId;
  }

  // ==================== Convenience Methods ====================
  // Higher-level checks that combine multiple state values

  /**
   * Checks if the tag is centered in the camera view (within tolerance).
   *
   * <p>This is a convenience method for commands - instead of:
   * <pre>
   * if (Math.abs(tracker.getHorizontalOffset()) < AprilTagConstants.AIM_TOLERANCE_DEGREES)
   * </pre>
   * You can just write:
   * <pre>
   * if (tracker.isAimed())
   * </pre>
   *
   * <p>Returns false if no tag is visible.
   *
   * @return true if the tag is within aim tolerance of center
   * @see AprilTagConstants#AIM_TOLERANCE_DEGREES
   */
  public boolean isAimed() {
    return tagVisible && Math.abs(horizontalOffset) < AprilTagConstants.AIM_TOLERANCE_DEGREES;
  }

  /**
   * Checks if the robot is at the target distance from the tag (within tolerance).
   *
   * <p>This is a convenience method for distance control commands.
   * Instead of manually comparing distances, just call this method.
   *
   * <p>Returns false if no tag is visible.
   *
   * @param targetDistance The desired distance in meters
   * @return true if within tolerance of the target distance
   * @see AprilTagConstants#DISTANCE_TOLERANCE_METERS
   */
  public boolean isAtDistance(double targetDistance) {
    return tagVisible
        && Math.abs(distanceMeters - targetDistance) < AprilTagConstants.DISTANCE_TOLERANCE_METERS;
  }

  /**
   * Gets the name of the Limelight camera this tracker is using.
   *
   * <p>Useful for debugging or if you need to access the Limelight directly.
   *
   * @return The Limelight name (e.g., "limelight")
   */
  public String getLimelightName() {
    return limelightName;
  }

  // ==================== Alliance Targeting Methods ====================
  // These methods let you target a specific alliance's fuel hub

  /**
   * Sets the tracker to target a specific alliance's fuel hub.
   *
   * <p>This is the MAIN method for alliance targeting! When you call this:
   * <ul>
   *   <li>The tracker will look for ANY tag on that alliance's hub</li>
   *   <li>It will ignore tags on the opponent's hub</li>
   *   <li>Vision lock will automatically aim at the correct goal</li>
   * </ul>
   *
   * <h3>Example Usage:</h3>
   * <pre>
   * // Driver presses LB for blue alliance
   * tracker.setAlliance(Alliance.BLUE);
   *
   * // Driver presses RB for red alliance
   * tracker.setAlliance(Alliance.RED);
   * </pre>
   *
   * @param alliance The alliance to target (BLUE or RED)
   */
  public void setAlliance(Alliance alliance) {
    this.currentAlliance = alliance;
    this.targetTagIds = alliance.getHubTagIds();

    System.out.println(
        "AprilTagTracker: Now targeting "
            + alliance.getDisplayName()
            + " alliance hub (tags: "
            + formatTagIds(targetTagIds)
            + ")");
  }

  /**
   * Gets the current alliance being targeted.
   *
   * @return The current alliance (BLUE or RED)
   */
  public Alliance getAlliance() {
    return currentAlliance;
  }

  /**
   * Checks if we're currently tracking a tag on the target alliance's hub.
   *
   * <p>This is useful for visual feedback (e.g., LED indicators):
   * <ul>
   *   <li>true = we see our target hub, ready to shoot!</li>
   *   <li>false = can't see target hub, need to reposition</li>
   * </ul>
   *
   * @return true if we can see a tag on the current alliance's hub
   */
  public boolean isTrackingAllianceHub() {
    if (!tagVisible || targetTagIds == null) {
      return false;
    }

    // Check if the visible tag is one of our alliance's tags
    for (int id : targetTagIds) {
      if (visibleTagId == id) {
        return true;
      }
    }
    return false;
  }

  /**
   * Clears alliance targeting and returns to single-tag mode.
   *
   * <p>After calling this, use {@link #setTargetTagId(int)} to track
   * a specific tag, or set to ANY_TAG_ID to track any visible tag.
   */
  public void clearAllianceTargeting() {
    this.targetTagIds = null;
    System.out.println("AprilTagTracker: Alliance targeting cleared, using single-tag mode");
  }

  /**
   * Helper method to format tag IDs for logging.
   */
  private String formatTagIds(int[] ids) {
    if (ids == null || ids.length == 0) {
      return "none";
    }
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < ids.length; i++) {
      if (i > 0) {
        sb.append(", ");
      }
      sb.append(ids[i]);
    }
    return sb.toString();
  }
}
