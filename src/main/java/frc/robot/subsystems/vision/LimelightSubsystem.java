package frc.robot.subsystems.vision;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;
import java.util.Optional;

/**
 * Limelight camera subsystem for tracking AprilTags and finding the robot's position.
 *
 * <p>This subsystem can:
 * <ul>
 *   <li>Figure out where the robot is using MegaTag (combines multiple AprilTag sightings)
 *   <li>Detect game pieces using the Limelight's AI detector
 *   <li>Account for camera lag to keep measurements accurate
 *   <li>Record all vision data for playback and debugging
 * </ul>
 *
 * <p><b>Setup:</b> You need to adjust camera height, angle, and other settings in VisionConstants
 * to match where the camera is mounted on your robot.
 *
 * <p><b>How it works:</b> The Limelight sends position data to the drivetrain, which uses it
 * to fix any drift in the wheel-based position tracking.
 *
 * @see LimelightHelpers
 */
@Logged
public class LimelightSubsystem extends SubsystemBase {
  private final String limelightName;
  private final CommandSwerveDrivetrain drivetrain;

  // Robot position from camera (using MegaTag)
  private Pose2d robotPose = new Pose2d(); // Where the camera thinks we are
  private double robotPoseTimestamp = 0.0; // When this measurement was taken
  private int tagCount = 0; // How many AprilTags the camera can see
  private double avgTagDistance = 0.0; // Average distance to visible tags (meters)
  private double ambiguity = 0.0; // Ambiguity value across all visible tags

  // System that records vision data for playback later
  private final HootAutoReplay autoReplay;

  public LimelightSubsystem(String limelightName, CommandSwerveDrivetrain drivetrain) {
    this.limelightName = limelightName;
    this.drivetrain = drivetrain;

    // Set up data recording for the Limelight
    this.autoReplay =
        new HootAutoReplay()
            .withStruct( // Record robot position
                "Limelight/" + limelightName + "/RobotPose",
                Pose2d.struct,
                () -> robotPose,
                val -> robotPose = val.value)
            .withDouble( // Record timestamp
                "Limelight/" + limelightName + "/RobotPoseTimestamp",
                () -> robotPoseTimestamp,
                val -> robotPoseTimestamp = val.value)
            .withInteger( // Record how many tags are visible
                "Limelight/" + limelightName + "/TagCount",
                () -> tagCount,
                val -> tagCount = val.value.intValue())
            .withDouble( // Record average tag distance
                "Limelight/" + limelightName + "/AvgTagDistance",
                () -> avgTagDistance,
                val -> avgTagDistance = val.value)
            .withDouble( // Record maximum ambiguity
                "Limelight/" + limelightName + "/Ambiguity",
                () -> ambiguity,
                val -> ambiguity = val.value).withTimestampReplay();
  }

  @Override
  public void periodic() {
    // Get new data from camera (unless we're replaying old data)
    if (!Utils.isReplay()) {
      fetchInputs();
    }

    // Update recording system (records on real robot, plays back during replay mode)
    autoReplay.update();

    // Use the camera data to update the robot's position
    processInputs();
  }

  private void fetchInputs() {
    // Get robot position estimate from Limelight (using blue alliance coordinates)
    LimelightHelpers.PoseEstimate poseEstimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

    if (poseEstimate != null && poseEstimate.tagCount > 0) {
      robotPose = poseEstimate.pose;
      robotPoseTimestamp = poseEstimate.timestampSeconds;
      tagCount = poseEstimate.tagCount;
      avgTagDistance = poseEstimate.avgTagDist;
      ambiguity = poseEstimate.rawFiducials[0].ambiguity;
    } else {
      robotPose = new Pose2d();
      robotPoseTimestamp = 0.0;
      tagCount = 0;
      avgTagDistance = 0.0;
      ambiguity = 0.0;
    }
  }

  private void processInputs() {
    // ==================== UPDATE ROBOT POSITION FROM APRILTAGS ====================
    // Send camera measurements to the drivetrain's position tracker
    if (tagCount > 0 && robotPoseTimestamp > 0) {
      // Check ambiguity before processing measurement. Ambiguity only happen on 1 tag
      if (tagCount == 1 && ambiguity > VisionConstants.MAX_TAG_AMBIGUITY) {
        return; // Reject measurement - don't add to drivetrain
      }

      // MegaTag combines camera and gyro data - camera for X/Y, gyro helps with rotation
      // Figure out how much to trust this measurement (more tags = more trust)
      double xyStdDev = VisionConstants.BASE_XY_STD_DEV / tagCount; // X/Y trust
      double thetaStdDev = VisionConstants.BASE_THETA_STD_DEV / tagCount; // Rotation trust

      // Trust the measurement less when tags are far away
      double avgDistDev = Math.pow(avgTagDistance,2);
      xyStdDev = xyStdDev * avgDistDev;
      thetaStdDev = thetaStdDev * avgDistDev;

      // Give the measurement to the drivetrain along with trust levels
      drivetrain.addVisionMeasurement(
          robotPose,
          robotPoseTimestamp,
          VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
    }
  }

  public Pose2d getRobotPose(){
    return robotPose;
  }

  public double getRobotPoseTimestamp() {
    return robotPoseTimestamp;
  }

  public int getTagCount() {
    return tagCount;
  }

  public double getAvgTagDistance() {
    return avgTagDistance;
  }

  public double getAmbiguity(){
    return ambiguity;
  }

  // ==================== Individual Tag Tracking Methods ====================
  // These methods demonstrate the "EXTEND EXISTING SUBSYSTEM" pattern for adding
  // new vision features.
  //
  // ┌─────────────────────────────────────────────────────────────────────────┐
  // │                    WHEN TO USE WHICH PATTERN?                          │
  // ├─────────────────────────────────────────────────────────────────────────┤
  // │                                                                         │
  // │  PATTERN 1: EXTEND EXISTING SUBSYSTEM (these methods)                  │
  // │  ─────────────────────────────────────────────────────                  │
  // │  Use this when:                                                         │
  // │    • The feature is small (just a few methods)                         │
  // │    • It's closely related to existing functionality                    │
  // │    • You want to share data already being collected                    │
  // │    • You don't need a separate periodic() loop                         │
  // │                                                                         │
  // │  Pros:                                                                  │
  // │    ✓ Less code duplication                                              │
  // │    ✓ Shares existing data (tagCount, ambiguity, etc.)                  │
  // │    ✓ Fewer files to manage                                             │
  // │                                                                         │
  // │  Cons:                                                                  │
  // │    ✗ Subsystem can grow large over time                                │
  // │    ✗ Multiple responsibilities in one class                            │
  // │    ✗ Harder to test features independently                             │
  // │                                                                         │
  // │  ─────────────────────────────────────────────────────────────────────  │
  // │                                                                         │
  // │  PATTERN 2: DEDICATED SUBSYSTEM (see AprilTagTracker.java)             │
  // │  ──────────────────────────────────────────────────────                 │
  // │  Use this when:                                                         │
  // │    • The feature is complex enough to stand alone                      │
  // │    • It has its own state to manage                                    │
  // │    • You want clear separation of concerns                             │
  // │    • You might reuse it on different robots                            │
  // │                                                                         │
  // │  Pros:                                                                  │
  // │    ✓ Single responsibility - does ONE thing well                       │
  // │    ✓ Easier to understand and maintain                                 │
  // │    ✓ Can have its own periodic() loop                                  │
  // │    ✓ Better for testing                                                │
  // │                                                                         │
  // │  Cons:                                                                  │
  // │    ✗ More files to maintain                                            │
  // │    ✗ Possible data duplication                                         │
  // │                                                                         │
  // │  ─────────────────────────────────────────────────────────────────────  │
  // │                                                                         │
  // │  RECOMMENDATION: For learning, use PATTERN 2 (dedicated subsystem).    │
  // │  It's clearer and teaches better separation of concerns. Use PATTERN 1 │
  // │  when you're confident the feature is small and won't grow.            │
  // │                                                                         │
  // └─────────────────────────────────────────────────────────────────────────┘
  //
  // The methods below show how you COULD add tag tracking directly to this
  // subsystem. Compare with AprilTagTracker to see both patterns in action!

  /**
   * Gets all raw AprilTag fiducials currently visible to the camera.
   *
   * <p>A "fiducial" is the raw detection data for each AprilTag, including:
   * <ul>
   *   <li>id - The tag's unique ID number</li>
   *   <li>txnc - Horizontal offset in degrees (no crosshair)</li>
   *   <li>tync - Vertical offset in degrees (no crosshair)</li>
   *   <li>ta - Target area (how much of the image the tag fills)</li>
   *   <li>distToCamera - Distance from camera to tag</li>
   *   <li>distToRobot - Distance from robot center to tag</li>
   *   <li>ambiguity - How certain the detection is (lower = better)</li>
   * </ul>
   *
   * <h3>Example Usage:</h3>
   * <pre>
   * RawFiducial[] tags = limelight.getRawFiducials();
   * for (RawFiducial tag : tags) {
   *   System.out.println("Tag " + tag.id + " is " + tag.distToRobot + "m away");
   * }
   * </pre>
   *
   * @return Array of visible fiducials, or empty array if none visible
   */
  public LimelightHelpers.RawFiducial[] getRawFiducials() {
    return LimelightHelpers.getRawFiducials(limelightName);
  }

  /**
   * Finds a specific AprilTag by its ID.
   *
   * <p>This searches through all visible tags to find one with a matching ID.
   * Returns an Optional because the tag might not be visible.
   *
   * <h3>Example Usage:</h3>
   * <pre>
   * Optional&lt;RawFiducial&gt; tag3 = limelight.getTagById(3);
   * if (tag3.isPresent()) {
   *   double distance = tag3.get().distToRobot;
   *   System.out.println("Tag 3 is " + distance + " meters away");
   * } else {
   *   System.out.println("Tag 3 is not visible");
   * }
   * </pre>
   *
   * @param tagId The AprilTag ID to search for
   * @return Optional containing the tag data if found, empty if not visible
   */
  public Optional<LimelightHelpers.RawFiducial> getTagById(int tagId) {
    LimelightHelpers.RawFiducial[] fiducials = getRawFiducials();
    for (LimelightHelpers.RawFiducial fiducial : fiducials) {
      if (fiducial.id == tagId) {
        return Optional.of(fiducial);
      }
    }
    return Optional.empty();
  }

  /**
   * Gets the horizontal offset (tx) of the primary target.
   *
   * <p>The "primary target" is whatever the Limelight considers the main target
   * in its current pipeline. This is the raw tx value, useful for simple aiming.
   *
   * <p>Positive values = target is to the RIGHT of center
   * <p>Negative values = target is to the LEFT of center
   *
   * @return Horizontal offset in degrees, or 0 if no target
   */
  public double getPrimaryTargetTX() {
    return LimelightHelpers.getTX(limelightName);
  }

  /**
   * Gets the vertical offset (ty) of the primary target.
   *
   * <p>Positive values = target is ABOVE center
   * <p>Negative values = target is BELOW center
   *
   * @return Vertical offset in degrees, or 0 if no target
   */
  public double getPrimaryTargetTY() {
    return LimelightHelpers.getTY(limelightName);
  }

  /**
   * Gets the ID of the primary AprilTag target.
   *
   * <p>This returns the ID of whatever tag the Limelight considers the
   * "primary" target in its current pipeline.
   *
   * @return The primary target's AprilTag ID, or -1 if no target
   */
  public int getPrimaryTargetId() {
    return (int) LimelightHelpers.getFiducialID(limelightName);
  }

  /**
   * Checks if the Limelight currently has any valid target.
   *
   * <p>This is a quick check to see if there's anything to track before
   * trying to use target data.
   *
   * <h3>Example Usage:</h3>
   * <pre>
   * if (limelight.hasTarget()) {
   *   double tx = limelight.getPrimaryTargetTX();
   *   // Use tx to aim...
   * }
   * </pre>
   *
   * @return true if a valid target is detected, false otherwise
   */
  public boolean hasTarget() {
    return LimelightHelpers.getTV(limelightName);
  }
}
