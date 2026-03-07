// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Settings for the Limelight vision camera.
 *
 * <p>Controls how much we trust the camera's position measurements:
 * <ul>
 *   <li>How accurate we expect measurements to be
 *   <li>How much to trust measurements (based on distance and number of tags)
 *   <li>Maximum and minimum trust levels
 * </ul>
 */
public final class VisionConstants {

  // ==================== Base Trust Levels ====================

  /** Starting trust level for X/Y position per AprilTag seen (meters) */
  public static final double BASE_XY_STD_DEV = 0.5;

  /** Starting trust level for rotation per AprilTag seen (radians) */
  public static final double BASE_THETA_STD_DEV = 5;

  // ==================== Ambiguity Filtering ====================

  /** Maximum acceptable ambiguity for AprilTag detections (0.0 = perfect, 1.0 = completely ambiguous) */
  public static final double MAX_TAG_AMBIGUITY = 0.7;

  // ==================== Trust Level Limits ====================

  /** Most we'll ever trust X/Y measurements (meters) - very confident */
  public static final double MIN_XY_STD_DEV = 0.01;

  /** Least we'll ever trust X/Y measurements (meters) - not confident */
  public static final double MAX_XY_STD_DEV = 2.0;

  /** Most we'll ever trust rotation measurements (radians) - very confident */
  public static final double MIN_THETA_STD_DEV = 0.05;

  /** Least we'll ever trust rotation measurements (radians) - not confident */
  public static final double MAX_THETA_STD_DEV = 1.0;

  // ==================== Hub AprilTag IDs ====================
  // TODO: Verify all hub tag IDs against the 2026 game manual before competition.

  /**
   * Center tag on the blue alliance hub.
   * Used as the final alignment target in auto — robot centers on this before shooting.
   */
  public static final int[] BLUE_HUB_CENTER_TAG_IDS = {26};

  /**
   * Center tag on the red alliance hub.
   * Used as the final alignment target in auto — robot centers on this before shooting.
   */
  public static final int[] RED_HUB_CENTER_TAG_IDS = {10};

  /**
   * All blue alliance hub-adjacent tags (center + sides), excluding neutral zone tags.
   * Used during the initial backward drive — robot stops as soon as any of these appear.
   * 18 = right side of blue hub (user-confirmed), 25 = left side (verify), 26 = center.
   * TODO: Confirm 25 is the left-side tag; add/remove IDs as needed.
   */
  public static final int[] BLUE_HUB_ALL_TAG_IDS = {18, 25, 26};

  /**
   * All red alliance hub-adjacent tags (center + sides), excluding neutral zone tags.
   * Used during the initial backward drive — robot stops as soon as any of these appear.
   * TODO: Add the red alliance side tag ID (symmetric to blue's 18 and 25).
   */
  public static final int[] RED_HUB_ALL_TAG_IDS = {9, 10};

  /** AprilTag IDs on the blue alliance hub — used by FaceTagCommand and DriveAndLockCommand */
  public static final int[] BLUE_HUB_TAG_IDS = {25, 26};

  /** AprilTag IDs on the red alliance hub — used by FaceTagCommand and DriveAndLockCommand */
  public static final int[] RED_HUB_TAG_IDS = {9, 10};

  // ==================== Climb AprilTag IDs ====================
  // TODO: Verify these IDs against the 2026 game manual before competition.
  // These are the tags mounted on the climb structure — used by the back camera only.

  /** AprilTag IDs on the blue alliance climb structure */
  public static final int[] BLUE_CLIMB_TAG_IDS = {31, 32};

  /** AprilTag IDs on the red alliance climb structure */
  public static final int[] RED_CLIMB_TAG_IDS = {15, 16};

  // ==================== Drive-to-Tag Constants ====================

  /** Target stopping distance from the AprilTag (meters) */
  public static final double DRIVE_TO_TAG_STOP_DISTANCE_METERS = 1.0;

  /** Proportional gain for forward driving (m/s per meter of distance error) */
  public static final double DRIVE_TO_TAG_DRIVE_KP = 1.0;

  /** Proportional gain for rotation (rad/s per degree of TX offset) */
  public static final double DRIVE_TO_TAG_TURN_KP = 0.03;

  /** Maximum approach speed (m/s) */
  public static final double DRIVE_TO_TAG_MAX_SPEED_MPS = 2.0;

  /** Maximum rotation rate while tracking (rad/s) */
  public static final double DRIVE_TO_TAG_MAX_ROTATION_RAD_S = 1.5;

  /** Distance deadband to prevent jitter at target (meters) */
  public static final double DRIVE_TO_TAG_DISTANCE_TOLERANCE_METERS = 0.05;

  /** Angular deadband to prevent oscillation (degrees) */
  public static final double DRIVE_TO_TAG_TX_TOLERANCE_DEG = 1.0;

  /** Spin speed while searching for a tag (rad/s) */
  public static final double DRIVE_TO_TAG_SEARCH_SPEED_RAD_S = 1.5;

  /** Spin direction when searching (+1.0 = CCW, -1.0 = CW) */
  public static final double DRIVE_TO_TAG_SEARCH_DIRECTION = 1.0;

  // ==================== Face-Tag Tuning ====================

  /**
   * Search spin speed for FaceTagCommand (rad/s).
   * Lower than the DriveToTag search speed to reduce momentum when the tag first appears,
   * preventing the robot from spinning past the tag before the P controller can stop it.
   * Raise if the robot is too slow to find the tag; lower if it keeps overshooting.
   */
  public static final double FACE_TAG_SEARCH_SPEED_RAD_S = 2.0;

  /**
   * How many extra loop cycles to keep braking after the tag is briefly lost.
   * When spinning fast, the tag may only be visible for 1-2 frames. This keeps the
   * P controller active on the last known TX angle so the robot slows down enough
   * to reacquire the tag instead of immediately resuming the search spin.
   * At 50 Hz, 5 cycles = 100 ms of coasting. Raise if still overshooting.
   */
  public static final int FACE_TAG_BRAKE_HOLD_CYCLES = 5;

  /**
   * Low-pass filter strength for TX smoothing (0.0–1.0).
   * Lower = smoother but slower to respond. Higher = faster but noisier.
   * Tune this if turning feels laggy (raise it) or jittery (lower it).
   */
  public static final double FACE_TAG_TX_FILTER_ALPHA = 0.4;

  /**
   * Minimum rotation output (rad/s) applied whenever the robot is outside the deadband.
   * Prevents the P controller from outputting a value too small to overcome drivetrain
   * static friction, which would cause the robot to stall short of center.
   * Raise this if the robot consistently stops a few degrees off-center.
   */
  public static final double FACE_TAG_MIN_ROTATION_RAD_S = 0.15;

  // ==================== Camera Mount Offset ====================

  /**
   * TX angle (degrees) at which the ROBOT'S CENTER — not the camera — is aimed at the tag.
   *
   * <p>Because the front camera is mounted to one side of the robot's centerline, TX=0
   * means the camera is centered on the tag but the robot center is still offset. This
   * value corrects for that so {@link frc.robot.commands.AlignToTagCommand} stops at the
   * angle where the robot body is truly facing the tag.
   *
   * <p><b>How to measure:</b>
   * <ol>
   *   <li>Drive the robot to a known position directly in front of an AprilTag.
   *   <li>Manually rotate until the robot's physical centerline (e.g., midpoint of bumpers)
   *       is aimed squarely at the tag.
   *   <li>Read the TX value from SmartDashboard → Limelight/TX.
   *   <li>Set this constant to that TX value.
   * </ol>
   *
   * <p><b>Sign convention:</b>
   * <ul>
   *   <li>Positive → camera is to the LEFT of robot center (tag appears right of camera
   *       center when the robot center is aimed).
   *   <li>Negative → camera is to the RIGHT of robot center (tag appears left of camera
   *       center when the robot center is aimed).
   *   <li>0.0 → camera is exactly on the robot's centerline (no correction needed).
   * </ul>
   */
  // Camera is ~9.5 in (0.241 m) to the RIGHT of robot center.
  // At ~2 m shooting distance: -arctan(9.5/80) ≈ -6.8°.
  // TODO: tune by driving to a known position in front of a hub tag, aligning robot center
  //       perpendicular to the tag, then reading Limelight/TX from SmartDashboard.
  public static final double CAMERA_TX_OFFSET_DEG = -6.5;

  // ==================== Drive-and-Lock Tuning ====================

  /**
   * Maximum translational speed (m/s) while DriveAndLockCommand is active.
   * Limits how fast the robot can drive so it doesn't outrun the camera's ability to
   * track the AprilTag. Lower this if the tag is lost at speed; raise it carefully.
   */
  public static final double DRIVE_AND_LOCK_MAX_SPEED_MPS = 1.5;

  private VisionConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
