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

  /** AprilTag IDs on the blue alliance hub (left, center) */
  public static final int[] BLUE_HUB_TAG_IDS = {25, 26};

  /** AprilTag IDs on the red alliance hub (left, center) */
  public static final int[] RED_HUB_TAG_IDS = {9, 10};

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
