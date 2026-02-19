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
  public static final double DRIVE_TO_TAG_SEARCH_SPEED_RAD_S = 1.0;

  /** Spin direction when searching (+1.0 = CCW, -1.0 = CW) */
  public static final double DRIVE_TO_TAG_SEARCH_DIRECTION = 1.0;

  private VisionConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
