// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Constants for AprilTag tracking and vision-based robot control.
 *
 * <h2>What is AprilTag Tracking?</h2>
 * <p>AprilTag tracking uses the Limelight camera to detect special square markers
 * (AprilTags) placed around the field. Each tag has a unique ID number, like a QR code.
 * By detecting these tags, we can:
 * <ul>
 *   <li><b>Know which tag we're looking at</b> - Each tag has a unique ID (1, 2, 3, etc.)
 *   <li><b>Aim at the tag</b> - The camera tells us how far left/right the tag is (tx)
 *   <li><b>Know how far away it is</b> - The camera calculates distance to the tag
 * </ul>
 *
 * <h2>How Aim Assist Works</h2>
 * <p>When you enable aim assist, the robot automatically rotates to center the tag in
 * the camera view. We use a PID controller to smoothly rotate:
 *
 * <pre>
 *   Camera View:
 *   ┌─────────────────────────────┐
 *   │                             │
 *   │      [TAG]                  │  ← Tag is to the LEFT (tx is negative)
 *   │        ↓                    │    Robot should rotate LEFT
 *   │      ──┼── center           │
 *   │                             │
 *   └─────────────────────────────┘
 *
 *   After aim assist:
 *   ┌─────────────────────────────┐
 *   │                             │
 *   │           [TAG]             │  ← Tag is CENTERED (tx ≈ 0)
 *   │             │               │    Robot stops rotating
 *   │           ──┼──             │
 *   │                             │
 *   └─────────────────────────────┘
 * </pre>
 *
 * <h2>How Distance Control Works</h2>
 * <p>Distance control drives the robot forward or backward to reach a target distance
 * from the tag. This is useful for:
 * <ul>
 *   <li>Lining up for scoring at a specific distance
 *   <li>Maintaining safe distance from field elements
 *   <li>Consistent autonomous positioning
 * </ul>
 *
 * <pre>
 *   TOO FAR:                    AT TARGET:                TOO CLOSE:
 *   [ROBOT]  ─────  [TAG]       [ROBOT] ──── [TAG]        [ROBOT]─[TAG]
 *      →                           ✓                          ←
 *   Drive FORWARD               STOP                      Drive BACKWARD
 * </pre>
 *
 * <h2>What Each Constant Means</h2>
 * <p>See the comments below for detailed explanations of each value.
 *
 * @see frc.robot.subsystems.vision.AprilTagTracker for the tracking subsystem
 * @see frc.robot.commands.vision.AimAtTagCommand for the aim assist command
 * @see frc.robot.commands.vision.DriveToTagDistanceCommand for distance control
 */
public final class AprilTagConstants {

  // ==================== Aiming PID Constants ====================
  // These control how the robot rotates to center a tag in the camera view
  //
  // The PID controller works like cruise control for rotation:
  // - It measures the "error" (how far off-center the tag is)
  // - It calculates how fast to rotate to fix that error
  // - It adjusts the rotation speed to smoothly reach the target

  /**
   * Proportional gain for aiming rotation (kP).
   *
   * <p>This controls how aggressively the robot rotates based on the angular error.
   * The formula is: rotation_speed = kP * error_degrees
   *
   * <p>Example with kP = 0.03:
   * <ul>
   *   <li>Tag is 10° off center → rotation = 0.03 * 10 = 0.3 rad/s
   *   <li>Tag is 2° off center → rotation = 0.03 * 2 = 0.06 rad/s
   * </ul>
   *
   * <p>If the robot rotates too slowly → increase kP
   * <p>If the robot oscillates (wobbles back and forth) → decrease kP
   */
  public static final double AIM_kP = 0.03;

  /**
   * Derivative gain for aiming rotation (kD).
   *
   * <p>This dampens the rotation to prevent overshooting. It looks at how fast
   * the error is changing and adds resistance when the robot is rotating quickly.
   *
   * <p>Think of it like shock absorbers on a car - it prevents bouncing.
   *
   * <p>If the robot overshoots and oscillates → increase kD
   * <p>If the robot responds too sluggishly → decrease kD
   */
  public static final double AIM_kD = 0.001;

  /**
   * Maximum angular velocity for aiming rotation (radians per second).
   *
   * <p>This caps how fast the robot can rotate when aiming, even if the PID
   * would otherwise command a faster rotation. This is a safety limit to
   * prevent the robot from spinning out of control.
   *
   * <p>1.5 rad/s ≈ 86 degrees per second ≈ 1/4 of a full rotation per second
   */
  public static final double AIM_MAX_ANGULAR_VELOCITY_RAD_S = 1.5;

  // ==================== Distance PID Constants ====================
  // These control how the robot drives forward/backward to reach target distance
  //
  // Similar to aiming, but controls linear (forward/back) motion instead of rotation

  /**
   * Proportional gain for distance control (kP).
   *
   * <p>This controls how aggressively the robot drives based on distance error.
   * The formula is: drive_speed = kP * error_meters
   *
   * <p>Example with kP = 0.5:
   * <ul>
   *   <li>0.5m too far → speed = 0.5 * 0.5 = 0.25 m/s forward
   *   <li>0.2m too close → speed = 0.5 * -0.2 = -0.1 m/s (backward)
   * </ul>
   *
   * <p>If the robot drives too slowly → increase kP
   * <p>If the robot overshoots the target → decrease kP
   */
  public static final double DISTANCE_kP = 0.5;

  /**
   * Derivative gain for distance control (kD).
   *
   * <p>This dampens the drive motion to prevent overshooting the target distance.
   * It slows down the robot as it approaches the target.
   */
  public static final double DISTANCE_kD = 0.05;

  /**
   * Maximum linear velocity for distance control (meters per second).
   *
   * <p>This caps how fast the robot can drive forward/backward when adjusting
   * distance. Lower than normal driving speed for safety and precision.
   *
   * <p>1.0 m/s is a moderate speed - fast enough to be useful, slow enough to be safe
   */
  public static final double DISTANCE_MAX_VELOCITY_MPS = 1.0;

  // ==================== Tolerances ====================
  // These define "close enough" - when we've successfully aimed or reached distance

  /**
   * Aiming tolerance in degrees.
   *
   * <p>When the tag is within this many degrees of center, we consider the
   * robot "aimed" at the tag. A smaller tolerance means more precise aiming
   * but may cause the robot to constantly adjust.
   *
   * <p>2.0 degrees is a good balance between precision and stability.
   * For reference, 2 degrees at 2 meters is about 7cm of error at the target.
   */
  public static final double AIM_TOLERANCE_DEGREES = 2.0;

  /**
   * Distance tolerance in meters.
   *
   * <p>When the robot is within this distance of the target, we consider it
   * "at the target distance". A smaller tolerance means more precise positioning
   * but may cause the robot to constantly adjust.
   *
   * <p>0.1 meters (10 cm, about 4 inches) is precise enough for most game tasks.
   */
  public static final double DISTANCE_TOLERANCE_METERS = 0.1;

  // ==================== Distance Presets ====================
  // Common distances for game tasks - use these for quick button mappings

  /**
   * Close distance preset (meters).
   *
   * <p>Use this for tasks that require being very close to the target,
   * like placing a game piece or reading a small target.
   */
  public static final double PRESET_CLOSE_METERS = 0.5;

  /**
   * Medium distance preset (meters).
   *
   * <p>Use this for general-purpose positioning, like lining up for a shot
   * or approaching a field element.
   */
  public static final double PRESET_MEDIUM_METERS = 1.0;

  /**
   * Far distance preset (meters).
   *
   * <p>Use this for tasks that require more distance, like shooting from
   * far away or maintaining safe clearance.
   */
  public static final double PRESET_FAR_METERS = 2.0;

  // ==================== Tag Selection ====================
  // Settings for which AprilTags to track

  /**
   * Default AprilTag ID to track.
   *
   * <p>When the tracker starts up, it will look for this tag by default.
   * You can change which tag to track at runtime using the tracker's
   * setTargetTagId() method.
   *
   * <p>Tag IDs on FRC fields typically start at 1.
   */
  public static final int DEFAULT_TARGET_TAG_ID = 1;

  /**
   * Special value meaning "track any visible tag".
   *
   * <p>When set to this value, the tracker will lock onto the first tag
   * it sees, regardless of ID. This is useful when you don't care which
   * specific tag you're tracking.
   *
   * <p>Use -1 because real tag IDs are always positive (1, 2, 3, etc.)
   */
  public static final int ANY_TAG_ID = -1;

  // ==================== Private Constructor ====================
  // Prevents anyone from creating an instance of this class

  /**
   * Private constructor to prevent instantiation.
   *
   * <p>This class only contains constants (static final values), so there's
   * no reason to create an instance of it. Making the constructor private
   * and throwing an exception ensures no one accidentally tries to do:
   * {@code AprilTagConstants constants = new AprilTagConstants(); // ERROR!}
   */
  private AprilTagConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
