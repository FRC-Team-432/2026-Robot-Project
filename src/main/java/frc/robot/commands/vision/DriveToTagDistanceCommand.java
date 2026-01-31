// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AprilTagConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.AprilTagTracker;

/**
 * Command to drive the robot forward/backward to reach a target distance from an AprilTag.
 *
 * <h2>What Does This Command Do?</h2>
 * <p>When you run this command, the robot will automatically drive forward or backward
 * until it's at the specified distance from the AprilTag. This is useful for:
 * <ul>
 *   <li>Positioning for scoring at a specific distance</li>
 *   <li>Maintaining consistent distance from field elements</li>
 *   <li>Automated approach sequences</li>
 * </ul>
 *
 * <h2>How Does It Work?</h2>
 * <p>Like the aim command, this uses a PID controller, but for linear motion:
 *
 * <pre>
 *   Target distance: 1.0m
 *   Current distance: 1.8m
 *   Error: 0.8m too far
 *
 *   [ROBOT]  ────────────────  [TAG]
 *              1.8 meters
 *
 *   PID says: "Drive FORWARD to close the gap"
 *
 *   After driving...
 *
 *   [ROBOT]  ──────  [TAG]
 *           1.0 meter
 *
 *   PID says: "We're at target distance - stop!"
 * </pre>
 *
 * <h2>Distance Direction Explained</h2>
 * <p>Understanding the sign of the error is important:
 * <ul>
 *   <li><b>Current > Target</b>: Robot is TOO FAR → drive FORWARD (positive)</li>
 *   <li><b>Current < Target</b>: Robot is TOO CLOSE → drive BACKWARD (negative)</li>
 *   <li><b>Current ≈ Target</b>: Robot is at target → stop (zero)</li>
 * </ul>
 *
 * <h2>Using Distance Presets</h2>
 * <p>Instead of specifying an exact distance, you can use presets:
 * <pre>
 * // Drive to "close" distance (0.5m)
 * DriveToTagDistanceCommand.withPreset(tracker, drivetrain, "close");
 *
 * // Drive to "medium" distance (1.0m)
 * DriveToTagDistanceCommand.withPreset(tracker, drivetrain, "medium");
 *
 * // Drive to "far" distance (2.0m)
 * DriveToTagDistanceCommand.withPreset(tracker, drivetrain, "far");
 * </pre>
 *
 * <p>Presets make button bindings easier to read and maintain!
 *
 * <h2>Important Design Decisions</h2>
 *
 * <h3>Robot-Centric Driving</h3>
 * <p>This command drives FORWARD/BACKWARD relative to the robot, not the field.
 * This makes sense because:
 * <ul>
 *   <li>The camera faces forward on the robot</li>
 *   <li>We want to drive toward/away from what the camera sees</li>
 *   <li>Field orientation doesn't matter for this maneuver</li>
 * </ul>
 *
 * <h3>This Command Never Finishes On Its Own</h3>
 * <p>Like AimAtTagCommand, this runs forever until stopped. Use .until() to
 * add automatic stopping:
 * <pre>
 * new DriveToTagDistanceCommand(tracker, drivetrain, 1.0)
 *     .until(() -> tracker.isAtDistance(1.0))
 * </pre>
 *
 * @see AprilTagTracker for getting tag distance data
 * @see AprilTagConstants for PID tuning values and presets
 * @see AimAtTagCommand for rotation control
 */
public class DriveToTagDistanceCommand extends Command {

  // ==================== Dependencies ====================

  /** The tag tracker that tells us how far away the tag is. */
  private final AprilTagTracker tracker;

  /** The drivetrain that we drive forward/backward. */
  private final CommandSwerveDrivetrain drivetrain;

  // ==================== Configuration ====================

  /** The target distance we want to be from the tag (meters). */
  private final double targetDistanceMeters;

  // ==================== Control ====================

  /**
   * PID controller for forward/backward driving.
   *
   * <p>Input: distance error in meters (current - target)
   * <p>Setpoint: 0 meters (no error = at target)
   * <p>Output: drive speed in meters/second
   */
  private final PIDController distancePID;

  /**
   * Swerve drive request for robot-centric driving.
   *
   * <p>We use RobotCentric because we want to drive forward/backward
   * relative to the robot (toward/away from what the camera sees),
   * not relative to the field.
   */
  private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

  // ==================== Static Factory Methods ====================
  // These provide convenient ways to create the command with common settings

  /**
   * Creates a command using a named distance preset.
   *
   * <p>This is a <b>static factory method</b> - a convenient way to create
   * commands with common configurations. Instead of remembering exact distances,
   * you can use friendly names like "close", "medium", or "far".
   *
   * <h3>Available Presets:</h3>
   * <ul>
   *   <li><b>"close"</b>: 0.5 meters - for precise, close-up work</li>
   *   <li><b>"medium"</b>: 1.0 meter - general-purpose positioning</li>
   *   <li><b>"far"</b>: 2.0 meters - safe distance for observation</li>
   * </ul>
   *
   * <h3>Example Usage:</h3>
   * <pre>
   * // In RobotContainer.java:
   * joystick.x().whileTrue(
   *     DriveToTagDistanceCommand.withPreset(tracker, drivetrain, "medium"));
   * </pre>
   *
   * @param tracker The AprilTagTracker subsystem
   * @param drivetrain The swerve drivetrain
   * @param presetName The preset name: "close", "medium", or "far"
   * @return A new DriveToTagDistanceCommand configured for the preset distance
   * @throws IllegalArgumentException if the preset name is not recognized
   */
  public static DriveToTagDistanceCommand withPreset(
      AprilTagTracker tracker, CommandSwerveDrivetrain drivetrain, String presetName) {

    // Convert preset name to distance
    double distance;
    switch (presetName.toLowerCase()) {
      case "close":
        distance = AprilTagConstants.PRESET_CLOSE_METERS;
        break;
      case "medium":
        distance = AprilTagConstants.PRESET_MEDIUM_METERS;
        break;
      case "far":
        distance = AprilTagConstants.PRESET_FAR_METERS;
        break;
      default:
        throw new IllegalArgumentException(
            "Unknown distance preset: '"
                + presetName
                + "'. Use 'close', 'medium', or 'far'.");
    }

    return new DriveToTagDistanceCommand(tracker, drivetrain, distance);
  }

  // ==================== Constructor ====================

  /**
   * Creates a new DriveToTagDistanceCommand with a specific target distance.
   *
   * <p>For common distances, consider using {@link #withPreset} instead for
   * more readable code.
   *
   * <h3>Example Usage:</h3>
   * <pre>
   * // Drive to exactly 1.5 meters from the tag:
   * new DriveToTagDistanceCommand(tracker, drivetrain, 1.5);
   *
   * // Or use a preset for common distances:
   * DriveToTagDistanceCommand.withPreset(tracker, drivetrain, "medium");
   * </pre>
   *
   * @param tracker The AprilTagTracker subsystem that provides distance data
   * @param drivetrain The swerve drivetrain to drive
   * @param targetDistanceMeters The desired distance from the tag in meters
   */
  public DriveToTagDistanceCommand(
      AprilTagTracker tracker, CommandSwerveDrivetrain drivetrain, double targetDistanceMeters) {
    this.tracker = tracker;
    this.drivetrain = drivetrain;
    this.targetDistanceMeters = targetDistanceMeters;

    // Create PID controller with tuned gains
    this.distancePID =
        new PIDController(AprilTagConstants.DISTANCE_kP, 0.0, AprilTagConstants.DISTANCE_kD);

    // This command needs exclusive control of the drivetrain
    addRequirements(drivetrain);
  }

  // ==================== Command Lifecycle Methods ====================

  /**
   * Called once when the command starts.
   *
   * <p>Resets the PID controller to start fresh, clearing any accumulated
   * state from previous runs.
   */
  @Override
  public void initialize() {
    distancePID.reset();
  }

  /**
   * Called repeatedly while the command is running (every ~20ms).
   *
   * <p>Each loop we:
   * <ol>
   *   <li>Check if we can see a tag</li>
   *   <li>If yes, calculate how fast to drive using PID</li>
   *   <li>Apply the drive speed to the drivetrain</li>
   *   <li>If no tag, stop driving (output 0)</li>
   * </ol>
   */
  @Override
  public void execute() {
    double driveSpeed = 0.0;

    // Only drive if we can see the tag
    if (tracker.isTagVisible()) {
      // Get current distance to the tag
      double currentDistance = tracker.getDistanceMeters();

      // Calculate distance error: positive = too far, negative = too close
      double distanceError = currentDistance - targetDistanceMeters;

      // Calculate drive speed using PID
      // - Positive error (too far) → positive speed (drive forward)
      // - Negative error (too close) → negative speed (drive backward)
      driveSpeed = distancePID.calculate(distanceError, 0.0);

      // Clamp to safety limits
      driveSpeed =
          MathUtil.clamp(
              driveSpeed,
              -AprilTagConstants.DISTANCE_MAX_VELOCITY_MPS,
              AprilTagConstants.DISTANCE_MAX_VELOCITY_MPS);
    }

    // Apply the drive speed
    // VelocityX is forward/backward, VelocityY is left/right (0 for this command)
    // Rotation is 0 because we're only controlling distance, not aiming
    drivetrain.setControl(
        driveRequest
            .withVelocityX(driveSpeed) // Forward/backward to adjust distance
            .withVelocityY(0.0) // No sideways motion
            .withRotationalRate(0.0)); // No rotation
  }

  /**
   * Called once when the command ends.
   *
   * <p>Stops the drivetrain to ensure the robot doesn't keep moving after
   * the command ends.
   *
   * @param interrupted true if the command was interrupted, false if it finished naturally
   */
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(
        driveRequest.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0));
  }

  /**
   * Returns whether the command has finished.
   *
   * <p>This command never finishes on its own. Use .until() to add a condition:
   * <pre>
   * command.until(() -> tracker.isAtDistance(targetDistance))
   * </pre>
   *
   * @return false always
   */
  @Override
  public boolean isFinished() {
    return false;
  }

  // ==================== Convenience Methods ====================

  /**
   * Checks if the robot is currently at the target distance.
   *
   * <p>This is useful for creating .until() conditions:
   * <pre>
   * DriveToTagDistanceCommand cmd = new DriveToTagDistanceCommand(..., 1.0);
   * button.whileTrue(cmd.until(() -> cmd.atTargetDistance()));
   * </pre>
   *
   * @return true if within tolerance of target distance and tag is visible
   */
  public boolean atTargetDistance() {
    return tracker.isAtDistance(targetDistanceMeters);
  }

  /**
   * Gets the target distance this command is trying to reach.
   *
   * <p>Useful for logging or debugging.
   *
   * @return The target distance in meters
   */
  public double getTargetDistance() {
    return targetDistanceMeters;
  }
}
