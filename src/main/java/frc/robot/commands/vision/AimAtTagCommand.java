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
 * Command to automatically rotate the robot to center an AprilTag in the camera view.
 *
 * <h2>What Does This Command Do?</h2>
 * <p>When you run this command, the robot will automatically rotate (spin in place)
 * until the AprilTag is centered in the camera view. This is often called "aim assist"
 * or "auto-aim" - the robot helps you line up with the target!
 *
 * <h2>How Does It Work?</h2>
 * <p>The command uses a <b>PID controller</b> to smoothly rotate the robot:
 *
 * <pre>
 *   1. Camera sees tag is 10° to the right (positive offset)
 *      ┌─────────────────────┐
 *      │              [TAG]  │
 *      │         ──┼──       │
 *      └─────────────────────┘
 *
 *   2. PID calculates: "Rotate LEFT at 0.3 rad/s to fix the error"
 *
 *   3. Robot rotates left...
 *
 *   4. Now tag is 2° to the right (getting closer!)
 *      ┌─────────────────────┐
 *      │          [TAG]      │
 *      │         ──┼──       │
 *      └─────────────────────┘
 *
 *   5. PID calculates: "Rotate LEFT at 0.06 rad/s" (slower now)
 *
 *   6. Eventually tag is centered (0°) - PID output is ~0 - we're aimed!
 *      ┌─────────────────────┐
 *      │         [TAG]       │
 *      │         ──┼──       │
 *      └─────────────────────┘
 * </pre>
 *
 * <h2>What is a PID Controller?</h2>
 * <p>PID stands for <b>P</b>roportional-<b>I</b>ntegral-<b>D</b>erivative. It's a
 * mathematical tool that calculates how to smoothly reach a target:
 *
 * <ul>
 *   <li><b>P (Proportional)</b>: How hard to push based on how far away we are.
 *       Bigger error → bigger correction.</li>
 *   <li><b>I (Integral)</b>: Accumulates past error. We don't use this for aiming
 *       (set to 0) because it can cause overshoot.</li>
 *   <li><b>D (Derivative)</b>: Looks at how fast the error is changing. Slows down
 *       the correction as we approach the target to prevent overshooting.</li>
 * </ul>
 *
 * <p>It's like having a smart assistant that says "push hard when far away,
 * slow down as you get close, and don't overshoot!"
 *
 * <h2>Important Design Decisions</h2>
 *
 * <h3>This Command Never Finishes On Its Own</h3>
 * <p>This command runs FOREVER until you stop it. Why? Because:
 * <ul>
 *   <li>The tag might move (or the robot might drift)</li>
 *   <li>You might want to keep aiming while doing other things</li>
 *   <li>It gives YOU control over when to stop</li>
 * </ul>
 *
 * <p>Use button bindings to control when it runs:
 * <pre>
 * // Run while holding the button:
 * button.whileTrue(new AimAtTagCommand(tracker, drivetrain));
 *
 * // Run until aimed, then stop automatically:
 * button.whileTrue(
 *     new AimAtTagCommand(tracker, drivetrain)
 *         .until(() -> tracker.isAimed()));
 * </pre>
 *
 * <h3>Stops Rotating When No Tag Visible</h3>
 * <p>If the camera loses sight of the tag, the robot stops rotating. It doesn't
 * keep spinning wildly looking for the tag - that would be dangerous!
 *
 * <h3>Uses RobotCentric Rotation</h3>
 * <p>The swerve drive request is set to robot-centric (not field-centric) because
 * we want to rotate the robot relative to itself, not the field.
 *
 * @see AprilTagTracker for getting tag position data
 * @see AprilTagConstants for PID tuning values
 * @see DriveToTagDistanceCommand for distance control
 */
public class AimAtTagCommand extends Command {

  // ==================== Dependencies ====================
  // The subsystems and data sources this command needs

  /** The tag tracker that tells us where the tag is. */
  private final AprilTagTracker tracker;

  /** The drivetrain that we rotate to aim. */
  private final CommandSwerveDrivetrain drivetrain;

  // ==================== Control ====================
  // The PID controller and drive request for rotation control

  /**
   * PID controller for rotation.
   *
   * <p>Input: horizontal offset in degrees (from camera)
   * <p>Setpoint: 0 degrees (centered)
   * <p>Output: rotation speed in radians/second
   */
  private final PIDController rotationPID;

  /**
   * Swerve drive request for robot-centric rotation.
   *
   * <p>We use RobotCentric (not FieldCentric) because we want to rotate
   * the robot relative to itself, regardless of which way it's facing
   * on the field.
   */
  private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

  // ==================== Constructor ====================

  /**
   * Creates a new AimAtTagCommand.
   *
   * <p>This command will rotate the robot to center the tag in the camera view.
   * It uses PID control for smooth, accurate rotation.
   *
   * <h3>Example Usage:</h3>
   * <pre>
   * // In RobotContainer.java:
   * joystick.y().whileTrue(new AimAtTagCommand(aprilTagTracker, drivetrain));
   * </pre>
   *
   * @param tracker The AprilTagTracker subsystem that provides tag position
   * @param drivetrain The swerve drivetrain to rotate
   */
  public AimAtTagCommand(AprilTagTracker tracker, CommandSwerveDrivetrain drivetrain) {
    this.tracker = tracker;
    this.drivetrain = drivetrain;

    // Create PID controller with our tuned gains
    // Setpoint is 0 (we want the tag centered, which means 0° offset)
    this.rotationPID =
        new PIDController(AprilTagConstants.AIM_kP, 0.0, AprilTagConstants.AIM_kD);

    // Tell the command scheduler that this command needs the drivetrain
    // This prevents other commands from controlling the drivetrain at the same time
    addRequirements(drivetrain);
  }

  // ==================== Command Lifecycle Methods ====================

  /**
   * Called once when the command is scheduled (starts running).
   *
   * <p>We reset the PID controller here to clear any accumulated error
   * from previous runs. This ensures we start fresh each time.
   */
  @Override
  public void initialize() {
    // Reset the PID controller to clear any accumulated state
    rotationPID.reset();
  }

  /**
   * Called repeatedly while the command is running (every ~20ms).
   *
   * <p>This is where the magic happens! Each loop we:
   * <ol>
   *   <li>Check if we can see a tag</li>
   *   <li>If yes, calculate how fast to rotate using PID</li>
   *   <li>Apply the rotation to the drivetrain</li>
   *   <li>If no tag, stop rotating (output 0)</li>
   * </ol>
   */
  @Override
  public void execute() {
    double rotationSpeed = 0.0;

    // Only rotate if we can see the tag
    // If we can't see it, rotationSpeed stays at 0 (no rotation)
    if (tracker.isTagVisible()) {
      // Get the horizontal offset (how far left/right the tag is)
      double horizontalOffset = tracker.getHorizontalOffset();

      // Calculate rotation speed using PID
      // - Input: current offset in degrees
      // - Setpoint: 0 degrees (we want the tag centered)
      // - Output: rotation speed (positive = clockwise, negative = counter-clockwise)
      //
      // NOTE: We NEGATE the output because:
      // - Positive tx means tag is to the RIGHT
      // - To center it, we need to rotate RIGHT (positive rotation in WPILib)
      // - But PID calculates "setpoint - measurement", giving negative output
      // - So we negate to get the correct direction
      rotationSpeed = -rotationPID.calculate(horizontalOffset, 0.0);

      // Clamp the rotation speed to our safety limit
      // This prevents the robot from spinning too fast
      rotationSpeed =
          MathUtil.clamp(
              rotationSpeed,
              -AprilTagConstants.AIM_MAX_ANGULAR_VELOCITY_RAD_S,
              AprilTagConstants.AIM_MAX_ANGULAR_VELOCITY_RAD_S);
    }

    // Apply the rotation to the drivetrain
    // VelocityX and VelocityY are 0 because we only want to rotate, not drive
    drivetrain.setControl(
        driveRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(rotationSpeed));
  }

  /**
   * Called once when the command ends (either finished or interrupted).
   *
   * <p>We stop the drivetrain here to ensure the robot doesn't keep rotating
   * after the command ends. The 'interrupted' parameter tells us why we stopped:
   * <ul>
   *   <li>interrupted = false: Command finished naturally (isFinished() returned true)</li>
   *   <li>interrupted = true: Something else stopped us (button released, another command, etc.)</li>
   * </ul>
   *
   * @param interrupted true if the command was interrupted, false if it finished naturally
   */
  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain - don't leave it rotating!
    drivetrain.setControl(
        driveRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(0.0));
  }

  /**
   * Returns whether the command has finished.
   *
   * <p>This command NEVER finishes on its own - it returns false always.
   * This is intentional! We want the command to keep running until:
   * <ul>
   *   <li>The button is released (whileTrue binding)</li>
   *   <li>A condition is met (using .until())</li>
   *   <li>Another command interrupts it</li>
   * </ul>
   *
   * <p>If you want the command to stop when aimed, use:
   * <pre>
   * new AimAtTagCommand(tracker, drivetrain)
   *     .until(() -> tracker.isAimed())
   * </pre>
   *
   * @return false always (never finishes on its own)
   */
  @Override
  public boolean isFinished() {
    // Never auto-finish - let the button binding or .until() control when we stop
    return false;
  }
}
