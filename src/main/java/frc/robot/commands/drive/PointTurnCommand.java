// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Command to spin the robot in place at a specified angular velocity.
 *
 * <h2>What Does This Command Do?</h2>
 * <p>This command makes the robot spin in place (point turn) without any
 * translational movement. This is useful for:
 * <ul>
 *   <li>Quick 180° turns to face the opposite direction</li>
 *   <li>"Turbo rotate" - spin faster than normal joystick allows</li>
 *   <li>Precise heading adjustments</li>
 * </ul>
 *
 * <h2>How It Works</h2>
 * <p>The command sets all swerve modules to rotate around the robot's center
 * with zero translation velocity. This creates a pure rotation:
 *
 * <pre>
 *   Normal swerve movement:        Point turn:
 *   ┌─────────────────┐            ┌─────────────────┐
 *   │  ↗           ↗  │            │  ↖           ↗  │
 *   │                 │            │        ↻        │
 *   │                 │            │                 │
 *   │  ↗           ↗  │            │  ↙           ↘  │
 *   └─────────────────┘            └─────────────────┘
 *   Driving forward               Spinning in place
 * </pre>
 *
 * <h2>Button Binding Examples</h2>
 * <pre>
 * // Spin clockwise while A is held
 * controller.a().whileTrue(new PointTurnCommand(drivetrain, 2.0));
 *
 * // Spin counter-clockwise while B is held
 * controller.b().whileTrue(new PointTurnCommand(drivetrain, -2.0));
 *
 * // Spin at half speed while holding both
 * controller.x().whileTrue(new PointTurnCommand(drivetrain, 1.0));
 * </pre>
 *
 * @see frc.robot.commands.vision.AimAtTagCommand for vision-guided rotation
 */
public class PointTurnCommand extends Command {

  // ==================== Configuration ====================

  /** Default rotation speed in radians per second. */
  public static final double DEFAULT_ANGULAR_VELOCITY_RAD_S = 2.0;

  // ==================== Dependencies ====================

  /** The drivetrain to rotate. */
  private final CommandSwerveDrivetrain drivetrain;

  /** The angular velocity to spin at (radians per second). */
  private final double angularVelocityRadS;

  // ==================== Control ====================

  /** Robot-centric drive request for pure rotation. */
  private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

  // ==================== Static Factory Methods ====================

  /**
   * Creates a command to spin clockwise at the default speed.
   *
   * @param drivetrain The swerve drivetrain
   * @return A PointTurnCommand spinning clockwise
   */
  public static PointTurnCommand clockwise(CommandSwerveDrivetrain drivetrain) {
    return new PointTurnCommand(drivetrain, DEFAULT_ANGULAR_VELOCITY_RAD_S);
  }

  /**
   * Creates a command to spin counter-clockwise at the default speed.
   *
   * @param drivetrain The swerve drivetrain
   * @return A PointTurnCommand spinning counter-clockwise
   */
  public static PointTurnCommand counterClockwise(CommandSwerveDrivetrain drivetrain) {
    return new PointTurnCommand(drivetrain, -DEFAULT_ANGULAR_VELOCITY_RAD_S);
  }

  // ==================== Constructor ====================

  /**
   * Creates a new PointTurnCommand with the default rotation speed.
   *
   * <p>Positive values spin clockwise (when viewed from above),
   * negative values spin counter-clockwise.
   *
   * @param drivetrain The swerve drivetrain to rotate
   */
  public PointTurnCommand(CommandSwerveDrivetrain drivetrain) {
    this(drivetrain, DEFAULT_ANGULAR_VELOCITY_RAD_S);
  }

  /**
   * Creates a new PointTurnCommand with a specified rotation speed.
   *
   * <p>Positive values spin clockwise (when viewed from above),
   * negative values spin counter-clockwise.
   *
   * <h3>Speed Guidelines:</h3>
   * <ul>
   *   <li>1.0 rad/s = slow, precise rotation</li>
   *   <li>2.0 rad/s = moderate speed (default)</li>
   *   <li>4.0 rad/s = fast rotation</li>
   *   <li>6.28 rad/s = one full rotation per second</li>
   * </ul>
   *
   * @param drivetrain The swerve drivetrain to rotate
   * @param angularVelocityRadS The rotation speed in radians per second
   */
  public PointTurnCommand(CommandSwerveDrivetrain drivetrain, double angularVelocityRadS) {
    this.drivetrain = drivetrain;
    this.angularVelocityRadS = angularVelocityRadS;

    // This command requires exclusive control of the drivetrain
    addRequirements(drivetrain);
  }

  // ==================== Command Lifecycle ====================

  @Override
  public void initialize() {
    // Nothing to initialize - we'll start spinning immediately
  }

  @Override
  public void execute() {
    // Apply pure rotation with no translation
    drivetrain.setControl(
        driveRequest
            .withVelocityX(0.0) // No forward/backward movement
            .withVelocityY(0.0) // No left/right movement
            .withRotationalRate(angularVelocityRadS));
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain when the command ends
    drivetrain.setControl(
        driveRequest.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0));
  }

  @Override
  public boolean isFinished() {
    // Never auto-finish - run until button is released or interrupted
    return false;
  }

  // ==================== Utility Methods ====================

  /**
   * Gets the configured angular velocity.
   *
   * @return The angular velocity in radians per second
   */
  public double getAngularVelocity() {
    return angularVelocityRadS;
  }
}
