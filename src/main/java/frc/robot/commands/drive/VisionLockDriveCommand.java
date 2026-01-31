// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AprilTagConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.AprilTagTracker;
import java.util.function.DoubleSupplier;

/**
 * Command that lets you drive while automatically rotating to face the target hub.
 *
 * <h2>What is Vision Lock?</h2>
 * <p>Vision lock is like having an auto-aim feature while driving! When you hold
 * the vision lock button (LT):
 * <ul>
 *   <li>You still control driving with the left stick (forward/back/strafe)</li>
 *   <li>The robot AUTOMATICALLY rotates to face the fuel hub</li>
 *   <li>Right stick rotation is ignored (robot handles that)</li>
 * </ul>
 *
 * <h2>Why is This Useful?</h2>
 * <p>In a game where you need to shoot at a goal:
 *
 * <pre>
 *   WITHOUT VISION LOCK:              WITH VISION LOCK:
 *   ────────────────────              ─────────────────
 *   Driver must manually             Driver just drives around,
 *   aim at the goal while            robot automatically stays
 *   also trying to drive.            aimed at goal!
 *
 *   Very hard to do both             Much easier - you can focus
 *   at the same time!                on positioning.
 *
 *        [GOAL]                          [GOAL]
 *           ↑                               ↑
 *           ?                           [ROBOT]──→ (strafing)
 *       [ROBOT]──→                      always facing goal
 *       trying to aim AND drive
 * </pre>
 *
 * <h2>How It Works</h2>
 * <pre>
 *   1. Camera sees the alliance fuel hub (AprilTag)
 *
 *   2. AprilTagTracker tells us how far off-center the tag is
 *      (e.g., "tag is 15° to the right")
 *
 *   3. PID controller calculates rotation speed needed
 *      (e.g., "rotate left at 0.5 rad/s")
 *
 *   4. Driver's translation input + auto rotation = swerve request
 *
 *   5. Robot drives where you want while facing the goal!
 * </pre>
 *
 * <h2>Field-Centric vs Robot-Centric</h2>
 * <p>This command uses FIELD-CENTRIC driving for translation:
 * <ul>
 *   <li>"Forward" always means toward the far end of the field</li>
 *   <li>Doesn't matter which way the robot is facing</li>
 *   <li>Much easier to drive while the robot is auto-rotating!</li>
 * </ul>
 *
 * @see AprilTagTracker for the vision system
 * @see frc.robot.constants.AllianceConstants for alliance targeting
 */
public class VisionLockDriveCommand extends Command {

  // ==================== Dependencies ====================

  /** The drivetrain to control. */
  private final CommandSwerveDrivetrain drivetrain;

  /** The vision tracker that finds the fuel hub. */
  private final AprilTagTracker tracker;

  // ==================== Input Suppliers ====================
  // These are "suppliers" - functions that return the current joystick values
  // We use suppliers instead of raw values so we get fresh input each loop

  /** Supplies forward/backward input (-1 to 1). */
  private final DoubleSupplier forwardInput;

  /** Supplies left/right strafe input (-1 to 1). */
  private final DoubleSupplier strafeInput;

  /** Supplies the maximum speed in meters per second. */
  private final DoubleSupplier maxSpeedSupplier;

  // ==================== Control ====================

  /** PID controller for auto-rotation. */
  private final PIDController rotationPID;

  /** Field-centric drive request. */
  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric();

  // ==================== Constructor ====================

  /**
   * Creates a new VisionLockDriveCommand.
   *
   * <p>This command combines:
   * <ul>
   *   <li>Manual translation control (from joystick)</li>
   *   <li>Automatic rotation to face the target hub</li>
   * </ul>
   *
   * <h3>Example Usage:</h3>
   * <pre>
   * // In RobotContainer.java:
   * driverController.leftTrigger().whileTrue(
   *     new VisionLockDriveCommand(
   *         drivetrain,
   *         aprilTagTracker,
   *         () -> -driverController.getLeftY(),  // Forward/back (inverted)
   *         () -> -driverController.getLeftX(),  // Left/right strafe (inverted)
   *         () -> maxSpeed * DriveConstants.NORMAL_MODE_MULTIPLIER
   *     )
   * );
   * </pre>
   *
   * @param drivetrain The swerve drivetrain
   * @param tracker The AprilTag tracker (should be set to track alliance hub)
   * @param forwardInput Supplier for forward/backward input (-1 to 1)
   * @param strafeInput Supplier for left/right input (-1 to 1)
   * @param maxSpeedSupplier Supplier for maximum speed in m/s
   */
  public VisionLockDriveCommand(
      CommandSwerveDrivetrain drivetrain,
      AprilTagTracker tracker,
      DoubleSupplier forwardInput,
      DoubleSupplier strafeInput,
      DoubleSupplier maxSpeedSupplier) {
    this.drivetrain = drivetrain;
    this.tracker = tracker;
    this.forwardInput = forwardInput;
    this.strafeInput = strafeInput;
    this.maxSpeedSupplier = maxSpeedSupplier;

    // Create PID for rotation (same gains as aim command)
    this.rotationPID =
        new PIDController(AprilTagConstants.AIM_kP, 0.0, AprilTagConstants.AIM_kD);

    addRequirements(drivetrain);
  }

  // ==================== Command Lifecycle ====================

  @Override
  public void initialize() {
    rotationPID.reset();
  }

  @Override
  public void execute() {
    // ----- Get Translation Input -----
    double forward = forwardInput.getAsDouble();
    double strafe = strafeInput.getAsDouble();

    // Apply deadband to ignore small stick movements
    forward = MathUtil.applyDeadband(forward, DriveConstants.JOYSTICK_DEADBAND);
    strafe = MathUtil.applyDeadband(strafe, DriveConstants.JOYSTICK_DEADBAND);

    // Apply input curve (squaring makes low speeds more precise)
    forward = Math.copySign(Math.pow(forward, DriveConstants.INPUT_CURVE_EXPONENT), forward);
    strafe = Math.copySign(Math.pow(strafe, DriveConstants.INPUT_CURVE_EXPONENT), strafe);

    // Scale to actual velocity
    double maxSpeed = maxSpeedSupplier.getAsDouble();
    double velocityX = forward * maxSpeed;
    double velocityY = strafe * maxSpeed;

    // ----- Calculate Auto-Rotation -----
    double rotationSpeed = 0.0;

    if (tracker.isTagVisible()) {
      // Tag visible - auto-rotate to center it
      double horizontalOffset = tracker.getHorizontalOffset();

      // PID calculates rotation speed (negate for correct direction)
      rotationSpeed = -rotationPID.calculate(horizontalOffset, 0.0);

      // Clamp to max rotation speed
      rotationSpeed =
          MathUtil.clamp(
              rotationSpeed,
              -AprilTagConstants.AIM_MAX_ANGULAR_VELOCITY_RAD_S,
              AprilTagConstants.AIM_MAX_ANGULAR_VELOCITY_RAD_S);
    }
    // If no tag visible, rotationSpeed stays 0 (no auto-rotation)

    // ----- Apply to Drivetrain -----
    drivetrain.setControl(
        driveRequest
            .withVelocityX(velocityX)
            .withVelocityY(velocityY)
            .withRotationalRate(rotationSpeed));
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain
    drivetrain.setControl(
        driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    // Never finishes on its own - runs while button is held
    return false;
  }
}
