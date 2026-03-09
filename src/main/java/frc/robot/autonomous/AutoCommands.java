package frc.robot.autonomous;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightSubsystem;
import java.util.OptionalDouble;

/**
 * Pre-built driving commands for autonomous mode.
 *
 * <p>Ready-to-use commands for common tasks:
 *
 * <ul>
 *   <li>Drive to a location (driveTo)
 *   <li>Drive while doing something else (driveToWithAction)
 *   <li>Drive then do something when you arrive (driveToThenExecute)
 *   <li>Start actions when close to target (distanceCommand)
 *   <li>Reset the robot's starting position (resetPose)
 * </ul>
 *
 * <p><b>Example:</b>
 * <pre>{@code
 * // Drive to a spot while running the intake
 * driveToWithAction(targetPose, intake.intakeCommand())
 *
 * // Start spinning flywheel when 1.5 meters from shooting position
 * distanceCommand(1.5, shootPose, spinFlywheelCommand())
 * }</pre>
 */
public class AutoCommands {

  // Subsystems
  private final CommandSwerveDrivetrain drivetrain;

  private final LinearPathRequest pathRequest;

  private final SwerveRequest.RobotCentric robotCentric =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  /**
   * Creates AutoCommands using your robot's drive system.
   *
   * @param drivetrain The robot's swerve drive
   * @param pathRequest Helper that plans smooth paths between positions
   */
  public AutoCommands(CommandSwerveDrivetrain drivetrain, LinearPathRequest pathRequest) {
    this.drivetrain = drivetrain;
    this.pathRequest = pathRequest;
  }

  // ==================== Drive Commands ====================

  /**
   * Command to drive to a specific location and angle.
   *
   * @param pose Where to drive (x, y coordinates and rotation)
   * @return Command that drives the robot there
   */
  public Command driveTo(Pose2d pose) {
    return drivetrain.runOnce(() -> pathRequest.reset(drivetrain.getPose(), drivetrain.getFieldSpeeds()))
        .andThen(
            drivetrain.applyRequest(() -> pathRequest.withTargetPose(pose))
                .until(() -> pathRequest.isFinished()))
        .withName("DriveTo");
  }

  /**
   * Reset the robot's starting position.
   *
   * <p>Bug 4 fix: flushes swerve state with an Idle request before resetting pose,
   * preventing stale rotation rates from a previous run from persisting.
   *
   * @param pose Where the robot should think it is (x, y, rotation)
   * @return Command that resets the position
   */
  public Command resetPose(Pose2d pose) {
    return drivetrain.runOnce(() -> {
      drivetrain.setControl(new SwerveRequest.Idle());
      drivetrain.resetPose(pose);
    });
  }

  // ==================== Drive with Parallel Actions ====================

  /**
   * Drive to a location while doing something else at the same time.
   *
   * <p>Both actions start together. Use this when you want to save time by preparing
   * while driving (like raising the arm, spinning up the shooter, or intaking).
   *
   * @param targetPose Where to drive (x, y, and rotation)
   * @param parallelCommand The other thing to do while driving
   * @return Command that does both things at once
   */
  public Command driveToWithAction(Pose2d targetPose, Command parallelCommand) {
    return Commands.parallel(driveTo(targetPose), parallelCommand);
  }

  /**
   * Drive to a location, then do something when you get there.
   *
   * <p>The second action only starts after the robot arrives. Use this for actions that
   * need the robot to be still (like scoring, precise alignment, or shooting).
   *
   * @param targetPose Where to drive (x, y, and rotation)
   * @param afterCommand What to do after arriving
   * @return Command that drives then executes the action
   */
  public Command driveToThenExecute(Pose2d targetPose, Command afterCommand) {
    return Commands.sequence(driveTo(targetPose), afterCommand);
  }

  /**
   * Start a command when the robot gets close to a target location.
   *
   * <p>Use this to prepare ahead of time. For example, start spinning up the shooter
   * before you reach the shooting spot. The command watches the distance and starts
   * the action when you're close enough.
   *
   * <p><b>Example:</b>
   * <pre>{@code
   * // Start spinning flywheel when 1.5 meters away from shooting position
   * Commands.parallel(
   *   driveTo(shootPose),
   *   distanceCommand(1.5, shootPose, spinFlywheelCommand())
   * )
   * }</pre>
   *
   * @param triggerDistance How close to get before starting (in meters)
   * @param targetPose The target location to measure distance from
   * @param command What to do when you get close
   * @return Command that executes when you reach the distance
   */
  public Command distanceCommand(double triggerDistance, Pose2d targetPose, Command command) {
    return Commands.waitUntil(() -> {
      Pose2d currentPose = drivetrain.getPose();
      double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
      return distance <= triggerDistance;
    }).andThen(command);
  }

  // ==================== Vision-Drive Building Blocks ====================

  /**
   * Drive backward (robot-relative, -X) until any of the specified tags is visible.
   * Stops driving when a tag is found OR when timeout expires.
   *
   * <p>Bug 4 fix: explicitly zeros RotationalRate and VelocityY on every loop cycle,
   * preventing stale values from a previous Phase 2 from causing an arc.
   *
   * @param limelight Vision subsystem to check for tags
   * @param tagIds Array of AprilTag IDs to look for
   * @param speedMps Backward driving speed in m/s (positive value, will be negated)
   * @param timeoutSeconds Maximum time to drive before giving up
   */
  public Command driveBackwardUntilTag(LimelightSubsystem limelight, int[] tagIds,
      double speedMps, double timeoutSeconds) {
    return drivetrain.applyRequest(() ->
            robotCentric.withVelocityX(-speedMps).withRotationalRate(0.0).withVelocityY(0.0))
        .until(() -> limelight.hasSpecificTag(tagIds))
        .withTimeout(timeoutSeconds)
        .withName("DriveBackwardUntilTag");
  }

  /**
   * Rotate in place to center on a specific hub tag.
   *
   * <p>Uses getTXForTags (rawFiducialsCache) instead of getTargetId/getTargetTX to avoid
   * Limelight tracker lag between Phase 1 and Phase 2 (Bug 2 fix).
   *
   * <p>Two-way logic each loop cycle:
   *   a) Center tag visible via rawFiducialsCache -> EMA-filtered P-control on TX
   *      with dynamic camera offset correction and 1° deadband
   *   b) Center tag not visible -> brake-hold for 5 cycles on last filteredTX,
   *      then fallback spin toward last known TX direction
   *
   * <p>Exits when the center tag's filtered TX is within 3° of the dynamic offset
   * for 0.1 seconds (Debouncer).
   *
   * @param limelight Vision subsystem
   * @param centerTagId The center hub tag ID to align to (26 blue, 10 red)
   * @param timeoutSeconds Maximum alignment time
   */
  public Command alignToHubTag(LimelightSubsystem limelight, int centerTagId,
      double timeoutSeconds) {
    // Bug 3: increased debouncer from 0.05s to 0.1s for genuine settling
    Debouncer centeredDebouncer = new Debouncer(0.1, DebounceType.kRising);

    // Mutable state containers for use inside lambdas
    double[] filteredTXHolder = {0.0};
    boolean[] hadTagHolder = {false};
    int[] brakeHoldHolder = {0};
    double[] lastSeenTXHolder = {Double.NaN};
    int[] centerTagIds = {centerTagId};

    return drivetrain.applyRequest(() -> {
          double rotRate;
          OptionalDouble txOpt = limelight.getTXForTags(centerTagIds);

          // Bonus: compute dynamic camera TX offset based on distance
          double distMeters = limelight.getNearestTagDistance();
          double dynamicOffset = (distMeters > 0.1)
              ? -Math.toDegrees(Math.atan(0.241 / distMeters))
              : VisionConstants.CAMERA_TX_OFFSET_DEG;

          if (txOpt.isPresent()) {
            double rawTX = txOpt.getAsDouble();
            lastSeenTXHolder[0] = rawTX;

            // Bug 3: EMA filter — seed on first detection, smooth thereafter
            if (!hadTagHolder[0]) {
              filteredTXHolder[0] = rawTX;
            } else {
              filteredTXHolder[0] =
                  VisionConstants.FACE_TAG_TX_FILTER_ALPHA * rawTX
                      + (1.0 - VisionConstants.FACE_TAG_TX_FILTER_ALPHA) * filteredTXHolder[0];
            }
            hadTagHolder[0] = true;
            brakeHoldHolder[0] = VisionConstants.FACE_TAG_BRAKE_HOLD_CYCLES;

            double error = filteredTXHolder[0] - dynamicOffset;

            // Bug 3: deadband — stop micro-corrections when close enough
            if (Math.abs(error) < VisionConstants.DRIVE_TO_TAG_TX_TOLERANCE_DEG) {
              rotRate = 0.0;
            } else {
              rotRate = MathUtil.clamp(
                  -error * VisionConstants.DRIVE_TO_TAG_TURN_KP,
                  -VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S,
                  VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S);
            }

            SmartDashboard.putNumber("AlignHub/RawTX", rawTX);
            SmartDashboard.putNumber("AlignHub/FilteredTX", filteredTXHolder[0]);
            SmartDashboard.putNumber("AlignHub/RotRate", rotRate);
            SmartDashboard.putNumber("AlignHub/DynamicOffset", dynamicOffset);
            SmartDashboard.putString("AlignHub/Status", "TRACKING");

          } else if (brakeHoldHolder[0] > 0) {
            // Bug 3: brake hold — continue correction on last filteredTX for 5 cycles
            brakeHoldHolder[0]--;
            hadTagHolder[0] = false;

            double error = filteredTXHolder[0] - dynamicOffset;
            if (Math.abs(error) < VisionConstants.DRIVE_TO_TAG_TX_TOLERANCE_DEG) {
              rotRate = 0.0;
            } else {
              rotRate = MathUtil.clamp(
                  -error * VisionConstants.DRIVE_TO_TAG_TURN_KP,
                  -VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S,
                  VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S);
            }

            SmartDashboard.putNumber("AlignHub/RotRate", rotRate);
            SmartDashboard.putString("AlignHub/Status", "BRAKING");

          } else {
            // Bug 2: fallback spin — direction based on last seen TX, not hardcoded
            hadTagHolder[0] = false;
            if (!Double.isNaN(lastSeenTXHolder[0])) {
              double lastError = lastSeenTXHolder[0] - VisionConstants.CAMERA_TX_OFFSET_DEG;
              rotRate = (lastError > 0) ? -0.3 : 0.3;
            } else {
              rotRate = 0.3 * VisionConstants.DRIVE_TO_TAG_SEARCH_DIRECTION;
            }

            SmartDashboard.putNumber("AlignHub/RotRate", rotRate);
            SmartDashboard.putString("AlignHub/Status", "SEARCHING");
          }
          return robotCentric.withVelocityX(0.0).withRotationalRate(rotRate);
        })
        .until(() -> {
          OptionalDouble txCheck = limelight.getTXForTags(centerTagIds);
          if (txCheck.isEmpty()) {
            centeredDebouncer.calculate(false);
            return false;
          }
          double distMeters = limelight.getNearestTagDistance();
          double dynamicOffset = (distMeters > 0.1)
              ? -Math.toDegrees(Math.atan(0.241 / distMeters))
              : VisionConstants.CAMERA_TX_OFFSET_DEG;
          return centeredDebouncer.calculate(
              Math.abs(filteredTXHolder[0] - dynamicOffset) <= 3.0);
        })
        .withTimeout(timeoutSeconds)
        .withName("AlignToHubTag");
  }

  /**
   * Spin in place (no forward/backward movement) until any of the specified tags
   * is visible, or timeout expires.
   *
   * @param limelight Vision subsystem
   * @param tagIds Array of tag IDs to search for
   * @param spinRateRadS Rotation speed in rad/s (positive = CCW)
   * @param timeoutSeconds Maximum search time
   */
  public Command spinToFindTag(LimelightSubsystem limelight, int[] tagIds,
      double spinRateRadS, double timeoutSeconds) {
    return drivetrain.applyRequest(() ->
            robotCentric.withVelocityX(0.0).withRotationalRate(spinRateRadS))
        .until(() -> limelight.hasSpecificTag(tagIds))
        .withTimeout(timeoutSeconds)
        .withName("SpinToFindTag");
  }

  /**
   * Blind spin (no exit condition other than time). Used to rotate away from
   * a known area before starting a tag search, preventing false-positive exits.
   *
   * @param spinRateRadS Rotation speed in rad/s (positive = CCW)
   * @param durationSeconds How long to spin
   */
  public Command blindSpin(double spinRateRadS, double durationSeconds) {
    return drivetrain.applyRequest(() ->
            robotCentric.withVelocityX(0.0).withRotationalRate(spinRateRadS))
        .withTimeout(durationSeconds)
        .withName("BlindSpin");
  }

  // ==================== Utility Building Blocks ====================

  /** Wait for a specified duration. Convenience wrapper. */
  public Command waitSeconds(double seconds) {
    return Commands.waitSeconds(seconds).withName("Wait");
  }

  /** Print a message to console. Useful for tracing auto execution. */
  public Command log(String message) {
    return Commands.print(message).withName("Log");
  }
}
