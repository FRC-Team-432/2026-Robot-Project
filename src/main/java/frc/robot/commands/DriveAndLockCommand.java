package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightSubsystem;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

/**
 * Locks the robot's rotation onto the hub AprilTag while the driver retains full
 * translational control with the left stick (field-relative, same feel as default drive).
 *
 * <p>Automatically selects red or blue hub tag IDs from {@link VisionConstants} based on alliance.
 *
 * <p>When the tag is not visible the robot holds its current heading and continues driving
 * normally. As soon as the tag reappears it relocks automatically. The command runs until
 * X is released — it never ends on its own due to tag loss.
 */
public class DriveAndLockCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final LimelightSubsystem limelight;
  private final DoubleSupplier velocityXSupplier;
  private final DoubleSupplier velocityYSupplier;

  /** Smoothed TX — seeded to real TX on first detection, then updated via EMA. */
  private double filteredTX = 0.0;

  private boolean hadTagLastLoop = false;

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  /**
   * @param velocityXSupplier Field-relative X velocity in m/s (already scaled and deadbanded)
   * @param velocityYSupplier Field-relative Y velocity in m/s (already scaled and deadbanded)
   */
  public DriveAndLockCommand(
      CommandSwerveDrivetrain drivetrain,
      LimelightSubsystem limelight,
      DoubleSupplier velocityXSupplier,
      DoubleSupplier velocityYSupplier) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.velocityXSupplier = velocityXSupplier;
    this.velocityYSupplier = velocityYSupplier;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    filteredTX = 0.0;
    hadTagLastLoop = false;
  }

  @Override
  public void execute() {
    OptionalDouble txOpt = limelight.getTXForTags(getAllianceHubTagIds());
    double vx = velocityXSupplier.getAsDouble();
    double vy = velocityYSupplier.getAsDouble();

    // Scale the velocity vector down if it exceeds the camera-safe speed limit.
    // Scaling preserves drive direction; clamping X/Y independently would skew it.
    double speed = Math.hypot(vx, vy);
    if (speed > VisionConstants.DRIVE_AND_LOCK_MAX_SPEED_MPS) {
      double scale = VisionConstants.DRIVE_AND_LOCK_MAX_SPEED_MPS / speed;
      vx *= scale;
      vy *= scale;
    }

    double rotationRate = 0.0;
    String status;

    if (txOpt.isPresent()) {
      double rawTX = txOpt.getAsDouble();

      if (!hadTagLastLoop) {
        // Tag just (re)acquired — seed filter for immediate response
        filteredTX = rawTX;
      } else {
        filteredTX =
            VisionConstants.FACE_TAG_TX_FILTER_ALPHA * rawTX
                + (1.0 - VisionConstants.FACE_TAG_TX_FILTER_ALPHA) * filteredTX;
      }
      hadTagLastLoop = true;

      if (Math.abs(filteredTX) > VisionConstants.DRIVE_TO_TAG_TX_TOLERANCE_DEG) {
        rotationRate = -filteredTX * VisionConstants.DRIVE_TO_TAG_TURN_KP;
        rotationRate = Math.copySign(
            Math.max(Math.abs(rotationRate), VisionConstants.FACE_TAG_MIN_ROTATION_RAD_S),
            rotationRate);
        rotationRate = MathUtil.clamp(
            rotationRate,
            -VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S,
            VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S);
        status = "LOCKING";
      } else {
        status = "LOCKED";
      }

      SmartDashboard.putNumber("DriveAndLock/FilteredTX", filteredTX);
    } else {
      // Tag not visible — hold heading (rotationRate = 0) and wait to reacquire
      hadTagLastLoop = false;
      status = "HOLDING_HEADING";
    }

    SmartDashboard.putString("DriveAndLock/Status", status);
    SmartDashboard.putNumber("DriveAndLock/TurnOutput", rotationRate);

    drivetrain.setControl(
        driveRequest
            .withVelocityX(vx)
            .withVelocityY(vy)
            .withRotationalRate(rotationRate));
  }

  @Override
  public void end(boolean interrupted) {
    // Default drive command resumes when X is released
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private int[] getAllianceHubTagIds() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        ? VisionConstants.RED_HUB_TAG_IDS
        : VisionConstants.BLUE_HUB_TAG_IDS;
  }
}
