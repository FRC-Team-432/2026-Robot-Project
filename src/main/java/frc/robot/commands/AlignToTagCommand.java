package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightSubsystem;

/**
 * Rotates the robot to center on whichever AprilTag the Limelight currently sees.
 *
 * <p>Designed for autonomous use:
 * <ul>
 *   <li>Holds still when no tag is visible — never spins to search.
 *   <li>Finishes automatically once the tag is centered within tolerance.
 *   <li>Uses {@link LimelightSubsystem#hasTarget()} and {@link LimelightSubsystem#getTargetTX()}
 *       directly for maximum reliability (no tag-ID filtering that could silently fail).
 * </ul>
 *
 * <p>Always pair with {@code .withTimeout()} to prevent hanging if the tag is lost.
 */
public class AlignToTagCommand extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final LimelightSubsystem limelight;

  private final SwerveRequest.RobotCentric driveRequest =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private double filteredTX = 0.0;
  private boolean hadTagLastLoop = false;

  public AlignToTagCommand(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    filteredTX = 0.0;
    hadTagLastLoop = false;
  }

  @Override
  public void execute() {
    if (limelight.hasTarget()) {
      double rawTX = limelight.getTargetTX();

      // Seed the filter immediately on first acquisition to avoid sluggish response
      filteredTX = hadTagLastLoop
          ? VisionConstants.FACE_TAG_TX_FILTER_ALPHA * rawTX
              + (1.0 - VisionConstants.FACE_TAG_TX_FILTER_ALPHA) * filteredTX
          : rawTX;
      hadTagLastLoop = true;

      // Error relative to the camera mount offset so the robot's physical center —
      // not the camera lens — ends up aimed at the tag.
      double error = filteredTX - VisionConstants.CAMERA_TX_OFFSET_DEG;

      double rotationRate;
      if (Math.abs(error) > VisionConstants.DRIVE_TO_TAG_TX_TOLERANCE_DEG) {
        rotationRate = -error * VisionConstants.DRIVE_TO_TAG_TURN_KP;
        rotationRate = Math.copySign(
            Math.max(Math.abs(rotationRate), VisionConstants.FACE_TAG_MIN_ROTATION_RAD_S),
            rotationRate);
        rotationRate = MathUtil.clamp(
            rotationRate,
            -VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S,
            VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S);
      } else {
        rotationRate = 0.0; // Robot center is aimed at the tag
      }

      SmartDashboard.putNumber("AlignToTag/TX", rawTX);
      SmartDashboard.putNumber("AlignToTag/Error", error);
      SmartDashboard.putNumber("AlignToTag/RotationRate", rotationRate);
      SmartDashboard.putString("AlignToTag/Status", Math.abs(error) <= VisionConstants.DRIVE_TO_TAG_TX_TOLERANCE_DEG ? "ALIGNED" : "TURNING");

      drivetrain.setControl(
          driveRequest.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(rotationRate));

    } else {
      // No tag visible — hold still, do NOT spin to search
      hadTagLastLoop = false;
      SmartDashboard.putString("AlignToTag/Status", "NO_TAG");
      drivetrain.setControl(
          driveRequest.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0));
    }
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("AlignToTag/Status", interrupted ? "INTERRUPTED" : "DONE");
    drivetrain.setControl(
        driveRequest.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0));
  }

  @Override
  public boolean isFinished() {
    if (!limelight.hasTarget()) return false;
    double error = limelight.getTargetTX() - VisionConstants.CAMERA_TX_OFFSET_DEG;
    return Math.abs(error) <= VisionConstants.DRIVE_TO_TAG_TX_TOLERANCE_DEG;
  }
}
