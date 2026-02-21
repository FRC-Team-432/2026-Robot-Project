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

/**
 * Rotates the robot to face a hub AprilTag while a button is held.
 *
 * <p>Automatically selects the correct hub tags based on the current alliance
 * ({@link VisionConstants#BLUE_HUB_TAG_IDS} or {@link VisionConstants#RED_HUB_TAG_IDS}).
 * When both hub tags are visible, faces the one most centered in the camera frame.
 * When no hub tag is visible, spins in place to search.
 *
 * <p>Does not drive forward or backward — use {@link DriveToTagCommand} in auto for approach.
 */
public class FaceTagCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final LimelightSubsystem limelight;

  private final SwerveRequest.RobotCentric driveRequest =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  public FaceTagCommand(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    int[] targetTagIds = getAllianceHubTagIds();
    OptionalDouble txOpt = limelight.getTXForTags(targetTagIds);

    double rotationRate = 0.0;
    String status;

    if (txOpt.isEmpty()) {
      // No hub tag visible — spin in place to search
      rotationRate =
          VisionConstants.DRIVE_TO_TAG_SEARCH_SPEED_RAD_S
              * VisionConstants.DRIVE_TO_TAG_SEARCH_DIRECTION;
      status = "SEARCHING";
    } else {
      double tx = txOpt.getAsDouble();

      if (Math.abs(tx) > VisionConstants.DRIVE_TO_TAG_TX_TOLERANCE_DEG) {
        rotationRate = -tx * VisionConstants.DRIVE_TO_TAG_TURN_KP;
        rotationRate =
            MathUtil.clamp(
                rotationRate,
                -VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S,
                VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S);
      }

      status = Math.abs(tx) <= VisionConstants.DRIVE_TO_TAG_TX_TOLERANCE_DEG
          ? "FACING_TAG" : "TURNING";

      SmartDashboard.putNumber("FaceTag/TX", tx);
    }

    SmartDashboard.putString("FaceTag/Status", status);
    SmartDashboard.putNumber("FaceTag/TurnOutput", rotationRate);

    drivetrain.setControl(
        driveRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(rotationRate));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(
        driveRequest.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0));
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
