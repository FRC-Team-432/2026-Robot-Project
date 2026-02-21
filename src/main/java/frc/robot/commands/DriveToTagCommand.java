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
 * Drives the robot toward the nearest visible AprilTag and stops at a configurable distance.
 *
 * <p>Uses robot-centric proportional control with the Limelight's TX (horizontal angle) and
 * distToRobot values. When no tag is visible, the robot spins in place to search.
 *
 * <p>Designed to run while a button is held (isFinished always returns false).
 */
public class DriveToTagCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final LimelightSubsystem limelight;

  private final SwerveRequest.RobotCentric driveRequest =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  public DriveToTagCommand(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    double forwardSpeed = 0.0;
    double rotationRate = 0.0;
    String status;

    if (!limelight.hasTarget()) {
      // No target visible — spin in place to search
      rotationRate =
          VisionConstants.DRIVE_TO_TAG_SEARCH_SPEED_RAD_S
              * VisionConstants.DRIVE_TO_TAG_SEARCH_DIRECTION;
      status = "SEARCHING";
    } else {
      double tx = limelight.getTargetTX();
      double distance = limelight.getNearestTagDistance();

      if (distance < 0) {
        // Have a target but no fiducial distance data — spin to search
        rotationRate =
            VisionConstants.DRIVE_TO_TAG_SEARCH_SPEED_RAD_S
                * VisionConstants.DRIVE_TO_TAG_SEARCH_DIRECTION;
        status = "SEARCHING";
      } else {
        double distanceError = distance - VisionConstants.DRIVE_TO_TAG_STOP_DISTANCE_METERS;

        // Rotation: P control on TX to center the tag
        if (Math.abs(tx) > VisionConstants.DRIVE_TO_TAG_TX_TOLERANCE_DEG) {
          rotationRate = -tx * VisionConstants.DRIVE_TO_TAG_TURN_KP;
          rotationRate =
              MathUtil.clamp(
                  rotationRate,
                  -VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S,
                  VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S);
        }

        // Forward: P control on distance error
        if (Math.abs(distanceError) > VisionConstants.DRIVE_TO_TAG_DISTANCE_TOLERANCE_METERS) {
          forwardSpeed = distanceError * VisionConstants.DRIVE_TO_TAG_DRIVE_KP;
          forwardSpeed =
              MathUtil.clamp(
                  forwardSpeed,
                  -VisionConstants.DRIVE_TO_TAG_MAX_SPEED_MPS,
                  VisionConstants.DRIVE_TO_TAG_MAX_SPEED_MPS);
        }

        status = Math.abs(distanceError) <= VisionConstants.DRIVE_TO_TAG_DISTANCE_TOLERANCE_METERS
            ? "AT_TARGET" : "TRACKING";

        SmartDashboard.putNumber("DriveToTag/Distance", distance);
        SmartDashboard.putNumber("DriveToTag/DistanceError", distanceError);
        SmartDashboard.putNumber("DriveToTag/TX", tx);
      }
    }

    SmartDashboard.putString("DriveToTag/Status", status);
    SmartDashboard.putNumber("DriveToTag/DriveOutput", forwardSpeed);
    SmartDashboard.putNumber("DriveToTag/TurnOutput", rotationRate);

    drivetrain.setControl(
        driveRequest
            .withVelocityX(forwardSpeed)
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
}
