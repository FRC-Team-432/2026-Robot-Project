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
 *
 * <p>When no hub tag is visible the robot spins to search. If the tag is briefly lost
 * (e.g. only visible for 1-2 frames at high spin speed), the command holds the last known
 * braking direction for {@link VisionConstants#FACE_TAG_BRAKE_HOLD_CYCLES} extra cycles
 * instead of immediately resuming the search spin, giving the camera time to reacquire.
 *
 * <p>Does not drive forward or backward — use {@link DriveToTagCommand} in auto for approach.
 *
 * @param searchDirection +1.0 to search CCW (left bumper), -1.0 to search CW (right bumper)
 */
public class FaceTagCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final LimelightSubsystem limelight;
  private final double searchDirection;

  /** Smoothed TX — updated via EMA each loop, seeded to real TX on first detection. */
  private double filteredTX = 0.0;

  /** Whether a hub tag was visible last loop — used to seed the filter on acquisition. */
  private boolean hadTagLastLoop = false;

  /**
   * Remaining cycles to keep braking on last known TX after briefly losing the tag.
   * Counts down to zero, then search spin resumes.
   */
  private int brakeHoldCycles = 0;

  private final SwerveRequest.RobotCentric driveRequest =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  /**
   * @param searchDirection +1.0 = search CCW, -1.0 = search CW
   */
  public FaceTagCommand(
      CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight, double searchDirection) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.searchDirection = searchDirection;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    filteredTX = 0.0;
    hadTagLastLoop = false;
    brakeHoldCycles = 0;
  }

  @Override
  public void execute() {
    int[] targetTagIds = getAllianceHubTagIds();
    OptionalDouble txOpt = limelight.getTXForTags(targetTagIds);

    double rotationRate = 0.0;
    String status;

    if (txOpt.isPresent()) {
      double rawTX = txOpt.getAsDouble();

      if (!hadTagLastLoop) {
        // Tag just acquired — seed filter to real angle for immediate braking
        filteredTX = rawTX;
      } else {
        filteredTX =
            VisionConstants.FACE_TAG_TX_FILTER_ALPHA * rawTX
                + (1.0 - VisionConstants.FACE_TAG_TX_FILTER_ALPHA) * filteredTX;
      }
      hadTagLastLoop = true;
      brakeHoldCycles = VisionConstants.FACE_TAG_BRAKE_HOLD_CYCLES;

      SmartDashboard.putNumber("FaceTag/RawTX", rawTX);
      SmartDashboard.putNumber("FaceTag/FilteredTX", filteredTX);
    } else if (brakeHoldCycles > 0) {
      // Tag briefly lost — keep braking on last known filteredTX to slow down
      // before the camera can reacquire, rather than snapping back to search spin
      brakeHoldCycles--;
      hadTagLastLoop = false;
      status = "BRAKING";

      rotationRate = -filteredTX * VisionConstants.DRIVE_TO_TAG_TURN_KP;
      rotationRate = Math.copySign(
          Math.max(Math.abs(rotationRate), VisionConstants.FACE_TAG_MIN_ROTATION_RAD_S),
          rotationRate);
      rotationRate = MathUtil.clamp(
          rotationRate,
          -VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S,
          VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S);

      SmartDashboard.putString("FaceTag/Status", status);
      SmartDashboard.putNumber("FaceTag/TurnOutput", rotationRate);
      drivetrain.setControl(
          driveRequest.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(rotationRate));
      return;
    } else {
      // No tag, brake hold expired — spin to search
      filteredTX = 0.0;
      hadTagLastLoop = false;
      rotationRate = VisionConstants.FACE_TAG_SEARCH_SPEED_RAD_S * searchDirection;

      SmartDashboard.putString("FaceTag/Status", "SEARCHING");
      SmartDashboard.putNumber("FaceTag/TurnOutput", rotationRate);
      drivetrain.setControl(
          driveRequest.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(rotationRate));
      return;
    }

    // Tag is visible — P control on filteredTX
    if (Math.abs(filteredTX) > VisionConstants.DRIVE_TO_TAG_TX_TOLERANCE_DEG) {
      rotationRate = -filteredTX * VisionConstants.DRIVE_TO_TAG_TURN_KP;
      rotationRate = Math.copySign(
          Math.max(Math.abs(rotationRate), VisionConstants.FACE_TAG_MIN_ROTATION_RAD_S),
          rotationRate);
      rotationRate = MathUtil.clamp(
          rotationRate,
          -VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S,
          VisionConstants.DRIVE_TO_TAG_MAX_ROTATION_RAD_S);
      status = "TURNING";
    } else {
      status = "FACING_TAG";
    }

    SmartDashboard.putString("FaceTag/Status", status);
    SmartDashboard.putNumber("FaceTag/TurnOutput", rotationRate);

    drivetrain.setControl(
        driveRequest.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(rotationRate));
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
