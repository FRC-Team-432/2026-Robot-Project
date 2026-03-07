package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

/**
 * Superstructure — coordinates the Shooter and Feeder together.
 *
 * <p>Climb is controlled directly from RobotContainer (Y/A buttons)
 * and does not need to be coordinated with the shooter.
 *
 * <p><b>Teleop shooting</b> (right trigger):
 * Use {@link #teleOpShootCommand()} to run all three motors at once while
 * the trigger is held. For distance-based speed, use
 * {@link #teleOpShootWithDistanceCommand(DoubleSupplier)} instead.
 *
 * <p><b>Auto shooting</b>: use the prepare → shoot → stow sequence:
 * <pre>
 *   superstructure.speakerCloseAndWaitCommand()  // shooter spins up, wait until ready
 *   superstructure.shootCommand()                // feeder fires ball through
 *   superstructure.stowCommand()                 // stop shooter
 * </pre>
 */
@Logged
public class Superstructure extends SubsystemBase {

  private final Shooter shooter;
  private final Feeder feeder;

  public Superstructure(Shooter shooter, Feeder feeder) {
    this.shooter = shooter;
    this.feeder = feeder;
  }

  // ==================== Teleop Shooting Commands ====================

  /**
   * Run all three motors simultaneously while the command is active.
   *
   * <p>Bind to the right trigger with {@code .whileTrue()} so the motors
   * run while the trigger is held and stop the moment it is released.
   *
   * @return Command that runs shooter + feeder while active, stops on release
   */
  public Command teleOpShootCommand() {
    return Commands.parallel(
            shooter.spinWhileHeld(),
            feeder.feedWhileHeld())
        .withName("TeleOpShoot");
  }

  /**
   * Same as {@link #teleOpShootCommand()} but shooter speed adjusts based on
   * AprilTag area — closer (larger area) = slower, farther (smaller area) = faster.
   *
   * @param areaSupplier Supplier for current tag area (0-100 from getTA())
   * @return Command that runs area-adjusted shooter + feeder while active
   */
  public Command teleOpShootWithDistanceCommand(DoubleSupplier areaSupplier) {
    return Commands.parallel(
            shooter.spinAtAreaWhileHeld(areaSupplier),
            feeder.feedWhileHeld())
        .withName("TeleOpShootWithDistance");
  }

  /**
   * Teleop: run shooter at area-based speed + feeder simultaneously.
   * Both stop when the command ends (trigger released).
   */
  public Command teleOpShootWithAreaCommand(DoubleSupplier areaSupplier) {
    return Commands.parallel(
            shooter.spinAtAreaWhileHeld(areaSupplier),
            feeder.feedWhileHeld())
        .withName("TeleOpShootWithArea");
  }

  /**
   * Auto: spin up shooter based on tag area, wait until at speed.
   * Reads area once at the moment this command starts.
   */
  public Command spinUpForAreaAndWaitCommand(DoubleSupplier areaSupplier) {
    return shooter.spinUpForArea(areaSupplier)
        .andThen(Commands.waitUntil(() -> shooter.isAtTarget()))
        .withName("SpinUpForAreaAndWait");
  }

  // ==================== State Commands ====================
  // These maintain the same method names used by AutoRoutines.java.
  // Without an arm, position-based commands simply manage the shooter state.

  /** Stop the shooter (safe state for transport). */
  public Command stowCommand() {
    return shooter.stopCommand().withName("Stow");
  }

  /** Stop the shooter and wait until it has fully stopped. */
  public Command stowAndWaitCommand() {
    return shooter.stopCommand()
        .andThen(Commands.waitUntil(() -> shooter.isAtTarget()))
        .withName("StowAndWait");
  }

  /** Stop the shooter (no arm to position for amp). */
  public Command ampScoreCommand() {
    return shooter.stopCommand().withName("AmpScore");
  }

  /** Stop the shooter (no arm to position for amp). */
  public Command ampScoreAndWaitCommand() {
    return shooter.stopCommand().withName("AmpScoreAndWait");
  }

  /**
   * Spin up the shooter for a close speaker shot.
   * The TalonFX holds speed after this command ends.
   */
  public Command speakerCloseCommand() {
    return shooter.spinUpOnce().withName("SpeakerClose");
  }

  /**
   * Spin up the shooter and wait until it has reached target speed.
   */
  public Command speakerCloseAndWaitCommand() {
    return shooter.spinUpOnce()
        .andThen(Commands.waitUntil(() -> shooter.isAtTarget()))
        .withName("SpeakerCloseAndWait");
  }

  /**
   * Spin up the shooter for a far speaker shot.
   * The TalonFX holds speed after this command ends.
   */
  public Command speakerFarCommand() {
    return shooter.spinUpOnce().withName("SpeakerFar");
  }

  /**
   * Spin up the shooter and wait until it has reached target speed.
   */
  public Command speakerFarAndWaitCommand() {
    return shooter.spinUpOnce()
        .andThen(Commands.waitUntil(() -> shooter.isAtTarget()))
        .withName("SpeakerFarAndWait");
  }

  /** Stop the shooter (no arm to move for ground intake). */
  public Command intakeGroundCommand() {
    return shooter.stopCommand().withName("IntakeGround");
  }

  /** Stop the shooter (no arm to move for ground intake). */
  public Command intakeGroundAndWaitCommand() {
    return shooter.stopCommand().withName("IntakeGroundAndWait");
  }

  // ==================== Auto Fire Command ====================

  /**
   * Fire a game piece in autonomous.
   *
   * <p>Runs the feeder to push the ball into the already-spinning shooter wheels.
   * Use this after {@code speakerCloseAndWaitCommand()} so the shooter is at speed.
   *
   * @return Command that feeds a ball through the shooter
   */
  public Command shootCommand() {
    return feeder.feedForTime(ShooterConstants.SHOOT_DURATION_SECONDS).withName("Shoot");
  }
}
