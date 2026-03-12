package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
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
   * Teleop: run shooter at a speed proportional to trigger pressure.
   * The feeder waits a short delay before feeding to let the shooter spin up.
   * Both stop when the command ends (trigger released).
   *
   * @param triggerSupplier Supplier for the trigger axis value (0.0–1.0)
   */
  public Command teleOpShootWithTriggerCommand(DoubleSupplier triggerSupplier) {
    return Commands.parallel(
            shooter.spinAtTriggerWhileHeld(triggerSupplier),
            Commands.waitSeconds(ShooterConstants.FEEDER_DELAY_SECONDS)
                .andThen(feeder.feedWhileHeld()))
        .withName("TeleOpShootWithTrigger");
  }

  /**
   * Teleop: run shooter at distance-based speed while the trigger is fully pressed.
   * Speed is determined by limelight distance to AprilTag — closer = slower, farther = faster.
   * Recalculates every loop cycle so speed adjusts live as the robot moves.
   * Both stop when the command ends (trigger released).
   *
   * @param distanceSupplier Supplier for current distance to target (meters)
   */
  public Command teleOpShootWithDistanceCommand(DoubleSupplier distanceSupplier) {
    return Commands.parallel(
            shooter.spinAtTeleOpDistanceWhileHeld(distanceSupplier),
            Commands.waitSeconds(ShooterConstants.FEEDER_DELAY_SECONDS)
                .andThen(feeder.feedWhileHeld()))
        .withName("TeleOpShootWithDistance");
  }

  /**
   * Reverse both shooter and feeder to unclog jammed balls.
   * Hold to reverse, release to stop.
   */
  public Command reverseCommand() {
    return Commands.parallel(
            shooter.reverseWhileHeld(),
            feeder.reverseFeedWhileHeld())
        .withName("Reverse");
  }

  /**
   * Auto: spin up shooter based on tag distance (meters), wait until at speed.
   * Closer = slower, farther = faster. Uses the DISTANCE_SPEED_MAP in ShooterConstants.
   */
  public Command spinUpForDistanceAndWaitCommand(DoubleSupplier distanceSupplier) {
    return shooter.spinUpForDistance(distanceSupplier)
        .andThen(Commands.waitUntil(() -> shooter.isAtTarget()))
        .withName("SpinUpForDistanceAndWait");
  }

  /**
   * Auto: set shooter speed from distance, then immediately start feeding after
   * a short delay. Runs for SHOOT_DURATION_SECONDS then stops everything.
   * No waiting for full spin-up — the feeder starts almost right away.
   */
  public Command autoShootByDistanceCommand(DoubleSupplier distanceSupplier) {
    return Commands.parallel(
            shooter.spinAtAutoDistanceWhileHeld(distanceSupplier),
            Commands.waitSeconds(ShooterConstants.FEEDER_DELAY_SECONDS)
                .andThen(feeder.feedWhileHeld()))
        .withTimeout(ShooterConstants.SHOOT_DURATION_SECONDS)
        .withName("AutoShootByDistance");
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
        .andThen(Commands.waitUntil(() ->
            shooter.getVelocity().in(Units.RotationsPerSecond) < ShooterConstants.VELOCITY_TOLERANCE_RPS))
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
