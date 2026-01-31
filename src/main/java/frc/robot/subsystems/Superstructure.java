package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.intake.Intake;

/**
 * Superstructure - Controls the Arm, Flywheel, and Intake together.
 *
 * <p>This coordinates:
 *
 * <ul>
 *   <li>Arm - Moves horizontal and vertical to position game pieces
 *   <li>Flywheel - Spins the shooter wheels at the right speed
 *   <li>Intake - Pulls game pieces from the floor into the robot
 * </ul>
 *
 * <p>Instead of controlling mechanisms separately, this gives you simple
 * commands like "score low" or "prepare for shooting" that move all parts together.
 * This makes driving easier and ensures everything moves in sync.
 *
 * <h2>Why Use a Superstructure?</h2>
 * <p>Coordinated commands prevent conflicts. For example:
 * <ul>
 *   <li>You don't want the intake running while the arm is in the way</li>
 *   <li>You want to stop the intake when starting to shoot</li>
 *   <li>The arm should be in position before the flywheel spins up</li>
 * </ul>
 *
 * <p>The Superstructure handles all this coordination so the driver doesn't have to!
 */
@Logged
public class Superstructure extends SubsystemBase {

  // ==================== Subsystems ====================
  private final Arm arm;
  private final Flywheel flywheel;
  private final Intake intake;

  // ==================== Constructor ====================

  /**
   * Creates a new Superstructure to coordinate all mechanisms.
   *
   * @param arm The arm subsystem
   * @param flywheel The flywheel/shooter subsystem
   * @param intake The intake subsystem (for collecting game pieces)
   */
  public Superstructure(Arm arm, Flywheel flywheel, Intake intake) {
    this.arm = arm;
    this.flywheel = flywheel;
    this.intake = intake;
  }

  // ==================== Coordinated Commands ====================

  /**
   * Command to safely stow the robot for transport.
   * Moves arm to vertical position, stops the flywheel, and stops the intake.
   */
  public Command stowCommand() {
    return Commands.parallel(
            arm.vertical(),
            flywheel.stopCommand(),
            intake.stopCommand())
        .withName("Stow");
  }

  /**
   * Command to stow and wait until all mechanisms reach target.
   * Waits for arm to reach vertical, flywheel to stop, and intake to stop.
   */
  public Command stowAndWaitCommand() {
    return Commands.parallel(
            arm.vertical(),
            flywheel.stopCommand(),
            intake.stopCommand())
        .andThen(Commands.waitUntil(() -> arm.isAtTarget() && flywheel.isAtTarget()))
        .withName("StowAndWait");
  }

  /**
   * Command to score in the amp (controlled slow spin).
   * Moves arm to vertical and spins flywheel slowly for controlled scoring.
   */
  public Command ampScoreCommand() {
    return Commands.parallel(
            arm.vertical(),
            flywheel.ampSpeed())
        .withName("AmpScore");
  }

  /**
   * Command to score in amp and wait until ready.
   * Waits for arm to reach vertical and flywheel to reach amp speed.
   */
  public Command ampScoreAndWaitCommand() {
    return Commands.parallel(
            arm.vertical(),
            flywheel.ampSpeed())
        .andThen(Commands.waitUntil(() -> arm.isAtTarget() && flywheel.isAtTarget()))
        .withName("AmpScoreAndWait");
  }

  /**
   * Command to prepare for close speaker shot.
   * Moves arm to scoring angle (30°) and spins flywheel at medium speed (25 RPS).
   */
  public Command speakerCloseCommand() {
    return Commands.parallel(
            arm.scoringPosition(),
            flywheel.spinUp())
        .withName("SpeakerClose");
  }

  /**
   * Command to prepare for close speaker shot and wait until ready.
   * Waits for arm to reach scoring position and flywheel to reach speed.
   */
  public Command speakerCloseAndWaitCommand() {
    return Commands.parallel(
            arm.scoringPosition(),
            flywheel.spinUp())
        .andThen(Commands.waitUntil(() -> arm.isAtTarget() && flywheel.isAtTarget()))
        .withName("SpeakerCloseAndWait");
  }

  /**
   * Command to prepare for far speaker shot.
   * Moves arm to high angle (45°) and spins flywheel fast (35 RPS).
   */
  public Command speakerFarCommand() {
    return Commands.parallel(
            arm.scoringHighPosition(),
            flywheel.farSpeed())
        .withName("SpeakerFar");
  }

  /**
   * Command to prepare for far speaker shot and wait until ready.
   * Waits for arm to reach high position and flywheel to reach fast speed.
   */
  public Command speakerFarAndWaitCommand() {
    return Commands.parallel(
            arm.scoringHighPosition(),
            flywheel.farSpeed())
        .andThen(Commands.waitUntil(() -> arm.isAtTarget() && flywheel.isAtTarget()))
        .withName("SpeakerFarAndWait");
  }

  /**
   * Command to prepare for ground intake.
   * Moves arm to horizontal position, stops flywheel, and stops intake.
   *
   * <p>Note: This just POSITIONS the robot for intake. Use the intake buttons
   * (or intakeFromGroundCommand) to actually run the intake.
   */
  public Command intakeGroundCommand() {
    return Commands.parallel(
            arm.horizontal(),
            flywheel.stopCommand())
        .withName("IntakeGround");
  }

  /**
   * Command to prepare for ground intake and wait until ready.
   * Waits for arm to reach horizontal and flywheel to stop.
   */
  public Command intakeGroundAndWaitCommand() {
    return Commands.parallel(
            arm.horizontal(),
            flywheel.stopCommand())
        .andThen(Commands.waitUntil(() -> arm.isAtTarget() && flywheel.isAtTarget()))
        .withName("IntakeGroundAndWait");
  }

  // ==================== Intake Coordinated Commands ====================

  /**
   * Command to run the full ground intake sequence.
   *
   * <p>This command:
   * <ol>
   *   <li>Moves arm to horizontal (intake position)</li>
   *   <li>Waits for arm to reach position</li>
   *   <li>Runs the intake until cancelled</li>
   *   <li>Stops intake when command ends (from button release or interrupt)</li>
   * </ol>
   *
   * <p><b>Usage:</b> Bind to a button with whileTrue() or toggleOnTrue()
   * <pre>
   * joystick.a().whileTrue(superstructure.intakeFromGroundCommand());
   * </pre>
   *
   * @return Command that positions arm and runs intake
   */
  public Command intakeFromGroundCommand() {
    return Commands.sequence(
            // First: Position the arm
            arm.horizontal(),
            // Wait until arm is in position
            Commands.waitUntil(() -> arm.isAtTarget()),
            // Then: Run intake (uses startEnd so it stops when command ends)
            intake.intakeCommand())
        .withName("IntakeFromGround");
  }

  /**
   * Command to eject game pieces from the intake.
   *
   * <p>Use this to clear a jam or eject an unwanted game piece.
   * Works similarly to intakeFromGroundCommand but in reverse.
   *
   * @return Command that runs outtake and stops when finished
   */
  public Command ejectCommand() {
    return intake.outtakeCommand().withName("Eject");
  }

  /**
   * Command to stop the intake immediately.
   *
   * <p>Use this for emergency stops or explicit manual control.
   *
   * @return Command that stops the intake
   */
  public Command stopIntakeCommand() {
    return intake.stopCommand().withName("StopIntake");
  }

  // ==================== Action Commands ====================

  /**
   * Command to shoot a game piece.
   * Waits briefly for the game piece to be expelled from the robot.
   *
   * <p>This represents the time needed for the spinning flywheel to launch the game piece.
   * Use this after preparing the shooter with speaker commands.
   */
  public Command shootCommand() {
    return Commands.waitSeconds(0.3).withName("Shoot");
  }
}
