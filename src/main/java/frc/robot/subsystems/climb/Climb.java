// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Climb subsystem - Robot climbing mechanism.
 *
 * <h2>STUB IMPLEMENTATION</h2>
 * <p><b>This subsystem is a PLACEHOLDER!</b> The commands below don't actually
 * do anything yet. This stub exists so that:
 * <ul>
 *   <li>Button bindings can be set up in advance</li>
 *   <li>The control layout is complete</li>
 *   <li>Students can see where climb code will go</li>
 * </ul>
 *
 * <h2>What is Climbing?</h2>
 * <p>At the end of FRC matches, robots can earn bonus points by climbing onto
 * elevated structures (bars, chains, platforms). Our climb system has:
 *
 * <pre>
 *   CLIMB MECHANISMS:
 *   ─────────────────
 *
 *   1. LIFT / DROP
 *      A winch or elevator that raises/lowers the robot body
 *
 *   2. FLIP UP / FLIP DOWN
 *      Arms that rotate to reach and hook onto climb bars
 *
 *   ═══════════════ ← Climbing bar
 *          │
 *          │  ← Hook/arm grabs bar
 *         ╱
 *      ──╱
 *   [ROBOT]  ↑ Robot lifts itself up
 * </pre>
 *
 * <h2>How to Implement This</h2>
 * <p>When the climb hardware is ready, you'll need to:
 *
 * <ol>
 *   <li><b>Update ClimbConstants.java:</b>
 *       Set the correct CAN IDs for your climb motors</li>
 *
 *   <li><b>Add motor objects:</b>
 *       <pre>
 *       private final TalonFX liftMotor = new TalonFX(ClimbConstants.LIFT_MOTOR_CAN_ID);
 *       private final TalonFX flipMotor = new TalonFX(ClimbConstants.FLIP_MOTOR_CAN_ID);
 *       </pre></li>
 *
 *   <li><b>Implement the commands:</b>
 *       Replace the "do nothing" commands with real motor control</li>
 *
 *   <li><b>Add safety limits:</b>
 *       Use limit switches or soft limits to prevent over-travel</li>
 *
 *   <li><b>Test carefully!</b>
 *       Put the robot on blocks before testing climb!</li>
 * </ol>
 *
 * <h2>Operator Controls</h2>
 * <pre>
 *   Y button: Flip Up (reach for bar)
 *   A button: Flip Down (retract arm)
 *   X button: Lift Robot (climb up)
 *   B button: Drop Robot (lower down)
 * </pre>
 *
 * @see frc.robot.constants.ClimbConstants for configuration values
 */
@Logged
public class Climb extends SubsystemBase {

  // ==================== Hardware ====================
  // TODO: Add motor objects when hardware is ready
  //
  // Example:
  // private final TalonFX liftMotor =
  //     new TalonFX(ClimbConstants.LIFT_MOTOR_CAN_ID, TunerConstants.kCANBus);
  // private final TalonFX flipMotor =
  //     new TalonFX(ClimbConstants.FLIP_MOTOR_CAN_ID, TunerConstants.kCANBus);

  // ==================== Constructor ====================

  /**
   * Creates a new Climb subsystem (stub).
   *
   * <p>TODO: Add motor configuration when hardware is ready.
   */
  public Climb() {
    // TODO: Configure motors when hardware is ready
    System.out.println("Climb subsystem created (STUB - not yet implemented)");
  }

  // ==================== Periodic ====================

  @Override
  public void periodic() {
    // TODO: Add telemetry when implemented
  }

  // ==================== Command Factory Methods ====================
  // These are STUB commands - they don't do anything yet!

  /**
   * Command to lift the robot (climb up).
   *
   * <p><b>STUB:</b> This command doesn't do anything yet!
   * Implement when climb hardware is ready.
   *
   * <h3>When Implemented:</h3>
   * <p>This should run the lift/winch motor to raise the robot body.
   *
   * @return Command that lifts the robot (currently does nothing)
   */
  public Command liftRobotCommand() {
    return run(() -> {
          // TODO: Implement lift motor control
          // Example: liftMotor.set(ClimbConstants.LIFT_SPEED_RPS);
        })
        .finallyDo(() -> {
          // TODO: Stop lift motor
          // Example: liftMotor.stopMotor();
        })
        .withName("LiftRobot (STUB)");
  }

  /**
   * Command to drop the robot (lower down).
   *
   * <p><b>STUB:</b> This command doesn't do anything yet!
   *
   * @return Command that lowers the robot (currently does nothing)
   */
  public Command dropRobotCommand() {
    return run(() -> {
          // TODO: Implement drop motor control (reverse of lift)
        })
        .finallyDo(() -> {
          // TODO: Stop lift motor
        })
        .withName("DropRobot (STUB)");
  }

  /**
   * Command to flip the climb arm up (reach for bar).
   *
   * <p><b>STUB:</b> This command doesn't do anything yet!
   *
   * @return Command that flips arm up (currently does nothing)
   */
  public Command flipUpCommand() {
    return run(() -> {
          // TODO: Implement flip motor control
        })
        .finallyDo(() -> {
          // TODO: Stop flip motor
        })
        .withName("FlipUp (STUB)");
  }

  /**
   * Command to flip the climb arm down (retract).
   *
   * <p><b>STUB:</b> This command doesn't do anything yet!
   *
   * @return Command that flips arm down (currently does nothing)
   */
  public Command flipDownCommand() {
    return run(() -> {
          // TODO: Implement flip motor control (reverse of flip up)
        })
        .finallyDo(() -> {
          // TODO: Stop flip motor
        })
        .withName("FlipDown (STUB)");
  }

  /**
   * Command to stop all climb motors.
   *
   * <p><b>STUB:</b> This command doesn't do anything yet!
   *
   * @return Command that stops climb (currently does nothing)
   */
  public Command stopCommand() {
    return runOnce(() -> {
          // TODO: Stop all climb motors
          // liftMotor.stopMotor();
          // flipMotor.stopMotor();
        })
        .withName("ClimbStop (STUB)");
  }
}
