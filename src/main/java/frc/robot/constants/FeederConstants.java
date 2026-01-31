// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Constants for the Feeder subsystem (moves balls from hopper to shooter).
 *
 * <h2>What is a Feeder?</h2>
 * <p>The feeder is the mechanism that moves balls from the storage area (hopper)
 * into the spinning shooter flywheels. Think of it as a "conveyor belt" that
 * delivers balls to the shooter one at a time:
 *
 * <pre>
 *   BALL FLOW THROUGH ROBOT:
 *   ────────────────────────
 *
 *        ┌──────────────┐
 *        │   SHOOTER    │  ← Balls exit here (launched!)
 *        │   ○○○○○○○○   │
 *        └──────┬───────┘
 *               │
 *        ┌──────┴───────┐
 *        │   FEEDER     │  ← Feeder pushes balls UP into shooter
 *        │   ═══════    │     (YOU control this with RB button)
 *        └──────┬───────┘
 *               │
 *        ┌──────┴───────┐
 *        │   HOPPER     │  ← Balls wait here
 *        │   ● ● ●      │
 *        └──────┬───────┘
 *               │
 *        ┌──────┴───────┐
 *        │   INTAKE     │  ← Balls enter here (from floor)
 *        │   ○─────○    │     (YOU control this with LT button)
 *        └──────────────┘
 *               ↑
 *            FLOOR
 * </pre>
 *
 * <h2>Why Separate Feeder from Intake?</h2>
 * <p>Having the feeder as a separate subsystem (not combined with intake) allows:
 * <ul>
 *   <li><b>Independent control:</b> Run intake without feeding to shooter</li>
 *   <li><b>Timing control:</b> Feed balls only when shooter is at speed</li>
 *   <li><b>Rate control:</b> Control how fast balls feed into shooter</li>
 *   <li><b>Reverse capability:</b> Can reverse feeder to clear jams</li>
 * </ul>
 *
 * <h2>Manual Shooting Workflow</h2>
 * <p>The operator controls the feeder manually. Typical sequence:
 *
 * <pre>
 *   STEP 1: Spin up shooter (hold RT)
 *           → Wait for flywheels to reach speed (~1 second)
 *
 *   STEP 2: Feed a ball (press RB)
 *           → Feeder pushes ball into spinning flywheels
 *           → Ball launches!
 *
 *   STEP 3: Repeat Step 2 for more balls
 *           → Keep holding RT while feeding
 *
 *   STEP 4: Release RT when done shooting
 *           → Flywheels spin down
 * </pre>
 *
 * @see frc.robot.subsystems.feeder.Feeder for the feeder subsystem
 * @see ShooterConstants for the shooter that receives balls from feeder
 */
public final class FeederConstants {

  // ==================== Hardware Configuration ====================

  /**
   * CAN ID for the feeder motor.
   *
   * <p>This motor moves balls from the hopper into the shooter.
   */
  public static final int MOTOR_CAN_ID = 23;

  /**
   * Whether the feeder motor should be inverted.
   *
   * <p>If balls go the wrong direction, change this!
   * Positive values should move balls TOWARD the shooter.
   */
  public static final boolean MOTOR_INVERTED = false;

  // ==================== Speed Setpoints ====================

  /**
   * Speed for feeding balls to the shooter (rotations per second).
   *
   * <p>This is how fast the feeder runs when pushing balls to the shooter.
   * <ul>
   *   <li>Too slow: Balls don't reach shooter with enough momentum</li>
   *   <li>Too fast: Multiple balls might jam together</li>
   * </ul>
   *
   * <p><b>TUNE THIS!</b> Start around 20-30 RPS and adjust.
   */
  public static final double FEED_SPEED_RPS = 25.0;

  /**
   * Speed for reversing the feeder (rotations per second).
   *
   * <p>Negative value to run feeder backward (move balls away from shooter).
   * Used to clear jams or move balls back into hopper.
   */
  public static final double REVERSE_SPEED_RPS = -15.0;

  // ==================== Current Limits ====================

  /**
   * Maximum current for the feeder motor (Amps).
   *
   * <p>Feeder motors don't need as much current as shooter motors.
   * Lower limit helps detect jams (current spikes when jammed).
   */
  public static final double CURRENT_LIMIT_AMPS = 30.0;

  // ==================== PID / Feedforward Constants ====================

  /**
   * Static friction compensation (kS).
   */
  public static final double kS = 0.0;

  /**
   * Velocity feedforward gain (kV).
   */
  public static final double kV = 0.125;

  /**
   * Proportional gain (kP).
   */
  public static final double kP = 0.1;

  // ==================== Private Constructor ====================

  /**
   * Private constructor to prevent instantiation.
   */
  private FeederConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
