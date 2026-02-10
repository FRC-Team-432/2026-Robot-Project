// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Constants for the Climb subsystem (robot climbing mechanism).
 *
 * <h2>What is Climbing?</h2>
 * <p>At the end of FRC matches, robots can earn extra points by climbing
 * onto elevated structures. Our climb system has two mechanisms:
 *
 * <pre>
 *   CLIMB MECHANISMS:
 *   ─────────────────
 *
 *   1. LIFT / DROP (Winch System)
 *      ────────────────────────────
 *      Raises and lowers the robot body
 *
 *           ════════ climbing bar
 *              │
 *              │ winch rope
 *              │
 *           [ROBOT] ↑ LIFT (X button)
 *                   ↓ DROP (B button)
 *
 *   2. FLIP UP / FLIP DOWN (Arm System)
 *      ──────────────────────────────────
 *      Rotates an arm to reach the bar
 *
 *           ════════ climbing bar
 *              ╱
 *           ──╱  arm flips UP (Y button)
 *           [ROBOT]
 *              ╲
 *           ──╲  arm flips DOWN (A button)
 * </pre>
 *
 * @see frc.robot.subsystems.climb.Climb for the climb subsystem
 */
public final class ClimbConstants {

  // ==================== Hardware Configuration ====================

  /**
   * CAN ID for the lift/winch motor.
   */
  public static final int LIFT_MOTOR_CAN_ID = 26;

  /**
   * CAN ID for the flip/arm motor.
   */
  public static final int FLIP_MOTOR_CAN_ID = 27;

  // ==================== Motor Direction ====================

  /**
   * Whether the lift motor should be inverted.
   * 
   * <p>Set to true if positive values make the robot go DOWN instead of UP.
   */
  public static final boolean LIFT_MOTOR_INVERTED = false;

  /**
   * Whether the flip motor should be inverted.
   * 
   * <p>Set to true if positive values make the arm flip DOWN instead of UP.
   */
  public static final boolean FLIP_MOTOR_INVERTED = false;

  // ==================== Speed Setpoints ====================

  /**
   * Speed for lifting the robot (rotations per second).
   *
   * <p>Start slow and increase carefully during testing!
   */
  public static final double LIFT_SPEED_RPS = 10.0;

  /**
   * Speed for dropping/lowering the robot (rotations per second).
   *
   * <p>Negative to run motor in reverse. Lower speed for controlled descent.
   */
  public static final double DROP_SPEED_RPS = -5.0;

  /**
   * Speed for flipping the arm up (rotations per second).
   */
  public static final double FLIP_UP_SPEED_RPS = 15.0;

  /**
   * Speed for flipping the arm down (rotations per second).
   *
   * <p>Negative for reverse direction.
   */
  public static final double FLIP_DOWN_SPEED_RPS = -10.0;

  // ==================== Current Limits ====================

  /**
   * Maximum current for LIFT motor (Amps).
   *
   * <p>Climb motors may need high current to lift the robot weight.
   */
  public static final double LIFT_CURRENT_LIMIT_AMPS = 80.0;

  /**
   * Maximum current for FLIP motor (Amps).
   */
  public static final double FLIP_CURRENT_LIMIT_AMPS = 60.0;

  // ==================== Velocity Control Gains ====================
  // These control how the motors reach and maintain target speeds

  // ----- Lift Motor Gains -----

  /**
   * Static friction compensation for lift motor.
   */
  public static final double LIFT_kS = 0.1;

  /**
   * Velocity feedforward for lift motor.
   */
  public static final double LIFT_kV = 0.12;

  /**
   * Proportional gain for lift motor.
   */
  public static final double LIFT_kP = 0.5;

  // ----- Flip Motor Gains -----

  /**
   * Static friction compensation for flip motor.
   */
  public static final double FLIP_kS = 0.1;

  /**
   * Velocity feedforward for flip motor.
   */
  public static final double FLIP_kV = 0.12;

  /**
   * Proportional gain for flip motor.
   */
  public static final double FLIP_kP = 0.5;

  // ==================== Private Constructor ====================

  /**
   * Private constructor to prevent instantiation.
   */
  private ClimbConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}