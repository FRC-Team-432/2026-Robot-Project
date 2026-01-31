// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Constants for the Climb subsystem (robot climbing mechanism).
 *
 * <h2>STUB FILE - NOT YET IMPLEMENTED</h2>
 * <p>This file contains placeholder values for the climb system.
 * The climb subsystem will be implemented later once the hardware
 * is finalized and tested.
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
 * <h2>Current Status</h2>
 * <p>The climb commands currently do nothing (placeholder stubs).
 * To implement climbing:
 * <ol>
 *   <li>Update the motor CAN IDs below</li>
 *   <li>Add motor objects to Climb.java</li>
 *   <li>Implement the control logic in each command</li>
 *   <li>Test carefully with the robot on blocks!</li>
 * </ol>
 *
 * @see frc.robot.subsystems.climb.Climb for the climb subsystem stub
 */
public final class ClimbConstants {

  // ==================== Hardware Configuration ====================
  // PLACEHOLDER VALUES - Update when climb hardware is finalized!

  /**
   * CAN ID for the lift/winch motor.
   *
   * <p><b>PLACEHOLDER</b> - Update with actual CAN ID when hardware is ready.
   */
  public static final int LIFT_MOTOR_CAN_ID = 26;

  /**
   * CAN ID for the flip/arm motor.
   *
   * <p><b>PLACEHOLDER</b> - Update with actual CAN ID when hardware is ready.
   */
  public static final int FLIP_MOTOR_CAN_ID = 27;

  // ==================== Speed Setpoints ====================
  // PLACEHOLDER VALUES - Tune these once climb is implemented!

  /**
   * Speed for lifting the robot (rotations per second).
   *
   * <p><b>PLACEHOLDER</b> - Start slow and increase carefully!
   */
  public static final double LIFT_SPEED_RPS = 10.0;

  /**
   * Speed for dropping/lowering the robot (rotations per second).
   *
   * <p><b>PLACEHOLDER</b> - Should be negative to run motor in reverse.
   * Lower speed than lifting for controlled descent.
   */
  public static final double DROP_SPEED_RPS = -5.0;

  /**
   * Speed for flipping the arm up (rotations per second).
   *
   * <p><b>PLACEHOLDER</b>
   */
  public static final double FLIP_UP_SPEED_RPS = 15.0;

  /**
   * Speed for flipping the arm down (rotations per second).
   *
   * <p><b>PLACEHOLDER</b> - Should be negative for reverse direction.
   */
  public static final double FLIP_DOWN_SPEED_RPS = -10.0;

  // ==================== Current Limits ====================

  /**
   * Maximum current for climb motors (Amps).
   *
   * <p>Climb motors may need high current to lift the robot weight.
   * <b>PLACEHOLDER</b> - Adjust based on actual motor load.
   */
  public static final double CURRENT_LIMIT_AMPS = 80.0;

  // ==================== Private Constructor ====================

  /**
   * Private constructor to prevent instantiation.
   */
  private ClimbConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
