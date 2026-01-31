// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Constants for the Shooter subsystem (dual flywheel ball launcher).
 *
 * <h2>What is a Flywheel Shooter?</h2>
 * <p>A flywheel shooter uses spinning wheels to launch balls. When the wheels
 * spin fast enough, a ball fed into them gets accelerated and shot out:
 *
 * <pre>
 *   DUAL FLYWHEEL SHOOTER (our design)
 *   ───────────────────────────────────
 *
 *        ○○○○○○○○○○  ← TOP WHEEL (Motor 1)
 *                      Spins one direction
 *        ══════════
 *           ball  →  →  →  SHOOTS OUT!
 *        ══════════
 *        ○○○○○○○○○○  ← BOTTOM WHEEL (Motor 2)
 *                      Spins opposite direction
 *
 *   Both wheels spin to grip the ball and accelerate it.
 *   The ball is squeezed between the wheels and launched!
 * </pre>
 *
 * <h2>Why Two Wheels?</h2>
 * <p>A dual flywheel design has advantages over a single wheel:
 * <ul>
 *   <li><b>More grip:</b> Ball is gripped on both sides</li>
 *   <li><b>More power:</b> Two motors = more acceleration</li>
 *   <li><b>Spin control:</b> Can add backspin by running wheels at different speeds</li>
 *   <li><b>Consistency:</b> Ball exits more consistently</li>
 * </ul>
 *
 * <h2>Velocity Control</h2>
 * <p>Like the intake, we use <b>velocity control</b> instead of just "on/off".
 * This means we tell the motors HOW FAST to spin (in rotations per second),
 * and they automatically adjust power to maintain that speed.
 *
 * <p>Why this matters for shooting:
 * <ul>
 *   <li>Consistent shot speed even as battery drains</li>
 *   <li>Can check if wheels are "at speed" before feeding balls</li>
 *   <li>Different speeds for different shot distances (future feature)</li>
 * </ul>
 *
 * <h2>Spin-Up Time</h2>
 * <p>Flywheels take time to reach full speed (typically 0.5-2 seconds).
 * The operator should hold the spin-up button BEFORE trying to shoot,
 * and keep holding it while shooting.
 *
 * <pre>
 *   SHOOTING SEQUENCE:
 *   ──────────────────
 *   1. Hold RT (spin up)     → Wheels start spinning
 *   2. Wait ~1 second        → Wheels reach full speed
 *   3. Press RB (feed)       → Ball enters shooter
 *   4. Ball shoots!          → Keep holding RT for next shot
 *   5. Release RT when done  → Wheels spin down
 * </pre>
 *
 * @see frc.robot.subsystems.shooter.Shooter for the shooter subsystem
 * @see FeederConstants for the feeder that sends balls to the shooter
 */
public final class ShooterConstants {

  // ==================== Hardware Configuration ====================

  /**
   * CAN ID for the TOP flywheel motor.
   *
   * <p>This motor spins the top wheel of the dual flywheel system.
   */
  public static final int TOP_MOTOR_CAN_ID = 24;

  /**
   * CAN ID for the BOTTOM flywheel motor.
   *
   * <p>This motor spins the bottom wheel of the dual flywheel system.
   */
  public static final int BOTTOM_MOTOR_CAN_ID = 25;

  /**
   * Whether the top motor should be inverted.
   *
   * <p>In a dual flywheel, the wheels typically spin in OPPOSITE directions
   * (both spinning inward to grip the ball). Depending on motor mounting,
   * you may need to invert one motor.
   *
   * <p>If balls shoot backward, try changing this!
   */
  public static final boolean TOP_MOTOR_INVERTED = false;

  /**
   * Whether the bottom motor should be inverted.
   *
   * <p>Usually the opposite of the top motor inversion.
   */
  public static final boolean BOTTOM_MOTOR_INVERTED = true;

  // ==================== Speed Setpoints ====================

  /**
   * Default shooting speed (rotations per second).
   *
   * <p>This is the speed both flywheels will spin at for shooting.
   * Higher speeds = faster/farther shots, but take longer to spin up.
   *
   * <p>Typical FRC shooter speeds: 40-80 RPS
   *
   * <p><b>TUNE THIS!</b> Start low (40 RPS) and increase until shots
   * reach the target consistently.
   */
  public static final double SHOOTING_SPEED_RPS = 60.0;

  /**
   * Idle speed when shooter is "ready" but not actively shooting (RPS).
   *
   * <p>Set to 0 to fully stop when not shooting.
   * Set to a low value (10-20) to keep wheels warm for faster spin-up.
   *
   * <p>For now, we use 0 (full stop) for simplicity and safety.
   */
  public static final double IDLE_SPEED_RPS = 0.0;

  // ==================== Tolerances ====================

  /**
   * How close to target speed to be considered "at speed" (RPS).
   *
   * <p>The shooter is "ready" when actual speed is within this tolerance
   * of the target speed. Used to know when it's safe to feed balls.
   *
   * <p>Example: Target is 60 RPS, tolerance is 3 RPS
   * → Shooter is "ready" when speed is 57-63 RPS
   */
  public static final double VELOCITY_TOLERANCE_RPS = 3.0;

  /**
   * Percentage of target speed to be considered "at speed" (0.0 to 1.0).
   *
   * <p>Alternative to absolute tolerance. Shooter is "ready" when
   * actual speed is at least this percentage of target.
   *
   * <p>0.95 = 95% of target speed
   */
  public static final double VELOCITY_TOLERANCE_PERCENT = 0.95;

  // ==================== Current Limits ====================

  /**
   * Maximum current for each shooter motor (Amps).
   *
   * <p>Shooter motors can draw a lot of current when spinning up.
   * This limit protects the motors and prevents battery brownouts.
   *
   * <p>60A is typical for shooter motors. Increase if spin-up is too slow,
   * decrease if you're having brownout issues.
   */
  public static final double CURRENT_LIMIT_AMPS = 60.0;

  // ==================== PID / Feedforward Constants ====================
  // These tune HOW the motors reach and maintain target speed

  /**
   * Static friction compensation (kS).
   *
   * <p>Voltage needed just to START the motor moving.
   * For flywheels, this is usually small (0.1-0.3).
   */
  public static final double kS = 0.1;

  /**
   * Velocity feedforward gain (kV).
   *
   * <p>Main "gas pedal" for velocity control.
   * voltage_needed ≈ kV * target_speed
   *
   * <p>For a typical FRC motor at ~100 RPS max:
   * 12V / 100 RPS ≈ 0.12 kV
   */
  public static final double kV = 0.12;

  /**
   * Proportional gain (kP).
   *
   * <p>Corrects errors between actual and target speed.
   * Start low (0.1) and increase if the shooter doesn't reach target speed.
   */
  public static final double kP = 0.2;

  /**
   * Derivative gain (kD).
   *
   * <p>Dampens oscillations. Usually 0 or very small for flywheels.
   */
  public static final double kD = 0.0;

  // ==================== Private Constructor ====================

  /**
   * Private constructor to prevent instantiation.
   */
  private ShooterConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
