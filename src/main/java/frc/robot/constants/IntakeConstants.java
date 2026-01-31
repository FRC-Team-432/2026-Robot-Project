// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Constants for the Intake subsystem (roller mechanism that picks up game pieces).
 *
 * <h2>What is an Intake?</h2>
 * <p>An intake is a mechanism that brings game pieces (balls, cubes, cones, etc.) from
 * the floor into the robot. Think of it like a vacuum cleaner that sucks up objects!
 *
 * <h2>How Our Intake Works</h2>
 * <p>Our intake uses:
 * <ul>
 *   <li><b>Sushi rolls/composite wheels</b> - Soft, grippy wheels on a hex shaft that
 *       grab balls gently without damaging them
 *   <li><b>Single TalonFX motor</b> - Spins the wheels to pull balls in (or push them out)
 *   <li><b>Velocity control</b> - We control HOW FAST the wheels spin, not just on/off
 * </ul>
 *
 * <h2>Why Use Velocity Control?</h2>
 * <p>Instead of just "motor on" or "motor off", velocity control lets us:
 * <ul>
 *   <li>Spin at a consistent speed regardless of battery voltage
 *   <li>Spin faster or slower depending on the situation
 *   <li>Know when the intake is actually spinning at the right speed
 * </ul>
 *
 * <h2>Diagram</h2>
 * <pre>
 *     [HOPPER/SHOOTER]
 *          ↑ balls go up
 *     ┌────┴────┐
 *     │ ○─────○ │  ← Intake rollers (sushi rolls on hex shaft)
 *     └────┬────┘      Spin to grab balls from floor
 *          │
 *     [FLOOR/BALLS]
 * </pre>
 *
 * <h2>What Each Constant Means</h2>
 * <p>See the comments below for detailed explanations of each value.
 */
public final class IntakeConstants {

  // ==================== Hardware Configuration ====================
  // These tell the code which motor to talk to

  /**
   * CAN ID for the intake motor.
   *
   * <p>Every motor on the robot has a unique ID number (like a phone number).
   * This ID must match what's set in Phoenix Tuner on the actual motor controller.
   *
   * <p>We use ID 22 to keep intake motors grouped together:
   * <ul>
   *   <li>Flywheel: ID 21</li>
   *   <li>Intake: ID 22</li>
   * </ul>
   */
  public static final int MOTOR_CAN_ID = 22;

  // ==================== Speed Setpoints ====================
  // These control how fast the intake spins in different situations

  /**
   * Speed for intaking game pieces (rotations per second).
   *
   * <p>This is the speed we want when picking up balls from the floor.
   * Positive values spin the rollers to pull balls INTO the robot.
   *
   * <p>Too slow = balls don't get grabbed reliably
   * <p>Too fast = balls might bounce around or get damaged
   *
   * <p>15 RPS is a good starting point - tune this on the actual robot!
   */
  public static final double INTAKE_SPEED_RPS = 5.0;

  /**
   * Speed for ejecting/outtaking game pieces (rotations per second).
   *
   * <p>This is the speed when we want to PUSH balls OUT of the robot
   * (for example, if we picked up the wrong color or need to clear a jam).
   *
   * <p>Negative values spin the rollers in reverse to eject balls.
   * We use a slower speed than intake because ejecting doesn't need to be fast.
   */
  public static final double OUTTAKE_SPEED_RPS = -10.0;

  // ==================== Tolerances ====================
  // These define "close enough" for our speed checks

  /**
   * How close the actual speed needs to be to the target to count as "at speed" (RPS).
   *
   * <p>Motors can't hit an exact speed perfectly - there's always tiny variations.
   * This tolerance says "if we're within 1 RPS of our target, that's good enough."
   *
   * <p>Example: If target is 15 RPS and actual is 14.5 RPS, we're "at speed"
   * because 15 - 14.5 = 0.5, which is less than our 1.0 tolerance.
   */
  public static final double VELOCITY_TOLERANCE_RPS = 1.0;

  // ==================== Current Limits ====================
  // These protect the motor and mechanism from damage

  /**
   * Maximum current the motor can draw (in Amps).
   *
   * <p>Current limiting protects against:
   * <ul>
   *   <li>Motor burnout from drawing too much power</li>
   *   <li>Mechanism damage if something gets jammed</li>
   *   <li>Battery brownouts from sudden power spikes</li>
   * </ul>
   *
   * <p>40 Amps is a safe limit for a single intake roller. If the motor hits this
   * limit (like when a ball gets stuck), it will reduce power automatically.
   */
  public static final double CURRENT_LIMIT_AMPS = 40.0;

  // ==================== PID / Feedforward Control Values ====================
  // These tune HOW the motor reaches and maintains the target speed

  /**
   * Static friction compensation (kS).
   *
   * <p>This is the voltage needed just to START the motor moving.
   * Think of it like the initial push needed to get a heavy door moving -
   * once it's moving, it takes less effort to keep it going.
   *
   * <p>Set to 0 initially. If the intake is slow to start spinning,
   * try increasing this (typical values: 0.1 to 0.5).
   */
  public static final double kS = 0.0;

  /**
   * Velocity feedforward gain (kV).
   *
   * <p>This predicts how much voltage is needed for a given speed.
   * It's the main "gas pedal" for velocity control.
   *
   * <p>Formula: voltage_needed = kV * target_speed
   *
   * <p>For a typical FRC motor:
   * <ul>
   *   <li>12V battery / 100 RPS max speed ≈ 0.12 kV</li>
   *   <li>We use 0.125 as a starting point</li>
   * </ul>
   *
   * <p>If the intake spins too slow, increase kV. Too fast, decrease it.
   */
  public static final double kV = 0.125;

  /**
   * Proportional gain (kP) - corrects speed errors.
   *
   * <p>If the actual speed doesn't match the target, kP adds extra voltage
   * to fix the error. Think of it like cruise control on a car.
   *
   * <p>Higher kP = faster correction, but can cause oscillation (wobbling)
   * Lower kP = slower correction, but more stable
   *
   * <p>Start with a small value (0.1) and increase if the intake
   * doesn't reach target speed quickly enough.
   */
  public static final double kP = 0.1;

  // ==================== Private Constructor ====================
  // Prevents anyone from creating an instance of this class

  /**
   * Private constructor to prevent instantiation.
   *
   * <p>This class only contains constants (static final values), so there's
   * no reason to create an instance of it. Making the constructor private
   * and throwing an exception ensures no one accidentally tries to do:
   * {@code IntakeConstants constants = new IntakeConstants(); // ERROR!}
   */
  private IntakeConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
