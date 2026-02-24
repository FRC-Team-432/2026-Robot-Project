// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Settings for the Shooter (two shooting motors + feeder motor).
 *
 * <p>All speed and CAN ID values here are adjustable placeholders.
 * Tune them once the physical robot is available.
 *
 * <ul>
 *   <li>CAN IDs — set to the actual motor controller IDs once assigned
 *   <li>SHOOTER_SPEED_RPS — target flywheel speed in rotations per second
 *   <li>FEEDER_SPEED_PERCENT — feeder duty cycle from 0.0 (stopped) to 1.0 (full)
 *   <li>DISTANCE_SPEED_MAP — lookup table for distance-based speed (bonus feature)
 * </ul>
 */
public final class ShooterConstants {

  // ==================== CAN IDs ====================
  // TODO: Set these to the correct motor controller IDs once assigned.
  // These motors are on the RoboRIO CAN bus (not CANivore).

  /** CAN ID for the first shooter motor (leader) */
  public static final int SHOOTER_LEADER_ID = 5;

  /** CAN ID for the second shooter motor (follower, runs opposite direction to grip ball) */
  public static final int SHOOTER_FOLLOWER_ID = 6;

  /** CAN ID for the feeder motor (feeds balls up to the shooter at slower speed) */
  public static final int FEEDER_ID = 40;

  // ==================== Shooter Speed ====================

  /**
   * Fixed shooting speed in rotations per second.
   * TODO: Tune this on the real robot — start low and increase until balls reach target.
   */
  public static final double SHOOTER_SPEED_RPS = 1700.0;

  // ==================== Feeder Speed ====================

  /**
   * Feeder motor duty cycle (0.0 = stopped, 1.0 = full speed).
   * TODO: Tune this on the real robot — feeder should move slower than shooter.
   */
  public static final double FEEDER_SPEED_PERCENT = 0.2;

  // ==================== Distance-Based Shooting (Bonus) ====================
  // Maps distance from target (meters) to required shooter speed (RPS).
  // The robot interpolates between these points automatically.
  //
  // TODO: Test and fill in real values after characterizing on the robot.
  //       Add more rows for better accuracy at more distances.
  //       Format: { distanceMeters, shooterSpeedRPS }

  public static final double[][] DISTANCE_SPEED_MAP = {
    {1.0, 20.0}, // 1 meter from target → 20 RPS
    {2.0, 25.0}, // 2 meters from target → 25 RPS
    {3.0, 30.0}, // 3 meters from target → 30 RPS
    {4.0, 35.0}, // 4 meters from target → 35 RPS
    {5.0, 40.0}, // 5 meters from target → 40 RPS
  };

  // ==================== Auto Fire Duration ====================

  /**
   * How long (seconds) the feeder runs during the auto shootCommand().
   * Too short = ball doesn't fully exit. Too long = wasted time in auto.
   * TODO: Tune after testing — 0.5 s is a starting estimate.
   */
  public static final double SHOOT_DURATION_SECONDS = 0.5;

  // ==================== Tolerances ====================

  /**
   * How close the shooter speed needs to be to count as "ready to shoot" (RPS).
   * Looser tolerance = faster ready time. Tighter = more accurate speed.
   */
  public static final double VELOCITY_TOLERANCE_RPS = 1.0;

  // ==================== PID / Feedforward Control Values ====================
  // These control how accurately the shooter holds its target speed.
  // TODO: Tune on real robot. Start with kV only, then add kS and kP as needed.

  /** Static friction compensation — voltage needed to just barely move the motor */
  public static final double kS = 0.0;

  /** Velocity feedforward — predicts the voltage needed for a given speed */
  public static final double kV = 0.125;

  /** Proportional gain — corrects speed errors */
  public static final double kP = 0.0;

  // ==================== Motion Magic Limits ====================

  /** Maximum shooter speed (RPS) — acts as a safety ceiling */
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 100.0;

  /** How fast the shooter can spin up (RPS per second) */
  public static final double MOTION_MAGIC_ACCELERATION = 1000.0;

  private ShooterConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
