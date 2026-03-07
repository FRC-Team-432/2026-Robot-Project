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
  public static final int FEEDER_ID = 7;

  // ==================== Shooter Speed ====================

  /**
   * Fixed shooting speed in rotations per second (fallback for non-area-based modes).
   * TODO: Tune this on the real robot — start low and increase until balls reach target.
   */
  public static final double SHOOTER_SPEED_RPS = 35.0;

  // ==================== Feeder Speed ====================

  /**
   * Feeder motor duty cycle (0.0 = stopped, 1.0 = full speed).
   * TODO: Tune this on the real robot — feeder should move slower than shooter.
   */
  public static final double FEEDER_SPEED_PERCENT = 0.2;

  // ==================== Area-Based Shooting ====================
  // Maps Limelight target area (getTA(), 0-100 scale) to shooter speed (RPS).
  // LARGER area = CLOSER to target = SLOWER speed.
  // SMALLER area = FARTHER from target = FASTER speed.
  //
  // HOW TO TUNE AT COMPETITION:
  //   1. Drive robot to a shooting distance
  //   2. Read "Limelight/TA" from SmartDashboard (this is the area value)
  //   3. Manually adjust shooter speed until shots score consistently
  //   4. Record the {area, speed} pair below
  //   5. Repeat at 3-4 different distances
  //
  // These are PLACEHOLDER values - you MUST tune them on the real robot.
  public static final double[][] AREA_SPEED_MAP = {
    // { tagAreaPercent, shooterSpeedRPS }
    {0.0,  35.0},  // No/tiny target - use fallback speed
    {0.5,  45.0},  // Very far away - high speed
    {1.0,  40.0},  // Far
    {2.0,  35.0},  // Medium-far
    {5.0,  28.0},  // Medium
    {10.0, 22.0},  // Close
    {15.0, 18.0},  // Very close - low speed
  };

  // Speed when no tag is visible (safe medium value)
  public static final double FALLBACK_SPEED_RPS = 35.0;

  // Safety limits
  public static final double MAX_SHOOTER_SPEED_RPS = 50.0;
  public static final double MIN_SHOOTER_SPEED_RPS = 15.0;

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

  /** Maximum shooter speed (RPS) — acts as a safety ceiling (must be >= MAX_SHOOTER_SPEED_RPS) */
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 60.0;

  /** How fast the shooter can spin up (RPS per second) */
  public static final double MOTION_MAGIC_ACCELERATION = 1000.0;

  private ShooterConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
