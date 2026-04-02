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
  public static final int SHOOTER_FOLLOWER_ID = 7;

  /** CAN ID for the feeder motor (feeds balls up to the shooter at slower speed) */
  public static final int FEEDER_ID = 6;

  // ==================== Shooter Speed ====================

  /**
   * Fixed shooting speed in rotations per second.
   * TODO: Tune this on the real robot — start low and increase until balls reach target.
   */
  public static final double SHOOTER_SPEED_RPS = 100.0;

  // ==================== Feeder Speed ====================

  /**
   * Feeder motor duty cycle (0.0 = stopped, 1.0 = full speed).
   * TODO: Tune this on the real robot — feeder should move slower than shooter.
   */
  public static final double FEEDER_SPEED_PERCENT = 0.75;

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
  // public static final double[][] AREA_SPEED_MAP = {
  //   // { tagAreaPercent, shooterSpeedRPS }
  //   {0.0,  100.0}, // No/tiny target - anchor at max speed so interpolation is monotonically decreasing
  //   {0.5,  100.0}, // Very far away - max speed
  //   {1.0,  90.0},  // Far
  //   {2.0,  80.0},  // Medium-far
  //   {5.0,  70.0},  // Medium
  //   {10.0, 60.0},  // Close
  //   {15.0, 50.0},  // Very close
  // };

  // Speed when no tag is visible (safe medium value)
  public static final double FALLBACK_SPEED_RPS = 80.0;

  // ==================== Distance-Based Shooting ====================
  // Maps Limelight avgTagDistance (meters) to shooter speed (RPS).
  // CLOSER = SLOWER, FARTHER = FASTER.
  // Auto uses lower speeds (robot is stationary + aimed).
  // Teleop uses higher speeds (robot is moving, less precise aim).
  //
  // NOTE: With kV=0.125, the motor saturates at ~96 RPS (12V / 0.125).
  // VALID RANGE: 15 - 85 RPS (above ~85 you're near saturation)

  // --- AUTO (robot stopped, aimed) ---
  public static final double[][] AUTO_DISTANCE_SPEED_MAP = {
    // { distanceMeters, shooterSpeedRPS }
    {0.5,  46.0},   // Very close
    {1.0,  49.0},   // Close
    {1.5,  52.0},   // Medium-close
    {2.0,  55.0},   // Medium
    {2.5,  58.0},   // Medium-far
    {3.0,  61.0},   // Far
    {4.0,  64.0},   // Very far
    {5.0,  67.0},   // Max range
  };

  // --- TELEOP (robot moving, need extra speed) ---
  public static final double[][] TELEOP_DISTANCE_SPEED_MAP = {
    // { distanceMeters, shooterSpeedRPS }
    {0.5,  45.0},   // Very close
    {1.0,  50.0},   // Close
    {1.5,  55.0},   // Medium-close
    {2.0,  60.0},   // Medium
    {2.5,  65.0},   // Medium-far
    {3.0,  70.0},   // Far
    {4.0,  75.0},   // Very far
    {5.0,  80.0},   // Max range
  };

  // Fallback speeds when no tag is visible
  public static final double AUTO_FALLBACK_SPEED_RPS = 52.0;
  public static final double TELEOP_FALLBACK_SPEED_RPS = 65.0;

  // Safety limits
  public static final double MAX_SHOOTER_SPEED_RPS = 85.0;
  public static final double MIN_SHOOTER_SPEED_RPS = 15.0;

  // Delay (seconds) before the feeder starts after the shooter begins spinning.
  // Gives the wheels time to spin up so balls don't stall.
  public static final double FEEDER_DELAY_SECONDS = 1.0;

  // Reverse speed for unclogging (RPS, applied as negative internally)
  public static final double REVERSE_SPEED_RPS = 20.0;

  // Reverse feeder duty cycle for unclogging
  public static final double FEEDER_REVERSE_PERCENT = -0.5;

  // ==================== Auto Fire Duration ====================
 
  /**
   * How long (seconds) the feeder runs during the auto shootCommand().
   * Too short = ball doesn't fully exit. Too long = wasted time in auto.
   * TODO: Tune after testing — 0.5 s is a starting estimate.
   */
  public static final double SHOOT_DURATION_SECONDS = 6.0;

  // ==================== Tolerances ====================

  /**
   * How close the shooter speed needs to be to count as "ready to shoot" (RPS).
   * Looser tolerance = faster ready time. Tighter = more accurate speed.
   */
  public static final double VELOCITY_TOLERANCE_RPS = 20.0;

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
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 500.0;

  /** How fast the shooter can spin up (RPS per second) */
  public static final double MOTION_MAGIC_ACCELERATION = 5000.0;

  private ShooterConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
