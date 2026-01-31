// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Constants for the drive system and driver controls.
 *
 * <h2>What is a Swerve Drive?</h2>
 * <p>A <b>swerve drive</b> is a special type of drivetrain where each wheel can rotate
 * independently AND steer independently. This gives the robot incredible maneuverability:
 *
 * <pre>
 *   TANK DRIVE (simple)          SWERVE DRIVE (our robot!)
 *   ┌─────────────────┐          ┌─────────────────┐
 *   │  ▓▓▓      ▓▓▓  │          │  ◇           ◇  │  ← Wheels can rotate 360°
 *   │  left     right │          │                 │
 *   │                 │          │                 │
 *   │  ▓▓▓      ▓▓▓  │          │  ◇           ◇  │
 *   └─────────────────┘          └─────────────────┘
 *
 *   Tank: Can only go forward/    Swerve: Can go ANY direction
 *   backward, must turn to           instantly - forward, sideways,
 *   change direction                 diagonal, spin in place!
 * </pre>
 *
 * <h2>How Swerve Movement Works</h2>
 * <p>Swerve drive has THREE types of motion that can happen AT THE SAME TIME:
 *
 * <pre>
 *   1. TRANSLATION X (Forward/Backward)
 *      ────────────────────────────────
 *      Push left stick UP = drive FORWARD
 *      Push left stick DOWN = drive BACKWARD
 *
 *          ↑ forward
 *          │
 *      [ROBOT]
 *          │
 *          ↓ backward
 *
 *   2. TRANSLATION Y (Left/Right - "Strafing")
 *      ─────────────────────────────────────────
 *      Push left stick LEFT = strafe LEFT
 *      Push left stick RIGHT = strafe RIGHT
 *
 *      ← left   [ROBOT]   right →
 *
 *      The robot slides sideways WITHOUT turning!
 *      This is impossible with tank drive.
 *
 *   3. ROTATION (Spinning)
 *      ────────────────────
 *      Push right stick LEFT = spin counter-clockwise
 *      Push right stick RIGHT = spin clockwise
 *
 *           ↺ CCW    [ROBOT]    CW ↻
 * </pre>
 *
 * <h2>Field-Centric vs Robot-Centric</h2>
 * <p>There are two ways to interpret joystick input:
 *
 * <pre>
 *   ROBOT-CENTRIC                    FIELD-CENTRIC (default)
 *   ──────────────                   ─────────────────────────
 *   "Forward" means forward          "Forward" ALWAYS means toward
 *   relative to the ROBOT            the FAR end of the field
 *
 *   If robot is facing right:        If robot is facing right:
 *   Push up → robot goes RIGHT       Push up → robot goes FORWARD
 *                                    (toward far end of field)
 *
 *   Confusing when robot             Intuitive! Forward is always
 *   turns around                     the same direction
 * </pre>
 *
 * <p>We use FIELD-CENTRIC by default because it's much easier to drive!
 *
 * <h2>Deadband Explained</h2>
 * <p>Controllers are never perfectly centered - they always have tiny movements
 * even when you're not touching them. The "deadband" ignores small inputs:
 *
 * <pre>
 *   Without deadband:              With deadband (0.1):
 *   ──────────────────             ────────────────────
 *   Joystick at 0.02 →             Joystick at 0.02 →
 *   Robot creeps slowly            Robot stays still (ignored)
 *
 *   Joystick at 0.15 →             Joystick at 0.15 →
 *   Robot moves                    Robot moves
 *
 *   The deadband "dead zone" is the area where input is ignored.
 * </pre>
 *
 * @see frc.robot.RobotContainer for how these constants are used
 */
public final class DriveConstants {

  // ==================== Speed Limits ====================
  // These control the maximum speeds the robot can reach

  /**
   * Multiplier for slow mode (0.0 to 1.0).
   *
   * <p>When the driver holds the slow mode trigger (RT), all speeds are
   * multiplied by this value. This gives precise control for:
   * <ul>
   *   <li>Lining up for scoring</li>
   *   <li>Navigating tight spaces</li>
   *   <li>Picking up game pieces</li>
   * </ul>
   *
   * <p>0.5 = 50% speed (half speed)
   * <p>0.3 = 30% speed (very slow, very precise)
   */
  public static final double SLOW_MODE_MULTIPLIER = 0.5;

  /**
   * Multiplier for normal driving (0.0 to 1.0).
   *
   * <p>This caps the normal driving speed. Set to 1.0 for full speed,
   * or lower for a safer maximum (good for practice/learning).
   *
   * <p>0.8 = 80% of maximum robot speed
   */
  public static final double NORMAL_MODE_MULTIPLIER = 0.8;

  // ==================== Input Processing ====================
  // These control how joystick inputs are interpreted

  /**
   * Deadband for joystick inputs.
   *
   * <p>Any joystick input with absolute value less than this is treated as 0.
   * This prevents the robot from drifting when the joysticks are released
   * but not perfectly centered.
   *
   * <p>Typical values: 0.05 to 0.15
   * <ul>
   *   <li>Too small (0.02): Robot might drift with stick "centered"</li>
   *   <li>Too large (0.3): Need to push stick far before robot responds</li>
   * </ul>
   */
  public static final double JOYSTICK_DEADBAND = 0.1;

  /**
   * Exponent for input curve (1.0 = linear, 2.0 = squared, 3.0 = cubed).
   *
   * <p>This makes the joystick response non-linear, giving finer control
   * at low speeds while still allowing full speed at max stick deflection.
   *
   * <pre>
   *   LINEAR (1.0):              SQUARED (2.0):
   *   output │      ╱            output │        ╱
   *          │    ╱                     │      ╱
   *          │  ╱                       │   ╱╱
   *          │╱                         │ ╱╱
   *          └──────── input            └──────── input
   *
   *   Stick at 50% → 50% speed    Stick at 50% → 25% speed
   *   Stick at 100% → 100% speed  Stick at 100% → 100% speed
   *
   *   Squared gives more control at low speeds!
   * </pre>
   */
  public static final double INPUT_CURVE_EXPONENT = 2.0;

  // ==================== Controller Ports ====================
  // Which USB port each controller is plugged into

  /**
   * USB port for the driver controller.
   *
   * <p>The driver controller handles robot movement and vision targeting.
   * This is typically port 0 (the first controller connected).
   */
  public static final int DRIVER_CONTROLLER_PORT = 0;

  /**
   * USB port for the operator controller.
   *
   * <p>The operator controller handles intake, shooter, feeder, and climb.
   * This is typically port 1 (the second controller connected).
   */
  public static final int OPERATOR_CONTROLLER_PORT = 1;

  // ==================== Vision Lock Settings ====================
  // These control the auto-aim feature while driving

  /**
   * Whether vision lock overrides manual rotation.
   *
   * <p>When true: Vision lock completely controls rotation, right stick ignored
   * <p>When false: Vision lock adds to manual rotation (more advanced)
   *
   * <p>We use true for simpler, more predictable behavior.
   */
  public static final boolean VISION_LOCK_OVERRIDES_ROTATION = true;

  // ==================== Private Constructor ====================

  /**
   * Private constructor to prevent instantiation.
   */
  private DriveConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
