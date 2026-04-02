// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.TalonFXUtil;
import java.util.function.DoubleSupplier;

/**
 * Shooter — controls the two flywheel motors that launch game pieces.
 *
 * <p>The two motors face each other so the ball passes between them.
 * The leader spins one direction; the follower spins the opposite
 * direction so both wheels grip and push the ball out the same side.
 *
 * <p>Shooting modes available:
 * <ul>
 *   <li>{@link #spinWhileHeld()} — fixed speed, for teleop trigger binding
 *   <li>{@link #spinAtDistanceWhileHeld(DoubleSupplier)} — speed adjusts continuously
 *       based on AprilTag distance (closer = slower, farther = faster)
 *   <li>{@link #spinUpForDistance(DoubleSupplier)} — instant distance-based spinup for auto sequences
 * </ul>
 *
 * <p>Both motors stop automatically when their command ends (trigger released or auto step done).
 *
 * <p>CAN IDs and speed values are set in {@link frc.robot.constants.ShooterConstants}.
 */
@Logged
public class Shooter extends SubsystemBase {

  // Leader motor — sets the target speed
  // Both motors are on the RoboRIO CAN bus (no second constructor argument)
  private final TalonFX leader = new TalonFX(ShooterConstants.SHOOTER_LEADER_ID);

  // Second shooter motor — runs independently at the same speed but physically inverted.
  // InvertedValue.Clockwise_Positive is set in its config so both wheels
  // spin opposite directions and grip the ball from both sides.
  private final TalonFX follower = new TalonFX(ShooterConstants.SHOOTER_FOLLOWER_ID);

  // Controller that tells the leader motor what speed to reach
  private final MotionMagicVelocityVoltage velocityOut = new MotionMagicVelocityVoltage(0);
  // Separate control request for the follower — never share a MotionMagicVelocityVoltage instance
  // between two motors; Phoenix 6 uses per-instance bookkeeping for trajectory state
  private final MotionMagicVelocityVoltage followerVelocityOut = new MotionMagicVelocityVoltage(0);

  // How close the speed needs to be before we consider the shooter "ready"
  private final AngularVelocity tolerance =
      RotationsPerSecond.of(ShooterConstants.VELOCITY_TOLERANCE_RPS);

  private double triggerValue = 0.0;
  private double commandedSpeedRPS = 0.0;

  // Last valid distance from Limelight — held so that brief tag loss (vibration, rotation)
  // doesn't cause the shooter to jump to fallback speed mid-shot.
  // Reset to 0.0 when the shooting command ends.
  private double lastKnownDistance = 0.0;

  // Throttle logging to ~5Hz (every 4 cycles at 20ms periodic) to avoid flooding RioLog
  private int distanceLogCounter = 0;
  private static final int DISTANCE_LOG_INTERVAL = 4;

  // Interpolation tables: auto uses lower speeds (stationary), teleop uses higher (moving)
  private final InterpolatingDoubleTreeMap autoDistanceSpeedMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap teleOpDistanceSpeedMap = new InterpolatingDoubleTreeMap();

  public Shooter() {
    // Build the distance → speed lookup tables from constants
    for (double[] point : ShooterConstants.AUTO_DISTANCE_SPEED_MAP) {
      autoDistanceSpeedMap.put(point[0], point[1]);
    }
    for (double[] point : ShooterConstants.TELEOP_DISTANCE_SPEED_MAP) {
      teleOpDistanceSpeedMap.put(point[0], point[1]);
    }

    // Configure the leader motor
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Spin freely when disabled
    config.Slot0.kS = ShooterConstants.kS;
    config.Slot0.kV = ShooterConstants.kV;
    config.Slot0.kP = ShooterConstants.kP;
    config.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = ShooterConstants.MOTION_MAGIC_ACCELERATION;
    config.MotorOutput.Inverted = InvertedValue. Clockwise_Positive;

    if (TalonFXUtil.applyConfigWithRetries(leader, config, 2)) {
      Robot.telemetry().log("Shooter/LeaderConfig", true);
    } else {
      Robot.telemetry().log("Shooter/LeaderConfig", false);
    }

    // Configure the follower motor with the same PID/Motion Magic settings as the leader,
    // but inverted so both wheels spin opposite directions and grip the ball from both sides.
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // opposite of leader
    followerConfig.Slot0.kS = ShooterConstants.kS;
    followerConfig.Slot0.kV = ShooterConstants.kV;
    followerConfig.Slot0.kP = ShooterConstants.kP;
    followerConfig.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    followerConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.MOTION_MAGIC_ACCELERATION;

    if (TalonFXUtil.applyConfigWithRetries(follower, followerConfig, 2)) {
      Robot.telemetry().log("Shooter/FollowerConfig", true);
    } else {
      Robot.telemetry().log("Shooter/FollowerConfig", false);
    }
  }

  @Override
  public void periodic() {
    // No periodic work needed — TalonFX maintains its last control request automatically
  }

  // ==================== Internal Motor Control ====================

  /**
   * Spin both shooter wheels at a fixed speed.
   * Follower automatically mirrors the leader in the opposite direction.
   *
   * @param velocity Target speed in rotations per second
   */
  private void setVelocity(AngularVelocity velocity) {
    // Both motors get the same velocity command. The follower's InvertedValue
    // in its config makes it spin the opposite physical direction automatically.
    leader.setControl(velocityOut.withVelocity(velocity));
    follower.setControl(followerVelocityOut.withVelocity(velocity));
  }

  /** Stop both shooter motors by commanding velocity=0 so velocityOut stays in sync. */
  private void stop() {
    leader.setControl(velocityOut.withVelocity(RotationsPerSecond.of(0)));
    follower.setControl(followerVelocityOut.withVelocity(RotationsPerSecond.of(0)));
  }

  // ==================== Commands ====================

  /**
   * Spin the shooter at the fixed speed from ShooterConstants.
   *
   * <p>Designed for the right trigger in teleop: the motors spin the whole
   * time the trigger is held and stop the moment it is released.
   *
   * @return Command that runs shooter while active, stops on cancel
   */
  public Command spinWhileHeld() {
    return run(() -> setVelocity(RotationsPerSecond.of(ShooterConstants.SHOOTER_SPEED_RPS)))
        .finallyDo(() -> stop())
        .withName("ShooterSpinWhileHeld");
  }

  /**
   * Spin the shooter at a speed proportional to how far the trigger is pressed.
   * The full trigger range (0.0–1.0) maps smoothly to 0–SHOOTER_SPEED_RPS
   * with no minimum floor, so every increment of the trigger gives a
   * proportional increase in speed.
   *
   * <p>Logs trigger value and commanded speed to the Driver Station console.
   *
   * @param triggerSupplier Supplier for trigger axis (0.0–1.0)
   */
  public Command spinAtTriggerWhileHeld(DoubleSupplier triggerSupplier) {
    return run(() -> {
            double trigger = MathUtil.clamp(triggerSupplier.getAsDouble(), 0.0, 1.0);
            double speed = trigger * ShooterConstants.SHOOTER_SPEED_RPS;
            speed = Math.min(speed, ShooterConstants.MAX_SHOOTER_SPEED_RPS);

            triggerValue = trigger;
            commandedSpeedRPS = speed;

            System.out.println(
                String.format("Shooter | trigger: %.0f%% | cmd: %.1f RPS | actual: %.1f RPS",
                    trigger * 100, speed, getVelocity().in(RotationsPerSecond)));

            setVelocity(RotationsPerSecond.of(speed));
        })
        .finallyDo(() -> {
            stop();
            triggerValue = 0.0;
            commandedSpeedRPS = 0.0;
        })
        .withName("ShooterSpinAtTrigger");
  }

  /**
   * TELEOP: Spin the shooter using the teleop speed map (higher speeds for moving robot).
   * Recalculates EVERY loop cycle (20ms). Holds last known distance on brief tag loss.
   *
   * @param distanceSupplier Supplier for current distance to target (meters)
   */
  public Command spinAtTeleOpDistanceWhileHeld(DoubleSupplier distanceSupplier) {
    return run(() -> {
            double distance = distanceSupplier.getAsDouble();
            double speed;
            String source;

            if (distance > 0) {
                lastKnownDistance = distance;
                speed = teleOpDistanceSpeedMap.get(distance);
                source = "LIVE";
            } else if (lastKnownDistance > 0) {
                distance = lastKnownDistance;
                speed = teleOpDistanceSpeedMap.get(lastKnownDistance);
                source = "HOLD";
            } else {
                speed = ShooterConstants.TELEOP_FALLBACK_SPEED_RPS;
                source = "FALLBACK";
            }

            speed = MathUtil.clamp(speed,
                ShooterConstants.MIN_SHOOTER_SPEED_RPS,
                ShooterConstants.MAX_SHOOTER_SPEED_RPS);

            commandedSpeedRPS = speed;

            if (distanceLogCounter++ % DISTANCE_LOG_INTERVAL == 0) {
                System.out.println(
                    String.format("Shooter TELEOP [%s] | dist: %.2f m | cmd: %.1f RPS | actual: %.1f RPS",
                        source, distance, speed, getVelocity().in(RotationsPerSecond)));
            }

            setVelocity(RotationsPerSecond.of(speed));
        })
        .finallyDo(() -> {
            stop();
            commandedSpeedRPS = 0.0;
            lastKnownDistance = 0.0;
            distanceLogCounter = 0;
        })
        .withName("ShooterTeleOpSpinAtDistance");
  }

  /**
   * AUTO: Spin the shooter using the auto speed map (lower speeds for stationary robot).
   * Recalculates EVERY loop cycle (20ms). Holds last known distance on brief tag loss.
   *
   * @param distanceSupplier Supplier for current distance to target (meters)
   */
  public Command spinAtAutoDistanceWhileHeld(DoubleSupplier distanceSupplier) {
    return run(() -> {
            double distance = distanceSupplier.getAsDouble();
            double speed;
            String source;

            if (distance > 0) {
                lastKnownDistance = distance;
                speed = autoDistanceSpeedMap.get(distance);
                source = "LIVE";
            } else if (lastKnownDistance > 0) {
                distance = lastKnownDistance;
                speed = autoDistanceSpeedMap.get(lastKnownDistance);
                source = "HOLD";
            } else {
                speed = ShooterConstants.AUTO_FALLBACK_SPEED_RPS;
                source = "FALLBACK";
            }

            speed = MathUtil.clamp(speed,
                ShooterConstants.MIN_SHOOTER_SPEED_RPS,
                ShooterConstants.MAX_SHOOTER_SPEED_RPS);

            commandedSpeedRPS = speed;

            if (distanceLogCounter++ % DISTANCE_LOG_INTERVAL == 0) {
                System.out.println(
                    String.format("Shooter AUTO [%s] | dist: %.2f m | cmd: %.1f RPS | actual: %.1f RPS",
                        source, distance, speed, getVelocity().in(RotationsPerSecond)));
            }

            setVelocity(RotationsPerSecond.of(speed));
        })
        .finallyDo(() -> {
            stop();
            commandedSpeedRPS = 0.0;
            lastKnownDistance = 0.0;
            distanceLogCounter = 0;
        })
        .withName("ShooterAutoSpinAtDistance");
  }

  /**
   * AUTO: Set shooter speed based on current tag distance (instant, then release).
   * The TalonFX holds speed on its own. Uses the auto speed map.
   *
   * @param distanceSupplier Supplier for current distance to target (meters)
   */
  public Command spinUpForDistance(DoubleSupplier distanceSupplier) {
    return runOnce(() -> {
            double distance = distanceSupplier.getAsDouble();
            double speed;
            String source;

            if (distance > 0) {
                lastKnownDistance = distance;
                speed = autoDistanceSpeedMap.get(distance);
                source = "LIVE";
            } else if (lastKnownDistance > 0) {
                distance = lastKnownDistance;
                speed = autoDistanceSpeedMap.get(lastKnownDistance);
                source = "HOLD";
            } else {
                speed = ShooterConstants.AUTO_FALLBACK_SPEED_RPS;
                source = "FALLBACK";
            }

            speed = MathUtil.clamp(speed,
                ShooterConstants.MIN_SHOOTER_SPEED_RPS,
                ShooterConstants.MAX_SHOOTER_SPEED_RPS);

            System.out.println(
                String.format("Shooter Auto [%s] | dist: %.2f m | speed: %.1f RPS",
                    source, distance, speed));

            setVelocity(RotationsPerSecond.of(speed));
        })
        .withName("ShooterSpinUpForDistance");
  }

  /**
   * Set the shooter to its target speed once, then release the subsystem.
   *
   * <p>The TalonFX holds this speed on its own until told otherwise,
   * so this is safe to use in auto sequences where the command needs to finish
   * before moving on (e.g., "start spinning up, then wait until at speed").
   *
   * @return Instant command that starts the shooter
   */
  public Command spinUpOnce() {
    return runOnce(() -> setVelocity(RotationsPerSecond.of(ShooterConstants.SHOOTER_SPEED_RPS)))
        .withName("ShooterSpinUpOnce");
  }

  /**
   * Reverse the shooter wheels to unclog jammed balls.
   * Stops on command cancel.
   *
   * @return Command that reverses shooter while active, stops on cancel
   */
  public Command reverseWhileHeld() {
    return run(() -> setVelocity(RotationsPerSecond.of(-ShooterConstants.REVERSE_SPEED_RPS)))
        .finallyDo(() -> stop())
        .withName("ShooterReverse");
  }

  /**
   * Stop the shooter motors once, then release the subsystem.
   *
   * @return Instant command that stops the shooter
   */
  public Command stopCommand() {
    return runOnce(() -> stop()).withName("ShooterStop");
  }

  // ==================== State Checks ====================

  /**
   * Check whether the shooter has reached its target speed and is ready to fire.
   *
   * @return true if within tolerance of target speed
   */
  public boolean isAtTarget() {
    return getVelocity().isNear(getTargetVelocity(), tolerance);
  }

  /** Current shooter speed (from the leader motor encoder). */
  public AngularVelocity getVelocity() {
    return leader.getVelocity().getValue();
  }

  /** Current shooter speed in RPS (convenience for threshold checks). */
  public double getSpeedRPS() {
    return getVelocity().in(RotationsPerSecond);
  }

  /** Target speed the shooter is trying to reach. */
  public AngularVelocity getTargetVelocity() {
    return velocityOut.getVelocityMeasure();
  }
}
