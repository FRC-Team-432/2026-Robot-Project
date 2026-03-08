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
 * <p>Three shooting modes are available:
 * <ul>
 *   <li>{@link #spinWhileHeld()} — fixed speed, for teleop trigger binding
 *   <li>{@link #spinAtAreaWhileHeld(DoubleSupplier)} — speed adjusts automatically
 *       based on AprilTag area (closer = slower, farther = faster)
 *   <li>{@link #spinUpForArea(DoubleSupplier)} — instant area-based spinup for auto sequences
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

  // Interpolation table: looks up the right speed for a given tag area
  // Built once at startup from the AREA_SPEED_MAP in ShooterConstants
  private final InterpolatingDoubleTreeMap areaSpeedMap = new InterpolatingDoubleTreeMap();

  public Shooter() {
    // Build the area → speed lookup table from constants
    for (double[] point : ShooterConstants.AREA_SPEED_MAP) {
      areaSpeedMap.put(point[0], point[1]);
    }

    // Configure the leader motor
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Spin freely when disabled
    config.Slot0.kS = ShooterConstants.kS;
    config.Slot0.kV = ShooterConstants.kV;
    config.Slot0.kP = ShooterConstants.kP;
    config.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = ShooterConstants.MOTION_MAGIC_ACCELERATION;

    if (TalonFXUtil.applyConfigWithRetries(leader, config, 2)) {
      Robot.telemetry().log("Shooter/LeaderConfig", true);
    } else {
      Robot.telemetry().log("Shooter/LeaderConfig", false);
    }

    // Configure the follower motor with the same PID/Motion Magic settings as the leader,
    // but inverted so both wheels spin opposite directions and grip the ball from both sides.
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // opposite of leader
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
   * Spin the shooter at a speed based on the current AprilTag area.
   * Larger area (closer) = slower. Smaller area (farther) = faster.
   * Recalculates speed every loop cycle. Stops on command cancel.
   *
   * For teleop use with .whileTrue() binding.
   *
   * @param areaSupplier Supplier for current tag area (0-100 from getTA())
   */
  public Command spinAtAreaWhileHeld(DoubleSupplier areaSupplier) {
    return run(() -> {
            double area = areaSupplier.getAsDouble();
            double speed;
            if (area < 0.01) {
                speed = ShooterConstants.FALLBACK_SPEED_RPS;
            } else {
                speed = areaSpeedMap.get(area);
            }
            speed = MathUtil.clamp(speed,
                ShooterConstants.MIN_SHOOTER_SPEED_RPS,
                ShooterConstants.MAX_SHOOTER_SPEED_RPS);
            setVelocity(RotationsPerSecond.of(speed));
        })
        .finallyDo(() -> stop())
        .withName("ShooterSpinAtArea");
  }

  /**
   * Set shooter speed based on current tag area, then release.
   * The TalonFX holds speed on its own. For use in auto sequences.
   *
   * @param areaSupplier Supplier for current tag area (0-100 from getTA())
   */
  public Command spinUpForArea(DoubleSupplier areaSupplier) {
    return runOnce(() -> {
            double area = areaSupplier.getAsDouble();
            double speed;
            if (area < 0.01) {
                speed = ShooterConstants.FALLBACK_SPEED_RPS;
            } else {
                speed = areaSpeedMap.get(area);
            }
            speed = MathUtil.clamp(speed,
                ShooterConstants.MIN_SHOOTER_SPEED_RPS,
                ShooterConstants.MAX_SHOOTER_SPEED_RPS);
            setVelocity(RotationsPerSecond.of(speed));
        })
        .withName("ShooterSpinUpForArea");
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

  /** Target speed the shooter is trying to reach. */
  public AngularVelocity getTargetVelocity() {
    return velocityOut.getVelocityMeasure();
  }
}
