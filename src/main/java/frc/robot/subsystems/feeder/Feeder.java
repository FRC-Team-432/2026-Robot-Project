// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.FeederConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;

/**
 * Feeder subsystem - Moves balls from hopper to shooter.
 *
 * <h2>What is the Feeder?</h2>
 * <p>The feeder is like a "delivery system" that moves balls from where they're
 * stored (the hopper) into the spinning shooter flywheels. Without the feeder,
 * balls would just sit in the hopper and never get shot!
 *
 * <pre>
 *   ROBOT BALL PATH:
 *   ────────────────
 *
 *        ┌─────────────┐
 *        │   SHOOTER   │  ← Balls launch from here
 *        │   ○○○○○○○   │
 *        └──────▲──────┘
 *               │
 *        ┌──────┴──────┐
 *        │   FEEDER    │  ← THIS SUBSYSTEM
 *        │   ═══════   │     Pushes balls UP into shooter
 *        └──────▲──────┘
 *               │
 *        ┌──────┴──────┐
 *        │   HOPPER    │  ← Balls stored here
 *        │   ● ● ●     │
 *        └──────▲──────┘
 *               │
 *        ┌──────┴──────┐
 *        │   INTAKE    │  ← Balls enter here
 *        └─────────────┘
 * </pre>
 *
 * <h2>How It Works</h2>
 * <p>The feeder uses a simple roller or belt mechanism driven by a single motor.
 * When the motor runs:
 * <ul>
 *   <li><b>Forward:</b> Balls move UP toward the shooter</li>
 *   <li><b>Reverse:</b> Balls move DOWN away from shooter (to clear jams)</li>
 *   <li><b>Stopped:</b> Balls stay in place</li>
 * </ul>
 *
 * <h2>Manual Control</h2>
 * <p>The OPERATOR controls the feeder with the RB button:
 *
 * <pre>
 *   SHOOTING WORKFLOW:
 *   ──────────────────
 *
 *   1. OPERATOR holds RT → Shooter spins up
 *      (Wait ~1 second for full speed)
 *
 *   2. OPERATOR presses RB → Feeder runs
 *      Ball gets pushed into spinning flywheels
 *      BALL SHOOTS!
 *
 *   3. Release RB → Feeder stops
 *      (Ready for next ball)
 *
 *   4. Repeat for more balls
 *
 *   5. Release RT when done → Shooter spins down
 * </pre>
 *
 * <h2>Important: Timing Matters!</h2>
 * <p>Always spin up the shooter BEFORE running the feeder! If you feed
 * balls before the shooter is at speed, the shots will be weak and
 * inaccurate. The shooter's {@code isAtSpeed()} method tells you when
 * it's safe to feed.
 *
 * @see FeederConstants for configuration values
 * @see frc.robot.subsystems.shooter.Shooter for the shooter that receives balls
 */
@Logged
public class Feeder extends SubsystemBase {

  // ==================== Hardware ====================

  /**
   * Motor that drives the feeder mechanism.
   *
   * <p>Protected so simulation subclass can access for physics simulation.
   */
  protected final TalonFX motor =
      new TalonFX(FeederConstants.MOTOR_CAN_ID, TunerConstants.kCANBus);

  // ==================== Control Requests ====================

  /**
   * Velocity control request for the feeder motor.
   */
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  // ==================== Configuration ====================

  /** Motor configuration. */
  protected TalonFXConfiguration config = new TalonFXConfiguration();

  // ==================== Constructor ====================

  /**
   * Creates a new Feeder subsystem.
   *
   * <p>Configures the motor with:
   * <ul>
   *   <li>Brake mode (stops quickly when released)</li>
   *   <li>Correct direction (positive = toward shooter)</li>
   *   <li>Current limits for protection</li>
   *   <li>Velocity control gains</li>
   * </ul>
   */
  public Feeder() {
    // Brake mode: Feeder stops quickly when released
    // This prevents extra balls from accidentally feeding
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Motor direction
    config.MotorOutput.Inverted =
        FeederConstants.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Current limits
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimit = FeederConstants.CURRENT_LIMIT_AMPS;
    currentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    // Velocity control gains
    config.Slot0.kS = FeederConstants.kS;
    config.Slot0.kV = FeederConstants.kV;
    config.Slot0.kP = FeederConstants.kP;

    // Apply configuration
    if (TalonFXUtil.applyConfigWithRetries(motor, config, 3)) {
      Robot.telemetry().log("Feeder/Config", true);
    } else {
      Robot.telemetry().log("Feeder/Config", false);
    }
  }

  // ==================== Periodic ====================

  @Override
  public void periodic() {
    // Velocity control handled by motor controller
    // @Logged handles telemetry
  }

  // ==================== Private Motor Control ====================

  /**
   * Sets the feeder to run at a specific velocity.
   *
   * @param velocity Target velocity (positive = toward shooter)
   */
  private void setVelocity(AngularVelocity velocity) {
    motor.setControl(velocityRequest.withVelocity(velocity));
  }

  /**
   * Stops the feeder motor.
   */
  private void stop() {
    motor.stopMotor();
  }

  // ==================== Command Factory Methods ====================

  /**
   * Command to run the feeder (move balls toward shooter).
   *
   * <p>Use this to feed balls into the spinning shooter flywheels.
   *
   * <h3>Usage:</h3>
   * <pre>
   * // Operator holds RB to feed balls
   * operatorController.rightBumper().whileTrue(feeder.feedCommand());
   * </pre>
   *
   * <p><b>Important:</b> Make sure the shooter is spun up before feeding!
   *
   * @return Command that runs the feeder forward
   */
  public Command feedCommand() {
    return startEnd(
            () -> setVelocity(RotationsPerSecond.of(FeederConstants.FEED_SPEED_RPS)),
            () -> stop())
        .withName("Feed");
  }

  /**
   * Command to run the feeder in reverse (move balls away from shooter).
   *
   * <p>Use this to:
   * <ul>
   *   <li>Clear jams</li>
   *   <li>Move balls back into the hopper</li>
   *   <li>Prepare for a different shot</li>
   * </ul>
   *
   * @return Command that runs the feeder in reverse
   */
  public Command reverseCommand() {
    return startEnd(
            () -> setVelocity(RotationsPerSecond.of(FeederConstants.REVERSE_SPEED_RPS)),
            () -> stop())
        .withName("FeederReverse");
  }

  /**
   * Command to stop the feeder immediately.
   *
   * @return Command that stops the feeder (runs once)
   */
  public Command stopCommand() {
    return runOnce(() -> stop()).withName("FeederStop");
  }

  // ==================== State Query Methods ====================

  /**
   * Gets the current velocity of the feeder.
   *
   * @return Current velocity
   */
  public AngularVelocity getVelocity() {
    return motor.getVelocity().getValue();
  }

  /**
   * Checks if the feeder is currently running.
   *
   * @return true if the feeder motor is moving
   */
  public boolean isRunning() {
    return Math.abs(motor.getVelocity().getValue().in(RotationsPerSecond)) > 0.5;
  }
}
