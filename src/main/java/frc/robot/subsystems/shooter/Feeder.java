// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.TalonFXUtil;

/**
 * Feeder — controls the motor that feeds balls up to the shooter.
 *
 * <p>This motor runs slower than the shooter wheels. Its job is to push
 * balls from the hopper/intake area up into the spinning shooter wheels.
 *
 * <p>Uses duty cycle (percent) control since precise speed isn't critical here —
 * the feeder just needs to move balls consistently. The speed is set in
 * {@link frc.robot.constants.ShooterConstants#FEEDER_SPEED_PERCENT}.
 *
 * <p>Brake mode is used so balls don't roll back through the feeder when it stops.
 */
@Logged
public class Feeder extends SubsystemBase {

  // Feeder motor — on the RoboRIO CAN bus (no second constructor argument)
  private final TalonFX motor = new TalonFX(ShooterConstants.FEEDER_ID);

  // Duty cycle output: drives motor at a fixed percentage of available voltage
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public Feeder() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue. Clockwise_Positive;

    // Brake mode: holds ball position when the feeder stops so balls don't drift back
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    if (TalonFXUtil.applyConfigWithRetries(motor, config, 2)) {
      Robot.telemetry().log("Feeder/Config", true);
    } else {
      Robot.telemetry().log("Feeder/Config", false);
    }
  }

  @Override
  public void periodic() {
    // No periodic work needed
  }

  // ==================== Internal Motor Control ====================

  /** Run the feeder at the speed set in ShooterConstants. */
  private void run() {
    motor.setControl(dutyCycleOut.withOutput(ShooterConstants.FEEDER_SPEED_PERCENT));
  }

  /** Stop the feeder motor. Brake mode holds the ball in place. */
  private void stop() {
    motor.stopMotor();
  }

  // ==================== Commands ====================

  /**
   * Run the feeder while the command is active, stop when it ends.
   *
   * <p>Designed for the right trigger in teleop: feeder runs the whole
   * time the trigger is held and stops the moment it is released.
   *
   * @return Command that runs feeder while active, stops on cancel
   */
  public Command feedWhileHeld() {
    return startEnd(() -> run(), () -> stop()).withName("FeederFeedWhileHeld");
  }

  /**
   * Run the feeder for a fixed amount of time, then stop automatically.
   *
   * <p>Useful in autonomous to feed exactly one ball through the shooter.
   *
   * @param seconds How long to run the feeder
   * @return Command that runs feeder for the given time then stops
   */
  public Command feedForTime(double seconds) {
    return startEnd(() -> run(), () -> stop())
        .withTimeout(seconds)
        .withName("FeederFeedForTime");
  }

  /**
   * Reverse the feeder to push balls back out (unclog).
   * Stops on command cancel.
   *
   * @return Command that reverses feeder while active, stops on cancel
   */
  public Command reverseFeedWhileHeld() {
    return startEnd(
            () -> motor.setControl(dutyCycleOut.withOutput(ShooterConstants.FEEDER_REVERSE_PERCENT)),
            () -> stop())
        .withName("FeederReverse");
  }

  /**
   * Stop the feeder once, then release the subsystem.
   *
   * @return Instant command that stops the feeder
   */
  public Command stopCommand() {
    return runOnce(() -> stop()).withName("FeederStop");
  }
}
