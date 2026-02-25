// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.ClimbConstants;
import frc.robot.utils.TalonFXUtil;

/**
 * Climb — controls the single motor that raises and lowers the robot.
 *
 * <p>The motor runs on the RoboRIO CAN bus and uses Brake mode so the
 * robot holds its position the moment the driver releases the button.
 *
 * <p>Controls (bound in RobotContainer):
 * <ul>
 *   <li>Y held → climb up
 *   <li>A held → climb down
 *   <li>Neither held → motor brakes and holds current position
 * </ul>
 *
 * <p>CAN ID and speeds are set in {@link frc.robot.constants.ClimbConstants}.
 */
@Logged
public class Climb extends SubsystemBase {

  // Climb motor — on the RoboRIO CAN bus (no second constructor argument)
  private final TalonFX motor = new TalonFX(ClimbConstants.CLIMB_MOTOR_ID);

  // Duty cycle output: drives motor at a fixed percentage of available voltage
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public Climb() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Brake mode: motor holds position when no command is running.
    // This is critical for climbing — the robot must not slip down when the driver
    // releases the button.
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    if (TalonFXUtil.applyConfigWithRetries(motor, config, 2)) {
      Robot.telemetry().log("Climb/Config", true);
    } else {
      Robot.telemetry().log("Climb/Config", false);
    }
  }

  @Override
  public void periodic() {
    // No periodic work needed
  }

  // ==================== Internal Motor Control ====================

  /** Run the climb motor upward. */
  private void runUp() {
    motor.setControl(dutyCycleOut.withOutput(ClimbConstants.CLIMB_UP_SPEED));
  }

  /** Run the climb motor downward. */
  private void runDown() {
    motor.setControl(dutyCycleOut.withOutput(-ClimbConstants.CLIMB_DOWN_SPEED));
  }

  /**
   * Stop the climb motor.
   * Brake mode holds the robot at its current height.
   */
  private void stop() {
    motor.stopMotor();
  }

  // ==================== Commands ====================

  /**
   * Climb upward while the command is active.
   *
   * <p>Bind to Y with {@code .whileTrue()} so the robot climbs while Y is held
   * and brakes to a stop the moment the button is released.
   *
   * @return Command that climbs up while active, brakes on release
   */
  public Command climbUpCommand() {
    return startEnd(() -> runUp(), () -> stop()).withName("ClimbUp");
  }

  /**
   * Lower the robot while the command is active.
   *
   * <p>Bind to A with {@code .whileTrue()} so the robot descends while A is held
   * and brakes to a stop the moment the button is released.
   *
   * @return Command that climbs down while active, brakes on release
   */
  public Command climbDownCommand() {
    return startEnd(() -> runDown(), () -> stop()).withName("ClimbDown");
  }
}
