// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.IntakeConstants;
import frc.robot.utils.TalonFXUtil;

/**
 * ArmRoller — wheels on the end of the foldable arm that act as an extension of the intake.
 *
 * <p>Runs in the opposite direction of the main intake motor so game pieces are
 * fed consistently from the arm into the robot. Activated at the same time as the
 * intake via the left trigger.
 */
@Logged
public class ArmRoller extends SubsystemBase {

  private final TalonFX motor = new TalonFX(IntakeConstants.ARM_ROLLER_CAN_ID);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public ArmRoller() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    if (TalonFXUtil.applyConfigWithRetries(motor, config, 2)) {
      Robot.telemetry().log("ArmRoller/Config", true);
    } else {
      Robot.telemetry().log("ArmRoller/Config", false);
    }
  }

  /**
   * Spin the arm roller in the opposite direction of the intake to feed game pieces inward.
   * Stops when the command ends (trigger released).
   *
   * @return Command that runs the arm roller while active, stops on cancel
   */
  public Command rollInward() {
    return runEnd(
            () -> motor.setControl(dutyCycleOut.withOutput(-IntakeConstants.ARM_ROLLER_SPEED)),
            motor::stopMotor)
        .withName("ArmRollerInward");
  }

  /**
   * Spin the arm roller in the forward direction to eject game pieces outward.
   * Stops when the command ends.
   *
   * @return Command that runs the arm roller in reverse while active, stops on cancel
   */
  public Command rollOutward() {
    return runEnd(
            () -> motor.setControl(dutyCycleOut.withOutput(IntakeConstants.ARM_ROLLER_SPEED)),
            motor::stopMotor)
        .withName("ArmRollerOutward");
  }
}
