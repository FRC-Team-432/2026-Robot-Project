// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.IntakeFoldConstants;
import frc.robot.utils.TalonFXUtil;

@Logged
public class IntakeFold extends SubsystemBase {

  private final TalonFX motor = new TalonFX(IntakeFoldConstants.FOLD_MOTOR_CAN_ID);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  private final StatusSignal<Voltage> motorVoltage = motor.getMotorVoltage();

  @Logged private boolean deploying = false;
  @Logged private double appliedVoltage = 0.0;

  public IntakeFold() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    if (TalonFXUtil.applyConfigWithRetries(motor, config, 2)) {
      Robot.telemetry().log("IntakeFold/Config", true);
    } else {
      Robot.telemetry().log("IntakeFold/Config", false);
    }
  }

  @Override
  public void periodic() {
    motorVoltage.refresh();
    appliedVoltage = motorVoltage.getValueAsDouble();
    SmartDashboard.putBoolean("IntakeFold/Deploying", deploying);
    SmartDashboard.putNumber("IntakeFold/AppliedVoltage", appliedVoltage);
    SmartDashboard.putBoolean("IntakeFold/MotorAlive", motor.isAlive());
  }

  public Command deployCommand() {
    return startEnd(
            () -> {
              deploying = true;
              motor.setControl(dutyCycleOut.withOutput(IntakeFoldConstants.DEPLOY_SPEED));
            },
            () -> {
              deploying = false;
              motor.stopMotor();
            })
            .withTimeout(IntakeFoldConstants.DEPLOY_TIME_SECONDS)
        .withName("IntakeFoldDeploy");
  }

  public Command retractCommand() {
    return startEnd(
            () -> motor.setControl(dutyCycleOut.withOutput(-IntakeFoldConstants.RETRACT_SPEED)),
            () -> motor.stopMotor())
        .withTimeout(IntakeFoldConstants.RETRACT_TIME_SECONDS)
        .withName("IntakeFoldRetract");
  }
}
