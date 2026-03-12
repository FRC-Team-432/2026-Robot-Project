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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.IntakeConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;

@Logged
public class Intake extends SubsystemBase {

  private final TalonFX motor = new TalonFX(IntakeConstants.INTAKE_CAN_ID, TunerConstants.kCANBus);

  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private final StatusSignal<Current> statorCurrent = motor.getStatorCurrent();

  @Logged private double currentAmps;
  @Logged private boolean isStalled;

  public Intake() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    // Brake mode: stops quickly when command ends so game pieces don't slip out
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    if (TalonFXUtil.applyConfigWithRetries(motor, config, 2)) {
      Robot.telemetry().log("Intake/Config", true);
    } else {
      Robot.telemetry().log("Intake/Config", false);
    }
  }

  /** Spin the intake motor forward to collect a game piece. Stops immediately if current hits 40A. */
  public Command intake() {
    return runEnd(
            () -> {
              statorCurrent.refresh();
              currentAmps = statorCurrent.getValueAsDouble();

              if (currentAmps >= IntakeConstants.CURRENT_LIMIT_AMPS) {
                isStalled = true;
                motor.stopMotor();
              } else if (!isStalled) {
                motor.setControl(dutyCycleOut.withOutput(IntakeConstants.INTAKE_SPEED));
              }
            },
            () -> {
              motor.stopMotor();
              isStalled = false;
            })
        .withName("Intake");
  }

  /** Spin the intake motor in reverse to eject a game piece. Runs until interrupted. */
  public Command eject() {
    return runEnd(
            () -> motor.setControl(dutyCycleOut.withOutput(-IntakeConstants.EJECT_SPEED)),
            motor::stopMotor)
        .withName("Eject");
  }
}
