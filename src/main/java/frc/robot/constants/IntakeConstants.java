// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

public final class IntakeConstants {

  // TODO: Set the real CAN ID once the intake motor is wired and assigned.
  // Check existing IDs: swerve (1-4, 16-23), Pigeon2 (30), Arm (31), ArmEncoder (32), Flywheel (35)
  public static final int INTAKE_CAN_ID = 12;

  /** Duty cycle output (0.0–1.0) for intaking (left trigger held). */
  public static final double INTAKE_SPEED = 1;

  /** Duty cycle output (0.0–1.0) for ejecting (left bumper held). Applied as negative. */
  public static final double EJECT_SPEED = 0.5;

  private IntakeConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
