// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

public final class IntakeFoldConstants {

  /** CAN ID for the intake fold motor (default CAN bus). */
  public static final int FOLD_MOTOR_CAN_ID = 13;

  /** Duty cycle (0.0–1.0) to deploy the intake (fold down/out). */
  public static final double DEPLOY_SPEED = 0.2;

  /** Duty cycle (0.0–1.0) to retract the intake (fold back up). Applied as negative. */
  public static final double RETRACT_SPEED = 0.2;

  /** How long (seconds) the motor runs to fully deploy (left trigger timeout). */
  public static final double DEPLOY_TIME_SECONDS = 1.0;

  /** How long (seconds) the motor runs in reverse to fully retract (left trigger release timeout). */
  public static final double RETRACT_TIME_SECONDS = 1.0;

  private IntakeFoldConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
