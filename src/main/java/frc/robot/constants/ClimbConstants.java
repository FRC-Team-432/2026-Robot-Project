// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Settings for the Climb subsystem.
 *
 * <p>All values here are adjustable placeholders — tune them once the robot is built.
 *
 * <ul>
 *   <li>CLIMB_MOTOR_ID — set to the actual CAN ID once assigned
 *   <li>CLIMB_UP_SPEED — duty cycle for climbing up (0.0 to 1.0)
 *   <li>CLIMB_DOWN_SPEED — duty cycle for climbing down (0.0 to 1.0, applied as negative)
 * </ul>
 */
public final class ClimbConstants {

  // ==================== CAN ID ====================

  /**
   * CAN ID for the climb motor.
   * TODO: Set this to the correct motor controller ID once assigned.
   * This motor is on the RoboRIO CAN bus (not CANivore).
   */
  public static final int CLIMB_MOTOR_ID = 7;

  // ==================== Speeds ====================

  /**
   * Duty cycle for climbing up (0.0 = stopped, 1.0 = full speed).
   * TODO: Tune on real robot — start low and increase as needed.
   */
  public static final double CLIMB_UP_SPEED = 0.5;

  /**
   * Duty cycle for climbing down (0.0 = stopped, 1.0 = full speed).
   * Applied as a negative value internally so the motor runs in reverse.
   * TODO: Tune on real robot — may differ from up speed due to gravity assist.
   */
  public static final double CLIMB_DOWN_SPEED = 0.3;

  private ClimbConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
