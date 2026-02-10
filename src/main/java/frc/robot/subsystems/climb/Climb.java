// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Robot;
import frc.robot.constants.ClimbConstants;
import frc.robot.generated.TunerConstants;
//import frc.robot.utils.TalonFXUtil;

/**
 * Climb subsystem - Robot climbing mechanism.
 *
 * <h2>What is Climbing?</h2>
 * <p>At the end of FRC matches, robots can earn bonus points by climbing onto
 * elevated structures (bars, chains, platforms). Our climb system has:
 *
 * <pre>
 *   CLIMB MECHANISMS:
 *   ─────────────────
 *
 *   1. LIFT / DROP
 *      A winch or elevator that raises/lowers the robot body
 *
 *   2. FLIP UP / FLIP DOWN
 *      Arms that rotate to reach and hook onto climb bars
 *
 *   ═══════════════ ← Climbing bar
 *          │
 *          │  ← Hook/arm grabs bar
 *         ╱
 *      ──╱
 *   [ROBOT]  ↑ Robot lifts itself up
 * </pre>
 *
 * <h2>Operator Controls</h2>
 * <pre>
 *   Y button: Flip Up (reach for bar)
 *   A button: Flip Down (retract arm)
 *   X button: Lift Robot (climb up)
 *   B button: Drop Robot (lower down)
 * </pre>
 *
 * @see frc.robot.constants.ClimbConstants for configuration values
 */
@Logged
public class Climb extends SubsystemBase {

  // ==================== Hardware ====================

  /**
   * Motor for lifting/lowering the robot body.
   *
   * <p>Protected so simulation subclass can access it.
   */
  protected final TalonFX liftMotor =
      new TalonFX(ClimbConstants.LIFT_MOTOR_CAN_ID, TunerConstants.kCANBus);

  /**
   * Motor for flipping the climb arms up/down.
   *
   * <p>Protected so simulation subclass can access it.
   */
  protected final TalonFX flipMotor =
      new TalonFX(ClimbConstants.FLIP_MOTOR_CAN_ID, TunerConstants.kCANBus);

  // ==================== Control Requests ====================

  /**
   * Velocity control request for lift motor.
   */
  private final VelocityVoltage liftVelocityRequest = new VelocityVoltage(0);

  /**
   * Velocity control request for flip motor.
   */
  private final VelocityVoltage flipVelocityRequest = new VelocityVoltage(0);

  // ==================== Configuration ====================

  /** Configuration for the lift motor. */
  protected TalonFXConfiguration liftConfig = new TalonFXConfiguration();

  /** Configuration for the flip motor. */
  protected TalonFXConfiguration flipConfig = new TalonFXConfiguration();

  // ==================== Constructor ====================

  /**
   * Creates a new Climb subsystem.
   *
   * <p>Configures both motors with proper settings.
   */
  public Climb() {
    // ----- Lift Motor Configuration -----

    // Brake mode: Holds position when stopped (important for safety!)
    liftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Motor direction
    liftConfig.MotorOutput.Inverted =
        ClimbConstants.LIFT_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Current limits
    CurrentLimitsConfigs liftCurrentLimits = new CurrentLimitsConfigs();
    liftCurrentLimits.StatorCurrentLimit = ClimbConstants.LIFT_CURRENT_LIMIT_AMPS;
    liftCurrentLimits.StatorCurrentLimitEnable = true;
    liftConfig.CurrentLimits = liftCurrentLimits;

    // Velocity control gains
    liftConfig.Slot0.kS = ClimbConstants.LIFT_kS;
    liftConfig.Slot0.kV = ClimbConstants.LIFT_kV;
    liftConfig.Slot0.kP = ClimbConstants.LIFT_kP;

    // ----- Flip Motor Configuration -----

    // Brake mode: Holds position when stopped
    flipConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Motor direction
    flipConfig.MotorOutput.Inverted =
        ClimbConstants.FLIP_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Current limits
    CurrentLimitsConfigs flipCurrentLimits = new CurrentLimitsConfigs();
    flipCurrentLimits.StatorCurrentLimit = ClimbConstants.FLIP_CURRENT_LIMIT_AMPS;
    flipCurrentLimits.StatorCurrentLimitEnable = true;
    flipConfig.CurrentLimits = flipCurrentLimits;

    // Velocity control gains
    flipConfig.Slot0.kS = ClimbConstants.FLIP_kS;
    flipConfig.Slot0.kV = ClimbConstants.FLIP_kV;
    flipConfig.Slot0.kP = ClimbConstants.FLIP_kP;

    // ----- Apply Configurations -----

    //boolean liftSuccess = TalonFXUtil.applyConfigWithRetries(liftMotor, liftConfig, 3);
    //boolean flipSuccess = TalonFXUtil.applyConfigWithRetries(flipMotor, flipConfig, 3);

    //Robot.telemetry().log("Climb/LiftMotorConfig", liftSuccess);
    //Robot.telemetry().log("Climb/FlipMotorConfig", flipSuccess);

    System.out.println("Climb subsystem initialized");
  }

  // ==================== Periodic ====================

  @Override
  public void periodic() {
    // Add SmartDashboard telemetry
    SmartDashboard.putNumber("Climb/Lift Velocity (RPS)", getLiftVelocity().in(RotationsPerSecond));
    SmartDashboard.putNumber("Climb/Flip Velocity (RPS)", getFlipVelocity().in(RotationsPerSecond));
    SmartDashboard.putNumber("Climb/Lift Current (Amps)", liftMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Climb/Flip Current (Amps)", flipMotor.getStatorCurrent().getValueAsDouble());
    
    // Status indicators
    boolean liftRunning = Math.abs(getLiftVelocity().in(RotationsPerSecond)) > 0.5;
    boolean flipRunning = Math.abs(getFlipVelocity().in(RotationsPerSecond)) > 0.5;
    
    SmartDashboard.putBoolean("Climb/Lift Running", liftRunning);
    SmartDashboard.putBoolean("Climb/Flip Running", flipRunning);
    
    // Overall status
    String status = "IDLE";
    if (liftRunning && flipRunning) {
      status = "LIFT + FLIP ACTIVE";
    } else if (liftRunning) {
      status = "LIFTING";
    } else if (flipRunning) {
      status = "FLIPPING";
    }
    SmartDashboard.putString("Climb/Status", status);
  }

  // ==================== Private Motor Control Methods ====================

  /**
   * Sets the lift motor to a specific velocity.
   *
   * <p>PRIVATE - all control goes through Commands.
   *
   * @param velocity Target velocity (positive = up, negative = down)
   */
  private void setLiftVelocity(AngularVelocity velocity) {
    liftMotor.setControl(liftVelocityRequest.withVelocity(velocity));
  }

  /**
   * Sets the flip motor to a specific velocity.
   *
   * <p>PRIVATE - all control goes through Commands.
   *
   * @param velocity Target velocity (positive = up, negative = down)
   */
  private void setFlipVelocity(AngularVelocity velocity) {
    flipMotor.setControl(flipVelocityRequest.withVelocity(velocity));
  }

  /**
   * Stops both climb motors.
   *
   * <p>PRIVATE - all control goes through Commands.
   */
  private void stopAll() {
    liftMotor.stopMotor();
    flipMotor.stopMotor();
  }

  // ==================== Command Factory Methods ====================

  /**
   * Command to lift the robot (climb up).
   *
   * <p>This runs the lift/winch motor to raise the robot body.
   *
   * @return Command that lifts the robot
   */
  public Command liftRobotCommand() {
    return startEnd(
            () -> setLiftVelocity(RotationsPerSecond.of(ClimbConstants.LIFT_SPEED_RPS)),
            () -> liftMotor.stopMotor())
        .withName("LiftRobot");
  }

  /**
   * Command to drop the robot (lower down).
   *
   * @return Command that lowers the robot
   */
  public Command dropRobotCommand() {
    return startEnd(
            () -> setLiftVelocity(RotationsPerSecond.of(ClimbConstants.DROP_SPEED_RPS)),
            () -> liftMotor.stopMotor())
        .withName("DropRobot");
  }

  /**
   * Command to flip the climb arm up (reach for bar).
   *
   * @return Command that flips arm up
   */
  public Command flipUpCommand() {
    return startEnd(
            () -> setFlipVelocity(RotationsPerSecond.of(ClimbConstants.FLIP_UP_SPEED_RPS)),
            () -> flipMotor.stopMotor())
        .withName("FlipUp");
  }

  /**
   * Command to flip the climb arm down (retract).
   *
   * @return Command that flips arm down
   */
  public Command flipDownCommand() {
    return startEnd(
            () -> setFlipVelocity(RotationsPerSecond.of(ClimbConstants.FLIP_DOWN_SPEED_RPS)),
            () -> flipMotor.stopMotor())
        .withName("FlipDown");
  }

  /**
   * Command to stop all climb motors.
   *
   * @return Command that stops climb
   */
  public Command stopCommand() {
    return runOnce(() -> stopAll()).withName("ClimbStop");
  }

  // ==================== State Query Methods ====================

  /**
   * Gets the current velocity of the lift motor.
   *
   * @return Current lift velocity
   */
  public AngularVelocity getLiftVelocity() {
    return liftMotor.getVelocity().getValue();
  }

  /**
   * Gets the current velocity of the flip motor.
   *
   * @return Current flip velocity
   */
  public AngularVelocity getFlipVelocity() {
    return flipMotor.getVelocity().getValue();
  }
}