// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

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
import frc.robot.constants.ShooterConstants;
import frc.robot.generated.TunerConstants;

/**
 * Shooter subsystem - Dual flywheel system for launching balls.
 *
 * <h2>What is a Flywheel Shooter?</h2>
 * <p>A flywheel shooter uses spinning wheels to launch game pieces. When the
 * wheels spin fast enough and a ball is fed into them, the ball gets gripped
 * by the wheels and accelerated outward at high speed.
 *
 * <p>Our shooter uses TWO flywheels (dual flywheel design):
 *
 * <pre>
 *   DUAL FLYWHEEL SHOOTER
 *   ─────────────────────
 *
 *        ○○○○○○○○○○  ← TOP WHEEL (topMotor)
 *        ══════════     Spins INWARD (toward the ball)
 *           ball  →  →  →  LAUNCHES!
 *        ══════════
 *        ○○○○○○○○○○  ← BOTTOM WHEEL (bottomMotor)
 *                       Spins INWARD (toward the ball)
 *
 *   Both wheels spin inward to grip the ball from above and below.
 *   The ball gets squeezed between them and accelerated out!
 * </pre>
 *
 * <h2>Why Use Two Motors?</h2>
 * <p>Having two motors (one per wheel) gives us:
 * <ul>
 *   <li><b>More power:</b> Two motors = double the acceleration</li>
 *   <li><b>Better grip:</b> Ball is grabbed from both sides</li>
 *   <li><b>Spin control:</b> Could run at different speeds for ball spin (future)</li>
 *   <li><b>Redundancy:</b> If one motor fails, the other might still work</li>
 * </ul>
 *
 * <h2>Velocity Control</h2>
 * <p>We use <b>velocity control</b> to maintain consistent wheel speed:
 * <ul>
 *   <li>We tell the motors "spin at 60 rotations per second"</li>
 *   <li>The motors automatically adjust power to maintain that speed</li>
 *   <li>Even as the battery drains, shots stay consistent</li>
 *   <li>We can check if wheels are "at speed" before shooting</li>
 * </ul>
 *
 * <h2>Spin-Up Time</h2>
 * <p>The flywheels take time to reach full speed (about 1 second). This is
 * because the wheels have mass (inertia) and need time to accelerate.
 *
 * <pre>
 *   SPIN-UP GRAPH:
 *
 *   Speed │         ┌────────────── Target speed (60 RPS)
 *   (RPS) │       ╱│
 *      60 │     ╱  │
 *         │   ╱    │
 *         │ ╱      │
 *       0 │╱───────┴─────────
 *         0    1    2    3   Time (seconds)
 *              ↑
 *         At speed!
 *         Safe to feed balls
 * </pre>
 *
 * <h2>Using the Shooter</h2>
 * <p>The shooter is controlled by the OPERATOR (not the driver):
 *
 * <pre>
 *   SHOOTING SEQUENCE:
 *   ──────────────────
 *
 *   1. HOLD RT → spinUpCommand() runs
 *      Wheels start spinning, wait ~1 second
 *
 *   2. Check if ready:
 *      isAtSpeed() returns true when wheels are fast enough
 *
 *   3. PRESS RB → (feeder runs, not this subsystem)
 *      Ball enters shooter and launches!
 *
 *   4. Keep holding RT for more shots
 *
 *   5. RELEASE RT → stopCommand() runs
 *      Wheels spin down
 * </pre>
 *
 * @see ShooterConstants for configuration values
 * @see frc.robot.subsystems.feeder.Feeder for feeding balls to the shooter
 */
@Logged
public class Shooter extends SubsystemBase {

  // ==================== Hardware ====================
  // The motors that spin the flywheel wheels

  /**
   * Motor for the TOP flywheel.
   *
   * <p>Protected so simulation subclass can access for physics simulation.
   */
  protected final TalonFX topMotor =
      new TalonFX(ShooterConstants.TOP_MOTOR_CAN_ID, TunerConstants.kCANBus);

  /**
   * Motor for the BOTTOM flywheel.
   *
   * <p>Protected so simulation subclass can access for physics simulation.
   */
  protected final TalonFX bottomMotor =
      new TalonFX(ShooterConstants.BOTTOM_MOTOR_CAN_ID, TunerConstants.kCANBus);

  // ==================== Control Requests ====================

  /**
   * Velocity control request for the top motor.
   *
   * <p>We tell the motor what speed we want, and it figures out the voltage needed.
   */
  private final VelocityVoltage topVelocityRequest = new VelocityVoltage(0);

  /**
   * Velocity control request for the bottom motor.
   */
  private final VelocityVoltage bottomVelocityRequest = new VelocityVoltage(0);

  // ==================== State ====================

  /**
   * Target velocity we're trying to reach.
   *
   * <p>Stored so we can check if we're "at speed" relative to this target.
   */
  private double targetVelocityRPS = 0.0;

  // ==================== Configuration ====================

  /** Configuration for the top motor. */
  protected TalonFXConfiguration topConfig = new TalonFXConfiguration();

  /** Configuration for the bottom motor. */
  protected TalonFXConfiguration bottomConfig = new TalonFXConfiguration();

  // ==================== Constructor ====================

  /**
   * Creates a new Shooter subsystem.
   *
   * <p>This sets up both flywheel motors with:
   * <ul>
   *   <li>Coast mode (wheels spin freely when stopped - safer for flywheels)</li>
   *   <li>Correct motor direction (both spin inward)</li>
   *   <li>Current limits (protects motors and battery)</li>
   *   <li>Velocity control PID gains</li>
   * </ul>
   */
  public Shooter() {
    // ----- Top Motor Configuration -----
    configureMotor(topConfig, ShooterConstants.TOP_MOTOR_INVERTED);

    // ----- Bottom Motor Configuration -----
    configureMotor(bottomConfig, ShooterConstants.BOTTOM_MOTOR_INVERTED);

    // ----- Apply Configurations -----

  
  }

  /**
   * Configures a shooter motor with standard settings.
   *
   * <p>Both motors use the same settings except for inversion.
   *
   * @param config The configuration object to modify
   * @param inverted Whether this motor should be inverted
   */
  private void configureMotor(TalonFXConfiguration config, boolean inverted) {
    // Coast mode: Wheels spin freely when no power applied
    // This is safer for flywheels - brake mode would stop them abruptly
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Set motor direction
    config.MotorOutput.Inverted =
        inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Current limits to protect motor and prevent brownouts
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimit = ShooterConstants.CURRENT_LIMIT_AMPS;
    currentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    // Velocity control PID gains
    config.Slot0.kS = ShooterConstants.kS;
    config.Slot0.kV = ShooterConstants.kV;
    config.Slot0.kP = ShooterConstants.kP;
    config.Slot0.kD = ShooterConstants.kD;
  }

  // ==================== Periodic ====================

  /**
   * Called every robot loop (~20ms).
   *
   * <p>For the shooter, velocity control is handled by the motor controllers,
   * so we don't need to do anything here. The @Logged annotation automatically
   * logs our state to SmartDashboard.
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Top Velocity (RPS)", getTopVelocityRPS());
    SmartDashboard.putNumber("Shooter/Bottom Velocity (RPS)", getBottomVelocityRPS());
    SmartDashboard.putNumber("Shooter/Target Velocity (RPS)", getTargetVelocityRPS());
    SmartDashboard.putBoolean("Shooter/At Speed", isAtSpeed());
    SmartDashboard.putNumber("Shooter/Top Current (Amps)", topMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Bottom Current (Amps)", bottomMotor.getStatorCurrent().getValueAsDouble());
    // Motor controllers handle velocity control internally
    // @Logged handles telemetry
  }

  // ==================== Private Motor Control ====================

  /**
   * Sets both flywheels to spin at a specific velocity.
   *
   * <p>PRIVATE - all control should go through Commands.
   *
   * @param velocityRPS Target velocity in rotations per second
   */
  private void setVelocity(double velocityRPS) {
    targetVelocityRPS = velocityRPS;
    AngularVelocity velocity = RotationsPerSecond.of(velocityRPS);
    topMotor.setControl(topVelocityRequest.withVelocity(velocity));
    bottomMotor.setControl(bottomVelocityRequest.withVelocity(velocity));
  }

  /**
   * Stops both flywheel motors.
   *
   * <p>PRIVATE - all control should go through Commands.
   */
  private void stop() {
    targetVelocityRPS = 0.0;
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  // ==================== Command Factory Methods ====================

  /**
   * Command to spin up the shooter to shooting speed.
   *
   * <p>This command:
   * <ul>
   *   <li>Starts spinning both flywheels when scheduled</li>
   *   <li>Keeps running until cancelled (never finishes on its own)</li>
   *   <li>Stops the flywheels when it ends</li>
   * </ul>
   *
   * <h3>Usage:</h3>
   * <pre>
   * // Operator holds RT to spin up shooter
   * operatorController.rightTrigger().whileTrue(shooter.spinUpCommand());
   * </pre>
   *
   * <p>The operator should hold this while shooting, then release when done.
   *
   * @return Command that spins up the shooter
   */
  public Command spinUpCommand() {
    return startEnd(
            () -> setVelocity(ShooterConstants.SHOOTING_SPEED_RPS),
            () -> stop())
        .withName("ShooterSpinUp");
  }

  /**
   * Command to stop the shooter immediately.
   *
   * <p>Use this for emergency stops or explicit shutdown.
   * Normally, just releasing the spin-up button will stop the shooter.
   *
   * @return Command that stops the shooter (runs once)
   */
  public Command stopCommand() {
    return runOnce(() -> stop()).withName("ShooterStop");
  }

  // ==================== State Query Methods ====================

  /**
   * Checks if the shooter flywheels are at shooting speed.
   *
   * <p>Use this to know when it's safe to feed balls to the shooter.
   * Feeding before the wheels are at speed results in weak shots!
   *
   * <h3>Usage:</h3>
   * <pre>
   * if (shooter.isAtSpeed()) {
   *   // Safe to feed balls!
   * } else {
   *   // Wait for spin-up...
   * }
   * </pre>
   *
   * @return true if both wheels are within tolerance of target speed
   */
  public boolean isAtSpeed() {
    if (targetVelocityRPS <= 0) {
      return false; // Not trying to spin = not "at speed"
    }

    double topSpeed = Math.abs(topMotor.getVelocity().getValue().in(RotationsPerSecond));
    double bottomSpeed = Math.abs(bottomMotor.getVelocity().getValue().in(RotationsPerSecond));
    double target = Math.abs(targetVelocityRPS);

    // Check if both motors are within tolerance
    boolean topReady = Math.abs(topSpeed - target) < ShooterConstants.VELOCITY_TOLERANCE_RPS;
    boolean bottomReady = Math.abs(bottomSpeed - target) < ShooterConstants.VELOCITY_TOLERANCE_RPS;

    return topReady && bottomReady;
  }

  /**
   * Gets the current velocity of the top flywheel.
   *
   * @return Current velocity in rotations per second
   */
  public double getTopVelocityRPS() {
    return topMotor.getVelocity().getValue().in(RotationsPerSecond);
  }

  /**
   * Gets the current velocity of the bottom flywheel.
   *
   * @return Current velocity in rotations per second
   */
  public double getBottomVelocityRPS() {
    return bottomMotor.getVelocity().getValue().in(RotationsPerSecond);
  }

  /**
   * Gets the target velocity the shooter is trying to reach.
   *
   * @return Target velocity in rotations per second
   */
  public double getTargetVelocityRPS() {
    return targetVelocityRPS;
  }
}
