// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.IntakeConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;

/**
 * Intake subsystem - Roller mechanism for collecting game pieces from the floor.
 *
 * <h2>What is a Subsystem?</h2>
 * <p>In FRC programming, a <b>subsystem</b> represents a major mechanism on the robot.
 * Each subsystem:
 * <ul>
 *   <li>Controls specific hardware (motors, sensors, pneumatics)</li>
 *   <li>Has one "owner" at a time - only one command can control it</li>
 *   <li>Updates periodically (every 20ms) via the {@link #periodic()} method</li>
 * </ul>
 *
 * <p>Examples of subsystems: Drivetrain, Arm, Shooter, Intake, Elevator
 *
 * <h2>How This Intake Works</h2>
 * <p>This intake uses a single TalonFX motor to spin hex shaft rollers with "sushi rolls"
 * (compliant wheels). When the rollers spin, they grab soft balls from the floor and pull
 * them into the robot.
 *
 * <pre>
 *     [HOPPER/SHOOTER]
 *          ↑ balls travel up
 *     ┌────┴────┐
 *     │ ○─────○ │  ← Intake rollers spin to grab balls
 *     └────┬────┘
 *          │
 *     [FLOOR/BALLS]
 * </pre>
 *
 * <h2>Velocity Control Explained</h2>
 * <p>Instead of just turning the motor "on" or "off", we use <b>velocity control</b>.
 * This means we tell the motor HOW FAST to spin (in rotations per second), and the motor
 * controller automatically adjusts power to maintain that speed.
 *
 * <p>Benefits:
 * <ul>
 *   <li>Consistent speed even as battery voltage drops</li>
 *   <li>Can easily change intake speed for different situations</li>
 *   <li>Can detect if something is blocking the intake (speed drops)</li>
 * </ul>
 *
 * <h2>Command Factory Pattern</h2>
 * <p>This class uses the <b>command factory pattern</b>. Instead of having public methods
 * like {@code intake()} that directly control the motor, we have methods like
 * {@code intakeCommand()} that RETURN a {@link Command} object.
 *
 * <p>Why use this pattern?
 * <ul>
 *   <li><b>Safety:</b> Commands automatically handle starting and stopping</li>
 *   <li><b>Scheduling:</b> WPILib's command scheduler manages which command runs</li>
 *   <li><b>Composition:</b> Commands can be combined (parallel, sequential, etc.)</li>
 *   <li><b>Debugging:</b> Commands have names that show up in dashboards</li>
 * </ul>
 *
 * <h2>Toggle vs Hold Button Modes</h2>
 * <p>The commands in this class are designed to work with BOTH control styles:
 *
 * <h3>Hold-to-Run (using whileTrue)</h3>
 * <p>The intake runs ONLY while you hold the button down. When you release,
 * it stops immediately. This is the SAFER option.
 * <pre>
 * // In RobotContainer.java:
 * rightBumper.whileTrue(intake.intakeCommand());
 * </pre>
 *
 * <h3>Toggle On/Off (using toggleOnTrue)</h3>
 * <p>Press once to start the intake, press again to stop it. The intake
 * keeps running even after you release the button. More convenient but
 * requires you to remember to stop it!
 * <pre>
 * // In RobotContainer.java:
 * rightBumper.toggleOnTrue(intake.intakeCommand());
 * </pre>
 *
 * <h2>Why We Use startEnd()</h2>
 * <p>All continuous commands in this class use {@code startEnd()} instead of {@code run()}.
 * This is CRITICAL for toggle mode to work correctly!
 *
 * <p>{@code startEnd(startAction, endAction)} does:
 * <ul>
 *   <li>startAction runs once when the command starts</li>
 *   <li>endAction runs once when the command ends (for ANY reason)</li>
 * </ul>
 *
 * <p>This ensures the motor ALWAYS stops when the command ends, whether:
 * <ul>
 *   <li>You release the button (whileTrue mode)</li>
 *   <li>You press the button again (toggleOnTrue mode)</li>
 *   <li>Another command interrupts this one</li>
 *   <li>The robot is disabled</li>
 * </ul>
 *
 * @see IntakeConstants for all configuration values
 * @see edu.wpi.first.wpilibj2.command.SubsystemBase for the base class
 */
@Logged
public class Intake extends SubsystemBase {

  // ==================== Hardware ====================
  // The actual motor that spins the intake rollers

  /**
   * The TalonFX motor controller for the intake.
   *
   * <p>Protected (not private) so the simulation subclass can access it
   * to update simulated encoder values.
   */
  protected final TalonFX motor =
      new TalonFX(IntakeConstants.MOTOR_CAN_ID, TunerConstants.kCANBus);

  // ==================== Control Requests ====================
  // These are "requests" we send to the motor to tell it what to do

  /**
   * Velocity control request using voltage output.
   *
   * <p>VelocityVoltage tells the motor: "spin at this speed (in RPS),
   * and figure out the voltage needed to get there."
   *
   * <p>We create ONE instance and reuse it with different values.
   * This is more efficient than creating new objects every loop.
   */
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  // ==================== Configuration ====================
  // Motor settings that get applied once at startup

  /**
   * Configuration object for the TalonFX motor.
   *
   * <p>Protected so the simulation subclass can modify simulation-specific
   * settings (like different PID values for simulation).
   */
  protected TalonFXConfiguration config = new TalonFXConfiguration();

  // ==================== Constructor ====================

  /**
   * Creates a new Intake subsystem.
   *
   * <p>The constructor is where we:
   * <ol>
   *   <li>Set up the motor configuration (direction, limits, control values)</li>
   *   <li>Apply the configuration to the motor</li>
   *   <li>Log whether configuration succeeded</li>
   * </ol>
   *
   * <p>This only runs ONCE when the robot code starts.
   */
  public Intake() {
    // ----- Motor Output Settings -----

    // Brake mode: Motor resists spinning when no power is applied.
    // This helps the intake stop quickly when we tell it to stop.
    // (Coast mode would let it spin freely, which we don't want for intake)
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Motor direction: Positive values = counterclockwise rotation.
    // You may need to change this depending on how your motor is mounted!
    // If the intake spins the wrong way, change to Clockwise_Positive.
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // ----- Current Limits (Motor Protection) -----

    // Current limiting prevents the motor from drawing too much power.
    // This protects against:
    //   - Motor burnout from overheating
    //   - Mechanism damage if something jams
    //   - Battery brownouts from sudden power spikes
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimit = IntakeConstants.CURRENT_LIMIT_AMPS;
    currentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    // ----- Velocity Control Settings (Slot 0) -----

    // These values control HOW the motor reaches and maintains target speed.
    // The motor uses the formula: output = kS + kV * velocity + kP * error
    config.Slot0.kS = IntakeConstants.kS; // Static friction compensation
    config.Slot0.kV = IntakeConstants.kV; // Velocity feedforward
    config.Slot0.kP = IntakeConstants.kP; // Proportional gain for error correction

    // ----- Apply Configuration -----

    // Apply the configuration to the motor with automatic retries.
    // CAN communication can sometimes fail, so we retry a few times.
    // Log the result so we can see if configuration succeeded.
    if (TalonFXUtil.applyConfigWithRetries(motor, config, 3)) {
      Robot.telemetry().log("Intake/Config", true);
    } else {
      Robot.telemetry().log("Intake/Config", false);
      // Note: The robot will still run, but the motor might behave unexpectedly
    }
  }

  // ==================== Periodic Methods ====================

  /**
   * Called every robot loop (approximately every 20ms).
   *
   * <p>This is where you would:
   * <ul>
   *   <li>Read sensor values</li>
   *   <li>Update telemetry/logging</li>
   *   <li>Check for problems</li>
   * </ul>
   *
   * <p>For this simple intake, we don't need to do anything here.
   * The motor handles velocity control internally.
   */
  @Override
  public void periodic() {
    // Velocity control is handled by the TalonFX internally.
    // We could add telemetry here if needed, but @Logged handles most of it.
  }

  // ==================== Private Motor Control Methods ====================
  // These are private to enforce command-based control flow

  /**
   * Sets the intake to spin at a specific velocity.
   *
   * <p>PRIVATE because we want all control to go through Commands.
   * This prevents accidental direct calls that bypass the command system.
   *
   * @param velocity Target velocity (positive = intake, negative = outtake)
   */
  private void setVelocity(AngularVelocity velocity) {
    motor.setControl(velocityRequest.withVelocity(velocity));
  }

  /**
   * Stops the intake motor.
   *
   * <p>PRIVATE because we want all control to go through Commands.
   */
  private void stop() {
    motor.stopMotor();
  }

  // ==================== Command Factory Methods ====================
  // These return Command objects for the command scheduler to manage

  /**
   * Command to run the intake (pull game pieces in).
   *
   * <p>This command:
   * <ul>
   *   <li>Starts spinning the intake at INTAKE_SPEED_RPS when scheduled</li>
   *   <li>Keeps running until cancelled or interrupted</li>
   *   <li>Automatically stops the motor when it ends</li>
   * </ul>
   *
   * <h3>Usage - Hold-to-Run:</h3>
   * <pre>
   * // Intake runs while button is held, stops when released
   * rightBumper.whileTrue(intake.intakeCommand());
   * </pre>
   *
   * <h3>Usage - Toggle:</h3>
   * <pre>
   * // Press to start, press again to stop
   * rightBumper.toggleOnTrue(intake.intakeCommand());
   * </pre>
   *
   * @return Command that runs the intake and stops when finished
   */
  public Command intakeCommand() {
    // startEnd(startAction, endAction):
    // - startAction: runs ONCE when command starts → spin up intake
    // - endAction: runs ONCE when command ends → stop motor
    //
    // This guarantees the motor stops no matter HOW the command ends!
    return startEnd(
            () -> setVelocity(RotationsPerSecond.of(IntakeConstants.INTAKE_SPEED_RPS)),
            () -> stop())
        .withName("Intake"); // Name shows up in dashboards/logs
  }

  /**
   * Command to run the intake in reverse (push game pieces out).
   *
   * <p>Use this to:
   * <ul>
   *   <li>Eject a game piece you don't want</li>
   *   <li>Clear a jam</li>
   *   <li>Pass a game piece to a teammate</li>
   * </ul>
   *
   * <p>Works with both hold-to-run and toggle modes (see intakeCommand docs).
   *
   * @return Command that runs the intake in reverse and stops when finished
   */
  public Command outtakeCommand() {
    return startEnd(
            () -> setVelocity(RotationsPerSecond.of(IntakeConstants.OUTTAKE_SPEED_RPS)),
            () -> stop())
        .withName("Outtake");
  }

  /**
   * Command to stop the intake immediately.
   *
   * <p>Use this for:
   * <ul>
   *   <li>Emergency stop</li>
   *   <li>Explicitly stopping when using other control schemes</li>
   *   <li>Coordinated commands where you need to stop the intake at a specific time</li>
   * </ul>
   *
   * <p>Note: If using intakeCommand() or outtakeCommand() with proper button bindings,
   * you usually don't need this - those commands stop automatically when they end.
   *
   * @return Command that stops the intake (runs once and finishes immediately)
   */
  public Command stopCommand() {
    // runOnce() runs the action once and immediately finishes.
    // Good for one-shot actions like "stop right now".
    return runOnce(() -> stop()).withName("StopIntake");
  }

  // ==================== State Query Methods ====================
  // These let other code check the intake's current state

  /**
   * Checks if the intake is spinning at (or near) its target speed.
   *
   * <p>This is useful for:
   * <ul>
   *   <li>Coordinated commands that wait for the intake to be ready</li>
   *   <li>Dashboard indicators showing intake status</li>
   *   <li>Detecting if something is jamming the intake (speed drops)</li>
   * </ul>
   *
   * <p>The check uses a tolerance (see {@link IntakeConstants#VELOCITY_TOLERANCE_RPS})
   * because motors can't hit an exact speed perfectly.
   *
   * @return true if within tolerance of target speed, false otherwise
   */
  public boolean isAtSpeed() {
    AngularVelocity currentVelocity = motor.getVelocity().getValue();
    AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
    AngularVelocity tolerance = RotationsPerSecond.of(IntakeConstants.VELOCITY_TOLERANCE_RPS);

    return currentVelocity.isNear(targetVelocity, tolerance);
  }

  /**
   * Gets the current velocity of the intake motor.
   *
   * <p>Useful for telemetry/debugging to see how fast the intake is actually spinning.
   *
   * @return Current angular velocity of the intake
   */
  public AngularVelocity getVelocity() {
    return motor.getVelocity().getValue();
  }

  /**
   * Gets the target velocity the intake is trying to reach.
   *
   * <p>This is what we told the motor to spin at. Compare with {@link #getVelocity()}
   * to see how close we are to the target.
   *
   * @return Target angular velocity
   */
  public AngularVelocity getTargetVelocity() {
    return velocityRequest.getVelocityMeasure();
  }
}
