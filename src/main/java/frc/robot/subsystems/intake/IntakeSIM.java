// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.MechanismUtil;
import frc.robot.utils.TalonFXUtil;

/**
 * Simulation implementation of the Intake subsystem.
 *
 * <h2>Why Do We Need Simulation?</h2>
 * <p>Simulation lets us test our robot code WITHOUT the actual robot! This is useful for:
 * <ul>
 *   <li><b>Testing at home:</b> Work on code without access to the robot</li>
 *   <li><b>Faster iteration:</b> No need to deploy to the robot for every test</li>
 *   <li><b>Safe testing:</b> Test potentially dangerous code without risk</li>
 *   <li><b>Debugging:</b> Easier to step through code and see what's happening</li>
 *   <li><b>Tuning:</b> Get initial PID values before testing on real hardware</li>
 * </ul>
 *
 * <h2>How Simulation Works</h2>
 * <p>The simulation does several things each robot loop:
 * <ol>
 *   <li><b>Read motor voltage:</b> What voltage is our code sending to the motor?</li>
 *   <li><b>Physics simulation:</b> Given that voltage, how fast would the real motor spin?</li>
 *   <li><b>Update encoder:</b> Tell the motor's simulated encoder the new velocity</li>
 *   <li><b>Update battery:</b> Simulate battery voltage drop from current draw</li>
 *   <li><b>Visualization:</b> Update the on-screen display</li>
 * </ol>
 *
 * <h2>Inheritance Explained</h2>
 * <p>This class <b>extends</b> the real {@link Intake} class. This means:
 * <ul>
 *   <li>It inherits ALL the real code (commands, motor setup, etc.)</li>
 *   <li>Only simulation-specific code needs to be added here</li>
 *   <li>When we create {@code new IntakeSIM()}, we get both real AND sim behavior</li>
 * </ul>
 *
 * <p>This is the "Hardware/SIM split" pattern used throughout this codebase.
 *
 * <h2>Visualization</h2>
 * <p>The intake is visualized as a spinning wheel using WPILib's Mechanism2d.
 * You can see this in SmartDashboard or Shuffleboard under "Intake Sim".
 * <ul>
 *   <li><b>Red wheel:</b> Intake is NOT at target speed</li>
 *   <li><b>Green wheel:</b> Intake IS at target speed</li>
 * </ul>
 *
 * @see Intake for the real hardware implementation
 * @see DCMotorSim for WPILib's motor physics simulation
 */
public class IntakeSIM extends Intake {

  // ==================== Physical Constants ====================
  // These describe the physical properties of the intake for simulation

  /**
   * Gear ratio between motor and intake rollers.
   *
   * <p>1.0 means direct drive (no gearbox). If you have a gearbox:
   * <ul>
   *   <li>3:1 gearbox = motor spins 3x for every 1 roller spin = ratio of 3.0</li>
   *   <li>1:2 gearbox = motor spins 0.5x for every 1 roller spin = ratio of 0.5</li>
   * </ul>
   *
   * <p>Our intake uses direct drive, so the ratio is 1.0.
   */
  private static final double GEAR_RATIO = 1.0;

  /**
   * Moment of inertia of the intake rollers (in kg⋅m²).
   *
   * <p>This represents how hard it is to speed up or slow down the rollers.
   * <ul>
   *   <li>Higher MOI = harder to accelerate (like a heavy flywheel)</li>
   *   <li>Lower MOI = easier to accelerate (like a light wheel)</li>
   * </ul>
   *
   * <p>0.005 kg⋅m² is a reasonable estimate for intake rollers.
   * The actual value depends on your specific rollers and shaft.
   */
  private static final double INTAKE_MOI = 0.005;

  /**
   * Simulation update period in seconds.
   *
   * <p>This matches the standard robot loop period of 20 milliseconds (50 Hz).
   * The simulation updates every time the real robot code would update.
   */
  private static final double SIM_PERIOD_SECONDS = 0.020;

  /**
   * Visual radius of the intake wheel for Mechanism2d display (in pixels).
   *
   * <p>This only affects the visualization size, not the physics simulation.
   * 60 pixels gives a nice visible wheel on the dashboard.
   */
  private static final double INTAKE_VISUAL_RADIUS = 60.0;

  // ==================== Simulation Components ====================

  /**
   * DC motor model for simulation.
   *
   * <p>This contains the electrical and mechanical characteristics of the motor:
   * <ul>
   *   <li>Free speed (how fast it spins with no load)</li>
   *   <li>Stall torque (how much force it can apply at 0 speed)</li>
   *   <li>Stall current (how much current it draws when stalled)</li>
   * </ul>
   *
   * <p>We use 1 Kraken X60 motor for our intake.
   */
  private final DCMotor dcMotor = DCMotor.getKrakenX60(1);

  /**
   * Physics simulation of the intake mechanism.
   *
   * <p>This simulates the physics of a spinning mass (flywheel physics).
   * Given a voltage input, it calculates:
   * <ul>
   *   <li>How fast the intake spins</li>
   *   <li>How much current the motor draws</li>
   *   <li>How the system accelerates/decelerates</li>
   * </ul>
   */
  private final DCMotorSim intakeSim;

  /**
   * Mechanism visualization for the SmartDashboard.
   *
   * <p>This creates a visual representation of the intake that shows:
   * <ul>
   *   <li>A spinning wheel that rotates with the simulated velocity</li>
   *   <li>Color changes (red → green) when at target speed</li>
   * </ul>
   */
  private final MechanismUtil.FlywheelMechanism intakeMechanism;

  // ==================== Constructor ====================

  /**
   * Creates a new IntakeSIM instance.
   *
   * <p>This constructor:
   * <ol>
   *   <li>Calls the parent Intake constructor (sets up real motor config)</li>
   *   <li>Overrides some config values for better simulation behavior</li>
   *   <li>Creates the physics simulation</li>
   *   <li>Sets up the visualization</li>
   * </ol>
   */
  public IntakeSIM() {
    // Call the parent constructor first - this sets up all the real motor config
    super();

    // ----- Override Some Config Values for Simulation -----

    // Set the gear ratio so the simulation knows the relationship
    // between motor shaft and mechanism
    config.Feedback.RotorToSensorRatio = GEAR_RATIO;

    // Simulation often needs different PID values than real hardware.
    // These values work well for the simulated physics model.
    config.Slot0.kS = 0.0; // Static friction (less needed in sim)
    config.Slot0.kV = 0.12; // Velocity feedforward (12V / 100 RPS ≈ 0.12)
    config.Slot0.kP = 0.1; // Proportional gain

    // Re-apply the modified configuration to the motor
    TalonFXUtil.applyConfigWithRetries(motor, config);

    // ----- Create Physics Simulation -----

    // Create a linear system model for a DC motor driving a flywheel/intake
    // This uses control theory math to model the physics accurately
    LinearSystem<N2, N1, N2> linearSystem =
        LinearSystemId.createDCMotorSystem(dcMotor, INTAKE_MOI, GEAR_RATIO);

    // Create the simulation with our motor and physics model
    intakeSim = new DCMotorSim(linearSystem, dcMotor);

    // ----- Create Visualization -----

    // Create a flywheel-style visualization (spinning wheel with spokes)
    intakeMechanism = new MechanismUtil.FlywheelMechanism("Intake", INTAKE_VISUAL_RADIUS);

    // Publish the visualization to SmartDashboard so we can see it
    SmartDashboard.putData("Intake Sim", intakeMechanism.getMechanism());
  }

  // ==================== Simulation Periodic ====================

  /**
   * Updates the simulation every robot loop.
   *
   * <p>This method is called automatically by WPILib when running in simulation mode.
   * It handles all the physics simulation and visualization updates.
   *
   * <h3>Step-by-Step Process:</h3>
   * <ol>
   *   <li><b>Get motor voltage:</b> What is our code telling the motor to do?</li>
   *   <li><b>Run physics:</b> Calculate how the motor would respond</li>
   *   <li><b>Simulate battery:</b> Account for battery voltage drop under load</li>
   *   <li><b>Update encoder:</b> Tell the motor controller what velocity to "see"</li>
   *   <li><b>Update position:</b> Track total rotations for encoder position</li>
   *   <li><b>Update display:</b> Animate the visualization</li>
   * </ol>
   */
  @Override
  public void simulationPeriodic() {
    // ----- Step 1: Get Motor Voltage -----
    // Read what voltage our code is sending to the motor
    // (from velocity control, stop commands, etc.)
    double motorVoltage = motor.getMotorVoltage().getValueAsDouble();

    // ----- Step 2: Run Physics Simulation -----
    // Feed the voltage into the physics model
    intakeSim.setInput(motorVoltage);

    // Step the simulation forward by one time period
    // This calculates the new velocity and current draw
    intakeSim.update(SIM_PERIOD_SECONDS);

    // ----- Step 3: Simulate Battery Voltage -----
    // When motors draw current, the battery voltage drops slightly.
    // This simulates that effect (called "voltage sag" or "brownout").
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(intakeSim.getCurrentDrawAmps()));

    // ----- Step 4: Update Simulated Encoder Velocity -----
    // Get the simulated velocity in radians per second
    double velocityRadPerSec = intakeSim.getAngularVelocityRadPerSec();

    // Convert to rotations per second and account for gear ratio
    // (motor velocity = intake velocity * gear ratio)
    double motorVelocityRPS = (velocityRadPerSec / (2 * Math.PI)) * GEAR_RATIO;

    // Tell the motor's simulated encoder what velocity to report
    motor.getSimState().setRotorVelocity(motorVelocityRPS);

    // ----- Step 5: Update Simulated Encoder Position -----
    // Position = integral of velocity over time
    // We approximate this by: new_position = old_position + velocity * dt
    double currentPosition = motor.getRotorPosition().getValueAsDouble();
    double newPosition = currentPosition + (motorVelocityRPS * SIM_PERIOD_SECONDS);
    motor.getSimState().setRawRotorPosition(newPosition);

    // ----- Step 6: Update Visualization -----
    // Animate the spinning wheel based on current velocity and state
    intakeMechanism.update(velocityRadPerSec, SIM_PERIOD_SECONDS, isAtSpeed());

    // ----- Publish Additional Telemetry -----
    // Show current draw in SmartDashboard for debugging
    SmartDashboard.putNumber("Intake Sim Current (A)", intakeSim.getCurrentDrawAmps());
  }
}
