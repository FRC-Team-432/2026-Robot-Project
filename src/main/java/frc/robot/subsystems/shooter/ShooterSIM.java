// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulated version of the Shooter subsystem for desktop testing.
 *
 * <h2>What is This?</h2>
 * <p>This class lets you test the shooter code WITHOUT a real robot!
 * When you run the robot code in simulation mode, this class simulates
 * how the flywheel physics would work.
 *
 * <h2>How Simulation Works</h2>
 * <p>WPILib provides physics simulation classes that model real mechanisms:
 *
 * <pre>
 *   REAL ROBOT:                      SIMULATION:
 *   ───────────────                  ───────────────
 *   Motor gets voltage      →        FlywheelSim calculates
 *   Flywheel spins up                how fast wheel WOULD spin
 *   Encoder reads speed              Fake encoder value updated
 *
 *   Your code can't tell the difference!
 *   It just sees "motor commanded, encoder reading speed"
 * </pre>
 *
 * <h2>Running in Simulation</h2>
 * <ol>
 *   <li>Run: {@code ./gradlew simulateJava}</li>
 *   <li>The simulation GUI opens</li>
 *   <li>Use the "Joysticks" panel to simulate controller input</li>
 *   <li>Watch the shooter values in SmartDashboard/Shuffleboard</li>
 * </ol>
 *
 * @see Shooter for the real hardware implementation
 */
public class ShooterSIM extends Shooter {

  // ==================== Simulation Objects ====================

  /**
   * Simulation state for the top motor.
   *
   * <p>This object lets us inject simulated sensor values into the motor controller.
   */
  private final TalonFXSimState topMotorSim;

  /**
   * Simulation state for the bottom motor.
   */
  private final TalonFXSimState bottomMotorSim;

  /**
   * Physics simulation for the top flywheel.
   *
   * <p>FlywheelSim models how a spinning mass accelerates and decelerates.
   */
  private final FlywheelSim topFlywheelSim;

  /**
   * Physics simulation for the bottom flywheel.
   */
  private final FlywheelSim bottomFlywheelSim;

  // Flywheel physics parameters
  // These approximate a typical FRC shooter flywheel
  private static final double FLYWHEEL_MOI_KG_M2 = 0.01; // Moment of inertia
  private static final double FLYWHEEL_GEARING = 1.0; // Direct drive (1:1)

  // ==================== Constructor ====================

  /**
   * Creates a simulated Shooter.
   *
   * <p>Sets up physics simulation for both flywheels.
   */
  public ShooterSIM() {
    super(); // Call parent constructor to set up motors

    // Get simulation state objects from the motors
    topMotorSim = topMotor.getSimState();
    bottomMotorSim = bottomMotor.getSimState();

    // Create flywheel physics simulations
    // DCMotor.getFalcon500(1) models a single Falcon 500 motor
    topFlywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getFalcon500(1), FLYWHEEL_MOI_KG_M2, FLYWHEEL_GEARING),
            DCMotor.getFalcon500(1));

    bottomFlywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getFalcon500(1), FLYWHEEL_MOI_KG_M2, FLYWHEEL_GEARING),
            DCMotor.getFalcon500(1));
  }

  // ==================== Simulation Update ====================

  /**
   * Called every simulation loop to update physics.
   *
   * <p>This method:
   * <ol>
   *   <li>Reads the voltage being sent to each motor</li>
   *   <li>Updates the physics simulation with that voltage</li>
   *   <li>Calculates the resulting wheel speed</li>
   *   <li>Feeds that speed back to the motor's simulated encoder</li>
   * </ol>
   */
  @Override
  public void simulationPeriodic() {
    // Update top flywheel simulation
    topFlywheelSim.setInputVoltage(topMotorSim.getMotorVoltage());
    topFlywheelSim.update(0.020); // 20ms loop time
    topMotorSim.setRotorVelocity(topFlywheelSim.getAngularVelocityRPM() / 60.0);

    // Update bottom flywheel simulation
    bottomFlywheelSim.setInputVoltage(bottomMotorSim.getMotorVoltage());
    bottomFlywheelSim.update(0.020);
    bottomMotorSim.setRotorVelocity(bottomFlywheelSim.getAngularVelocityRPM() / 60.0);
  }
}
