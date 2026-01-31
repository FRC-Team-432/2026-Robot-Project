// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulated version of the Feeder subsystem for desktop testing.
 *
 * <h2>What is This?</h2>
 * <p>This class lets you test the feeder code WITHOUT a real robot!
 * When you run in simulation mode, this simulates how the feeder
 * motor would respond to commands.
 *
 * <h2>Running in Simulation</h2>
 * <ol>
 *   <li>Run: {@code ./gradlew simulateJava}</li>
 *   <li>Open the simulation GUI</li>
 *   <li>Use simulated joysticks to test feeder commands</li>
 *   <li>Watch the feeder values in SmartDashboard</li>
 * </ol>
 *
 * @see Feeder for the real hardware implementation
 */
public class FeederSIM extends Feeder {

  // ==================== Simulation Objects ====================

  /** Simulation state for the feeder motor. */
  private final TalonFXSimState motorSim;

  /** Physics simulation for the feeder mechanism (modeled as a simple flywheel). */
  private final FlywheelSim feederSim;

  // Feeder physics parameters
  private static final double FEEDER_MOI_KG_M2 = 0.005; // Small moment of inertia
  private static final double FEEDER_GEARING = 1.0;

  // ==================== Constructor ====================

  /**
   * Creates a simulated Feeder.
   */
  public FeederSIM() {
    super();

    motorSim = motor.getSimState();

    feederSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getFalcon500(1), FEEDER_MOI_KG_M2, FEEDER_GEARING),
            DCMotor.getFalcon500(1));
  }

  // ==================== Simulation Update ====================

  @Override
  public void simulationPeriodic() {
    feederSim.setInputVoltage(motorSim.getMotorVoltage());
    feederSim.update(0.020);
    motorSim.setRotorVelocity(feederSim.getAngularVelocityRPM() / 60.0);
  }
}
