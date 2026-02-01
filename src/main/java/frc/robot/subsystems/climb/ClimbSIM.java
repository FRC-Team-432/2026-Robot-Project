// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Simulated Climb subsystem for desktop testing.
 *
 * <h2>Purpose</h2>
 * <p>This class extends the real Climb subsystem but simulates the behavior
 * instead of controlling actual motors. This allows you to:
 * <ul>
 *   <li>Test climb button bindings in simulation</li>
 *   <li>Verify command logic without hardware</li>
 *   <li>See simulated state on SmartDashboard</li>
 * </ul>
 *
 * <h2>How Simulation Works</h2>
 * <p>When commands are executed, this class:
 * <ol>
 *   <li>Tracks simulated state (positions, speeds)</li>
 *   <li>Logs state to SmartDashboard for visibility</li>
 *   <li>Simulates basic physics (position changes over time)</li>
 * </ol>
 *
 * <h2>What Gets Simulated</h2>
 * <pre>
 *   SIMULATED STATE:
 *   ────────────────
 *   flipPosition: 0.0 to 1.0 (0=down, 1=up)
 *   liftPosition: 0.0 to 1.0 (0=lowered, 1=raised)
 *   isFlipping: true when flip command active
 *   isLifting: true when lift command active
 * </pre>
 *
 * <h2>Viewing in Simulation</h2>
 * <p>Open SmartDashboard and look for:
 * <ul>
 *   <li>ClimbSIM/flipPosition</li>
 *   <li>ClimbSIM/liftPosition</li>
 *   <li>ClimbSIM/isFlipping</li>
 *   <li>ClimbSIM/isLifting</li>
 * </ul>
 *
 * <p>Note: This class intentionally does NOT use @Logged annotation since
 * the parent Climb class is already logged. Using @Logged on both causes
 * EPILOGUE annotation conflicts with overridden methods.
 *
 * @see Climb for the real hardware implementation (stub)
 */
public class ClimbSIM extends Climb {

  // ==================== Simulated State ====================

  /** Simulated flip arm position (0 = down, 1 = up). */
  private double flipPosition = 0.0;

  /** Simulated lift position (0 = lowered, 1 = raised). */
  private double liftPosition = 0.0;

  /** Whether the flip arm is currently moving. */
  private boolean isFlipping = false;

  /** Whether the lift is currently moving. */
  private boolean isLifting = false;

  /** Direction of flip motion: 1 = up, -1 = down, 0 = stopped. */
  private int flipDirection = 0;

  /** Direction of lift motion: 1 = up, -1 = down, 0 = stopped. */
  private int liftDirection = 0;

  // ==================== Simulation Parameters ====================

  /** Speed at which flip position changes per second. */
  private static final double FLIP_SPEED = 0.5; // 2 seconds full travel

  /** Speed at which lift position changes per second. */
  private static final double LIFT_SPEED = 0.3; // ~3.3 seconds full travel

  /** Simulation time step (20ms = 0.02s). */
  private static final double DT = 0.02;

  // ==================== Constructor ====================

  /**
   * Creates a new ClimbSIM subsystem.
   *
   * <p>Initializes the simulated climb in the "stowed" position:
   * <ul>
   *   <li>Flip arm down (position 0)</li>
   *   <li>Lift lowered (position 0)</li>
   * </ul>
   */
  public ClimbSIM() {
    super(); // Call parent constructor
    System.out.println("ClimbSIM created (simulation mode)");
  }

  // ==================== Periodic ====================

  /**
   * Called every robot loop (20ms).
   *
   * <p>Updates simulated positions based on current motion commands
   * and logs state to SmartDashboard.
   */
  @Override
  public void periodic() {
    // Update flip position based on direction
    if (flipDirection != 0) {
      flipPosition += flipDirection * FLIP_SPEED * DT;
      flipPosition = Math.max(0.0, Math.min(1.0, flipPosition)); // Clamp to [0, 1]
    }

    // Update lift position based on direction
    if (liftDirection != 0) {
      liftPosition += liftDirection * LIFT_SPEED * DT;
      liftPosition = Math.max(0.0, Math.min(1.0, liftPosition)); // Clamp to [0, 1]
    }

    // Update status flags
    isFlipping = flipDirection != 0;
    isLifting = liftDirection != 0;

    // Log to SmartDashboard
    SmartDashboard.putNumber("ClimbSIM/flipPosition", flipPosition);
    SmartDashboard.putNumber("ClimbSIM/liftPosition", liftPosition);
    SmartDashboard.putBoolean("ClimbSIM/isFlipping", isFlipping);
    SmartDashboard.putBoolean("ClimbSIM/isLifting", isLifting);
    SmartDashboard.putString(
        "ClimbSIM/flipState", flipPosition > 0.9 ? "UP" : flipPosition < 0.1 ? "DOWN" : "MOVING");
    SmartDashboard.putString(
        "ClimbSIM/liftState",
        liftPosition > 0.9 ? "RAISED" : liftPosition < 0.1 ? "LOWERED" : "MOVING");
  }

  // ==================== Command Factory Methods ====================

  /**
   * Command to lift the robot (climb up) - SIMULATED.
   *
   * <p>In simulation, this moves the lift position toward 1.0 (raised).
   *
   * @return Command that simulates lifting the robot
   */
  @Override
  public Command liftRobotCommand() {
    return run(() -> {
          liftDirection = 1; // Move up
        })
        .finallyDo(
            () -> {
              liftDirection = 0; // Stop
            })
        .withName("LiftRobot (SIM)");
  }

  /**
   * Command to drop the robot (lower down) - SIMULATED.
   *
   * <p>In simulation, this moves the lift position toward 0.0 (lowered).
   *
   * @return Command that simulates lowering the robot
   */
  @Override
  public Command dropRobotCommand() {
    return run(() -> {
          liftDirection = -1; // Move down
        })
        .finallyDo(
            () -> {
              liftDirection = 0; // Stop
            })
        .withName("DropRobot (SIM)");
  }

  /**
   * Command to flip the climb arm up - SIMULATED.
   *
   * <p>In simulation, this moves the flip position toward 1.0 (up).
   *
   * @return Command that simulates flipping the arm up
   */
  @Override
  public Command flipUpCommand() {
    return run(() -> {
          flipDirection = 1; // Move up
        })
        .finallyDo(
            () -> {
              flipDirection = 0; // Stop
            })
        .withName("FlipUp (SIM)");
  }

  /**
   * Command to flip the climb arm down - SIMULATED.
   *
   * <p>In simulation, this moves the flip position toward 0.0 (down).
   *
   * @return Command that simulates flipping the arm down
   */
  @Override
  public Command flipDownCommand() {
    return run(() -> {
          flipDirection = -1; // Move down
        })
        .finallyDo(
            () -> {
              flipDirection = 0; // Stop
            })
        .withName("FlipDown (SIM)");
  }

  /**
   * Command to stop all climb motors - SIMULATED.
   *
   * <p>Stops all simulated motion immediately.
   *
   * @return Command that stops all climb motion
   */
  @Override
  public Command stopCommand() {
    return runOnce(
            () -> {
              flipDirection = 0;
              liftDirection = 0;
            })
        .withName("ClimbStop (SIM)");
  }

  // ==================== Simulation State Access ====================

  /**
   * Gets the simulated flip arm position.
   *
   * @return Position from 0.0 (down) to 1.0 (up)
   */
  public double getFlipPosition() {
    return flipPosition;
  }

  /**
   * Gets the simulated lift position.
   *
   * @return Position from 0.0 (lowered) to 1.0 (raised)
   */
  public double getLiftPosition() {
    return liftPosition;
  }

  /**
   * Checks if the flip arm is fully up.
   *
   * @return true if flip position is at or above 0.95
   */
  public boolean isFlipUp() {
    return flipPosition >= 0.95;
  }

  /**
   * Checks if the flip arm is fully down.
   *
   * @return true if flip position is at or below 0.05
   */
  public boolean isFlipDown() {
    return flipPosition <= 0.05;
  }

  /**
   * Checks if the lift is fully raised.
   *
   * @return true if lift position is at or above 0.95
   */
  public boolean isLiftRaised() {
    return liftPosition >= 0.95;
  }

  /**
   * Checks if the lift is fully lowered.
   *
   * @return true if lift position is at or below 0.05
   */
  public boolean isLiftLowered() {
    return liftPosition <= 0.05;
  }

  /**
   * Resets the simulation to the initial "stowed" state.
   *
   * <p>Useful for testing or when starting a new match.
   */
  public void resetSimulation() {
    flipPosition = 0.0;
    liftPosition = 0.0;
    flipDirection = 0;
    liftDirection = 0;
    isFlipping = false;
    isLifting = false;
  }
}
