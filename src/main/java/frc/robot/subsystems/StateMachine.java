package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface StateMachine extends Subsystem {
  /**
   * Get current system state
   * @return Current system state
   */
  public SystemState getState();

  /**
   * Iterate system to next state
   * <p>
   * ONLY TO BE USED BY {@link StateCommand#end(boolean)}
   * <p>
   * DO NOT CALL THIS METHOD!
   */
  public void setState(SystemState state);

  /**
   * Sets the default command for this subsystem using the state machine architecture
   * @param commands List of state commands
   */
  public default void setDefaultCommand() {
    // Set default command to a repeating select command, with the state getter as the selector
    setDefaultCommand(new StateCommand(() -> getState(), this).repeatedly());
  }
}
