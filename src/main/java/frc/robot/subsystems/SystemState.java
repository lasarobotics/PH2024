package frc.robot.subsystems;

public interface SystemState {
  public default void initialize() {}

  public default void execute() {}

  public default void end(boolean interrupted) {}

  public SystemState nextState();
}
