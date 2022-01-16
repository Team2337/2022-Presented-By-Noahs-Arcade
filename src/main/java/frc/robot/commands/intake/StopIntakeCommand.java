package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class StopIntakeCommand extends InstantCommand {
  private final Intake subsystem;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public StopIntakeCommand(Intake subsystem) {
    this.subsystem = subsystem;
    
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    //Sets intake speed
    subsystem.stopIntake();
  }

}