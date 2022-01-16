package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class StopIntake extends InstantCommand {
  private final Intake subsystem;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public StopIntake(Intake subsystem) {
    this.subsystem = subsystem;
    
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    //Sets intake speed
    subsystem.stopIntake();
  }

}