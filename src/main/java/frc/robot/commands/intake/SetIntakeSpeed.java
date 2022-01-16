package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetIntakeSpeed extends InstantCommand {
  private final Intake subsystem;
  private final double speed;

  /**
   * @param subsystem The subsystem used by this command.
   * @param speed The speed (as a percent)
   */
  public SetIntakeSpeed(Intake subsystem, double speed) {
    this.subsystem = subsystem;
    this.speed = speed;
    
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    //Sets intake speed
    subsystem.setIntakeSpeed(speed);
  }

}