package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class StopIntake extends InstantCommand {
  private final Intake subsystem;

  /**
   * Creates a new SetIntakeSpeed.
   *
   * @param subsystem The subsystem used by this command.
   */
  public StopIntake(Intake subsystem) {
    this.subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Sets intake speed
    subsystem.stopIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Stops motors
  }

}