package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** An example command that uses an example subsystem. */
public class SetIntakeSpeed extends InstantCommand {
  private final Intake subsystem;
  private final double speed;

  /**
   * Creates a new SetIntakeSpeed.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetIntakeSpeed(Intake subsystem, double speed) {
    this.subsystem = subsystem;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Sets intake speed
    subsystem.setIntakeSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}