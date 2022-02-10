package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *Runs the intake
 *@author Nicholas S.
 */

public class RunIntake extends CommandBase {
  // The subsystem the command runs on
  private final Intake intake;

  public RunIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.startIntake();
  }

  @Override
  public void execute() {  
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}