package frc.robot.commands.shooter;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *Runs intake
 *@author Nicholas S.
 */
public class RunIntake extends CommandBase {
  // The subsystem the command runs on
  private final Intake subsystem;

  public RunIntake(Intake intake) {
    subsystem = intake;
    addRequirements(subsystem);

  }
  public void initialize() {

  }

  @Override
  public void execute() {
    subsystem.startIntake();
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.stopIntake();;
  }


  @Override
  public boolean isFinished() {
    return false;
}
}