package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the kicker motor
 * @author Nicholas S
 *
 */
public class StartIntake extends CommandBase {

  private final Intake intake;

  public StartIntake(Intake intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.start();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}