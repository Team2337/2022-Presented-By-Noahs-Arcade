package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Runs the intake motor
 * @author Madison J.
 *
 */
public class AutoStartIntake extends InstantCommand {

  private final Intake intake;

  public AutoStartIntake(Intake intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.start();
  }

}