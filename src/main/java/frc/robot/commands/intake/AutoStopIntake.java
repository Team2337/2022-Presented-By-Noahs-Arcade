package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Stops the intake motor
 * @author Madison J.
 *
 */
public class AutoStopIntake extends InstantCommand {

  private final Intake intake;

  public AutoStopIntake(Intake intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.stop();
  }

}