package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Runs the kicker motor
 * @author Nicholas S
 *
 */
public class StartIntake extends InstantCommand {

  private final Intake intake;

  public StartIntake(Intake intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.start();
  }

}