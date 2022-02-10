package frc.robot.commands.shooter;

import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the kicker motor
 * @author Nicholas S
 * 
 */
public class RunKicker extends CommandBase {
  // The subsystem the command runs on
  private final Kicker kicker;

  public RunKicker(Kicker kicker) {
    this.kicker = kicker;
    addRequirements(kicker);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    kicker.startKicker(kicker.kick3r.getDouble(0));
  }

  @Override
  public void end(boolean interrupted) {
    kicker.stopKicker();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}