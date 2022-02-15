package frc.robot.commands.shooter;

import frc.robot.subsystems.Kicker;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the kicker motor
 * @author Nicholas S
 *
 */
public class AutoKickerCommand extends CommandBase {

  private final Kicker kicker;
  private double timeout;

  public AutoKickerCommand(Kicker kicker, double timeout) {
    this.kicker = kicker;
    this.timeout = timeout;
    addRequirements(kicker);
  }

  @Override 
  public void initialize() {
    
  }

  @Override
  public void execute() {
    kicker.start(kicker.kickerSpeedPercentageWidget.getDouble(0));
  }

  @Override
  public void end(boolean interrupted) {
    kicker.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}