package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *Runs the shooter in reverse and make sure it stops correctly
 *
 * @author Nicholas S.
 */
public class ReverseShooter extends CommandBase {

  private final Shooter shooter;

  public ReverseShooter(Shooter shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void execute() {
    shooter.setSpeed(-10);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}