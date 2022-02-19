package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *Runs the shooter when a button is held and make sure it stops correctly
 *
 * @author Nicholas S.
 */
public class StartShooter extends CommandBase {

  private final Shooter shooter;

  public StartShooter(Shooter shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void execute() {
    shooter.setSpeed(shooter.shooterSpeedFeetPerSecondWidget.getDouble(0));
    shooter.updateSpeedTarget(shooter.shooterSpeedFeetPerSecondWidget.getDouble(0));
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return shooter.isOverheated();
  }

}