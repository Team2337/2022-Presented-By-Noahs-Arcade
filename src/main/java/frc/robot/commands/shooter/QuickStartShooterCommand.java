package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *Runs the shooter when a button is held and make sure it stops correctly
 *
 * @author Nicholas S.
 */
public class QuickStartShooterCommand extends CommandBase {

  private final double speedFeetPerSecond;
  private final Shooter shooter;

  public QuickStartShooterCommand(double speedFeetPerSecond, Shooter shooter) {
    this.speedFeetPerSecond = speedFeetPerSecond;
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setSpeed(speedFeetPerSecond);
  }

  @Override
  public boolean isFinished() {
    return shooter.isShooterToSpeed();
  }

}