package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *Runs the shooter when a button is held and make sure it stops correctly
 *
 * @author Nicholas S.
 */
public class QuickStartShooter extends CommandBase {

  private final Shooter shooter;
  private double speed;

  public QuickStartShooter(double speed, Shooter shooter) {
    this.shooter = shooter;
    this.speed = speed;

    addRequirements(shooter);
  }
  @Override
  public void initialize() {
    shooter.setSpeed(speed);
  }

  @Override
  public boolean isFinished() {
    return shooter.isShooterToSpeed();
  }

}