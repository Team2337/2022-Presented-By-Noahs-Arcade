package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *Runs the shooter when a button is held and make sure it stops correctly
 * 
 * @author Nicholas S.
 */
public class StartShooter extends CommandBase {
  // The shooter the command runs on
  private final Shooter shooter;

  public StartShooter(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.configureMotorStart();
  }

  @Override
  public void execute() {
    shooter.setShooterSpeed(shooter.shooter.getDouble(0));
  }

  @Override
  public void end(boolean interrupted) {
    shooter.configureMotorStop();
    shooter.stopShooter();
  }

  @Override
  public boolean isFinished() {
    if (shooter.isMotorOverheated()) { 
      return true;
    } else {
      return false;
    }   
  }
}