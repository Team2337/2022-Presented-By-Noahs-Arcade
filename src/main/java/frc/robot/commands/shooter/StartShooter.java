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
  public void initialize() {
    shooter.currentLimitConfigurationMotor.currentLimit = 50;
    shooter.leftMotor.configStatorCurrentLimit(shooter.currentLimitConfigurationMotor, 0);
    shooter.leftMotor.configClosedloopRamp(0.1);
  }

  @Override
  public void execute() {
    shooter.setShooterSpeed(shooter.shooter.getDouble(0));
  }

  @Override
  public void end(boolean interrupted) {
    shooter.currentLimitConfigurationMotor.currentLimit = 0;
    shooter.leftMotor.configStatorCurrentLimit(shooter.currentLimitConfigurationMotor, 0);
    shooter.leftMotor.configClosedloopRamp(0.1);
    
    shooter.stopShooter();
  }


  @Override
  public boolean isFinished() {
    if (shooter.motorOverTemp) { 
      return true;
    } else {
      return false;
    }
    
  }
}