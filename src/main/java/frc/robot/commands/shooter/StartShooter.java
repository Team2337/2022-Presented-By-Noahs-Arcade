package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *Runs the shooter when a button is held and make sure it stops correctly
 * 
 * @author Nicholas S.
 */
public class StartShooter extends CommandBase {
  // The subsystem the command runs on
  private final Shooter subsystem;

  public StartShooter(Shooter m_subsystem) {
    subsystem = m_subsystem;
    addRequirements(subsystem);

  }
  public void initialize() {
    subsystem.currentLimitConfigurationMotor.currentLimit = 50;
    subsystem.leftShoot.configStatorCurrentLimit(subsystem.currentLimitConfigurationMotor, 0);
    subsystem.leftShoot.configClosedloopRamp(0.1);
  }

  @Override
  public void execute() {
    subsystem.setShooterSpeed(subsystem.shooter.getDouble(0));
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.currentLimitConfigurationMotor.currentLimit = 0;
    subsystem.leftShoot.configStatorCurrentLimit(subsystem.currentLimitConfigurationMotor, 0);
    
    subsystem.leftShoot.configClosedloopRamp(0.1);
    subsystem.stopShooter();
  }


  @Override
  public boolean isFinished() {
    if (subsystem.motorOverTemp) { 
      return true;
    } else {
      return false;
    }
    
  }
}