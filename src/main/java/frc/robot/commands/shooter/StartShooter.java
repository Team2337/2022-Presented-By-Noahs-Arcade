package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
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
    subsystem.topShoot.configStatorCurrentLimit(subsystem.currentLimitConfigurationMotor, 0);
    subsystem.bottomShoot.configStatorCurrentLimit(subsystem.currentLimitConfigurationMotor, 0);
    subsystem.topShoot.configClosedloopRamp(0.0);
    subsystem.bottomShoot.configClosedloopRamp(0.0);
  }

  @Override
  public void execute() {
    subsystem.setTopShooterSpeed(subsystem.topShooter.getDouble(0));
    subsystem.setBottomShooterSpeed(subsystem.topShooter.getDouble(0));

  }

  @Override
  public void end(boolean interrupted) {
    subsystem.currentLimitConfigurationMotor.currentLimit = 0;
    subsystem.topShoot.configStatorCurrentLimit(subsystem.currentLimitConfigurationMotor, 0);
    subsystem.bottomShoot.configStatorCurrentLimit(subsystem.currentLimitConfigurationMotor, 0);
    
    subsystem.topShoot.configClosedloopRamp(0.1);
    subsystem.bottomShoot.configClosedloopRamp(0.1);
    subsystem.stopTopShooter();
    subsystem.stopBottomShooter();
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