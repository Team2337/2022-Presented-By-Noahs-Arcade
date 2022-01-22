package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
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

  @Override
  public void execute() {
    subsystem.setTopShooterSpeed(subsystem.topShooter.getDouble(0));
    subsystem.setBottomShooterSpeed(subsystem.bottomShooter.getDouble(0));
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.currentLimitConfigurationMotor.currentLimit = 0;
    subsystem.topShoot.configStatorCurrentLimit(subsystem.currentLimitConfigurationMotor, 0);
    subsystem.bottomShoot.configStatorCurrentLimit(subsystem.currentLimitConfigurationMotor, 0);
    
    subsystem.topShoot.configClosedloopRamp(0.5);
    subsystem.bottomShoot.configClosedloopRamp(0.5);
    subsystem.stopTopShooter();
    subsystem.stopBottomShooter();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}