package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */
public class StopShooter extends CommandBase {
  // The subsystem the command runs on
  private final Shooter subsystem;

  public StopShooter(Shooter m_subsystem) {
    subsystem = m_subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    subsystem.setTopShooterSpeed(0);
    subsystem.setBottomShooterSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}