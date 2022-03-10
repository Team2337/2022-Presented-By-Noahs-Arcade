package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the shooter at a given speed. Stops the shooter when the command ends.
 *
 * @author Nicholas S.
 */
public class StartStopShooterCommand extends CommandBase {

  private final double speedFeetPerSecond;
  private final Shooter shooter;

  public StartStopShooterCommand(double speedFeetPerSecond, Shooter shooter) {
    this.speedFeetPerSecond = speedFeetPerSecond;
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void execute() {
    shooter.setSpeed(speedFeetPerSecond);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

}