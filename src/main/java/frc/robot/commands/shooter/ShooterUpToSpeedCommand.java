package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Start the shooter at a given speed. Command will finish when the shooter is up
 * to speed. Command will not stop the shooter.
 *
 * @author Nicholas S.
 */
public class ShooterUpToSpeedCommand extends CommandBase {

  private final Shooter shooter;

  public ShooterUpToSpeedCommand(Shooter shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
  }

  @Override
  public boolean isFinished() {
    return shooter.isShooterToSpeed();
  }

}