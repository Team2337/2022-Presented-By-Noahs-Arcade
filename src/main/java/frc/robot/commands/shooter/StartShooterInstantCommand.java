package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Start the shooter at a given speed. Speed should be in ft/s.
 *
 * @author Madison J.
 */
public class StartShooterInstantCommand extends InstantCommand {

  private final double speedFeetPerSecond;
  private final Shooter shooter;

  /**
   * Start the shooter at a given speed. Speed should be in ft/s.
   */
  public StartShooterInstantCommand(double speedFeetPerSecond, Shooter shooter) {
    this.speedFeetPerSecond = speedFeetPerSecond;
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setSpeed(speedFeetPerSecond);
  }
}