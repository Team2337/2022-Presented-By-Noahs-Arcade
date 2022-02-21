package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Runs the shooter 
 *
 * @author Madison J.
 */
public class AutoStartShooter extends InstantCommand {

  private final Shooter shooter;
  private double shootSpeed;

  public AutoStartShooter(Shooter shooter, double shootSpeed) {
    this.shooter = shooter;
    this.shootSpeed = shootSpeed;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setSpeed(shootSpeed);
  }
}