package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Start the shooter at a given speed. Command will finish when the shooter is up
 * to speed. Command will not stop the shooter.
 *
 * @author Nicholas S.
 */
public class StartShooterUpToSpeedCommand extends CommandBase {

  private final double speedFeetPerSecond;
  private final double overrideSpeedFeetPerSecond;
  private final Supplier<Boolean>overrideSupplier;
  private final Shooter shooter;

  public StartShooterUpToSpeedCommand(double speedFeetPerSecond, Shooter shooter) {
    this(
      speedFeetPerSecond,
      0.0,
      () -> false,
      shooter
    );
  }

  public StartShooterUpToSpeedCommand(double speedFeetPerSecond, double overrideSpeedFeetPerSecond, Supplier<Boolean> overrideSupplier, Shooter shooter) {
    this.speedFeetPerSecond = speedFeetPerSecond;
    this.overrideSpeedFeetPerSecond = overrideSpeedFeetPerSecond;
    this.overrideSupplier = overrideSupplier;
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setSpeed(overrideSupplier.get() ? overrideSpeedFeetPerSecond : speedFeetPerSecond);
  }

  @Override
  public boolean isFinished() {
    return shooter.isShooterToSpeed();
  }

}