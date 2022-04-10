package frc.robot.commands.shooter;

import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the shooter at a given speed. Stops the shooter when the command ends.
 *
 * @author Nicholas S.
 */
public class StartStopShooterDynamic extends CommandBase {

  private final Supplier<Double> distanceSupplier;
  private final Supplier<Boolean> overrideSupplier;
  private final Shooter shooter;

  private static final double kHighGoalSpeedFeetPerSecond = 38.5;
  private static final double kLaunchpadCloseSpeedFeetPerSecond = 47.5;
  // Modify to increase or decrease the speed of the shooter
  // private static double yInterceptModification = 49.8;
  private static double yInterceptModification = 32.6;

  Double previousTime;
  PolarCoordinate previousRobotCoordinate;

  public StartStopShooterDynamic(Supplier<Double> distanceSupplier,Supplier<Boolean> overrideSupplier, Shooter shooter) {
    this.distanceSupplier = distanceSupplier;
    this.overrideSupplier = overrideSupplier;

    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    double distanceInches = distanceSupplier.get();
    SmartDashboard.putNumber("Distance", distanceInches);
    double shooterSpeed;

    if (distanceInches == 0) {
      shooterSpeed = kHighGoalSpeedFeetPerSecond;
    } else {
      // shooterSpeed = yInterceptModification - 0.169 * distanceInches + 0.00108 * Math.pow(distanceInches, 2);
      // shooterSpeed = yInterceptModification - 0.219 * distanceInches + 0.00118 * Math.pow(distanceInches, 2);
      shooterSpeed = yInterceptModification + 0.0402 * distanceInches + 0.000221 * Math.pow(distanceInches, 2);
    }

    SmartDashboard.putNumber("Shooter speed", shooterSpeed);
    if (overrideSupplier.get()) {
      shooter.setSpeed(kLaunchpadCloseSpeedFeetPerSecond);
    } else {
       shooter.setSpeed(shooterSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}