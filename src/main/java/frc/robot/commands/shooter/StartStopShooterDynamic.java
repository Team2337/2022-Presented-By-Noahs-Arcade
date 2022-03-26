package frc.robot.commands.shooter;

import frc.robot.coordinates.PolarCoordinate;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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

  private static final double kLowGoalThresholdInches = 72;
  private static final double ringOfFireTopEnd = 180;
  private static final double launchPadTopEnd = 202;
  private static final double extremeTopEnd = 250;
  private static final double kLowGoalSpeedFeetPerSecond = 19.25;
  private static final double kHighGoalSpeedFeetPerSecond = 38.5;
  private static final double kLaunchpadCloseSpeedFeetPerSecond = 47.5;
  private static final double kLaunchpadFarSpeedFeetPerSecond = 48;

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
      shooterSpeed = 44.6 - 0.169 * distanceInches + 0.00108 * Math.pow(distanceInches, 2);
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