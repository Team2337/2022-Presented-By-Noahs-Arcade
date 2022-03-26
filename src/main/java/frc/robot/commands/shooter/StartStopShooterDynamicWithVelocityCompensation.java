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
public class StartStopShooterDynamicWithVelocityCompensation extends CommandBase {

  private final Supplier<Translation2d> translationSupplier;
  private final Shooter shooter;

  private static final double kLowGoalThresholdInches = 72;
  private static final double ringOfFireTopEnd = 180;
  private static final double launchPadTopEnd = 202;
  private static final double extremeTopEnd = 250;
  private static final double kLowGoalSpeedFeetPerSecond = 19.25;
  private static final double kHighGoalSpeedFeetPerSecond = 38.5;
  private static final double kLaunchpadCloseSpeedFeetPerSecond = 43.7;
  private static final double kLaunchpadFarSpeedFeetPerSecond = 48;

  Double previousTime;
  PolarCoordinate previousRobotCoordinate;

  public StartStopShooterDynamicWithVelocityCompensation(Supplier<Translation2d> translationSupplier, Shooter shooter) {
    this.translationSupplier = translationSupplier;

    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    previousTime = 0.0;
    previousRobotCoordinate = PolarCoordinate.fromFieldCoordinate(translationSupplier.get());
  }

  @Override
  public void execute() {
    PolarCoordinate robotCoordinate = PolarCoordinate.fromFieldCoordinate(translationSupplier.get());
    double time = Timer.getFPGATimestamp();
    double timeDelta = time - previousTime;

    double deltaDistanceMeters = robotCoordinate.getRadiusMeters() - previousRobotCoordinate.getRadiusMeters();
    double velocity = Units.metersToFeet(deltaDistanceMeters) / timeDelta;

    double distanceInches = Units.metersToInches(robotCoordinate.getRadiusMeters());
    SmartDashboard.putNumber("Distance", distanceInches);

    double shooterSpeed = 47.8 - 0.169 * distanceInches + 0.00108 * Math.pow(distanceInches, 2);
    double shooterXSpeed = Math.cos(Units.degreesToRadians(25)) * shooterSpeed;
    double shooterYSpeed = Math.sin(Units.degreesToRadians(25)) * shooterSpeed;
    shooterXSpeed -= velocity;
    shooterSpeed = Math.sqrt(Math.pow(shooterXSpeed, 2) + Math.pow(shooterYSpeed, 2));

    SmartDashboard.putNumber("Shooter speed", shooterSpeed);
    SmartDashboard.putNumber("Velocity", velocity);

    shooter.setSpeed(shooterSpeed);

    previousTime = time;
    previousRobotCoordinate = robotCoordinate;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

}