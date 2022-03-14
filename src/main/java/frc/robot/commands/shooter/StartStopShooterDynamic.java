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

  private final Shooter shooter;
  private PolarCoordinate robotCoordinate;
  private Supplier<Translation2d> translationSupplier; 

  private static final double kLowGoalThresholdInches = 72;
  private static final double ringOfFireTopEnd = 180;
  private static final double launchPadTopEnd = 202;
  private static final double extremeTopEnd = 250;
  private static final double kLowGoalSpeedFeetPerSecond = 19.25;
  private static final double kHighGoalSpeedFeetPerSecond = 38.5;
  private static final double kLaunchpadCloseSpeedFeetPerSecond = 43.7;
  private static final double kLaunchpadFarSpeedFeetPerSecond = 48;
  private double lastDistance;
  private double distance;
  private double lastTime;
  private double time;  
  private double deltaTime;
  private double deltaDistance;


  public StartStopShooterDynamic(Supplier<Translation2d> translationSupplier, Shooter shooter) {
    this.shooter = shooter;
    this.translationSupplier = translationSupplier;

    robotCoordinate = PolarCoordinate.fromFieldCoordinate(translationSupplier.get());
    
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    robotCoordinate = PolarCoordinate.fromFieldCoordinate(translationSupplier.get());

    lastDistance = distance;
    lastTime = time;
    distance = Units.metersToInches(robotCoordinate.getRadiusMeters());
    time = Timer.getFPGATimestamp();
    deltaTime = lastTime - time;
    double velocity = Utilities.calculateDerivative(distance, lastDistance, deltaTime);
    deltaDistance = (lastDistance - distance);
    SmartDashboard.putNumber("Velocity", velocity);
    SmartDashboard.putNumber("Change in distance", (lastDistance - distance));
    double shooterSpeed = 36.1 - 0.0114 * distance + 0.00027 * Math.pow(distance, 2);
    shooterSpeed = shooterSpeed - deltaDistance;
    SmartDashboard.putNumber("Distance", distance);
    shooter.setSpeed(shooterSpeed);
    SmartDashboard.putNumber("Shooter speed", shooterSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

}