package frc.robot.commands.shooter;

import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Runs the shooter at a given speed. Stops the shooter when the command ends.
 *
 * @author Nicholas S.
 */
public class StartStopShooterDynamicCommand extends CommandBase {

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


  public StartStopShooterDynamicCommand(Supplier<Translation2d> translationSupplier, Shooter shooter) {
    this.shooter = shooter;
    this.translationSupplier = translationSupplier;

    robotCoordinate = PolarCoordinate.fromFieldCoordinate(translationSupplier.get());
    
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    robotCoordinate = PolarCoordinate.fromFieldCoordinate(translationSupplier.get());

    double distance = Units.metersToInches(robotCoordinate.getRadiusMeters());
    SmartDashboard.putNumber("Distance", distance);
    if (distance < kLowGoalThresholdInches) {
      shooter.setSpeed(kLowGoalSpeedFeetPerSecond);
    } else if (distance < ringOfFireTopEnd &&  distance > kLowGoalThresholdInches) {
      shooter.setSpeed(kHighGoalSpeedFeetPerSecond);
    } else if (distance < launchPadTopEnd &&  distance > ringOfFireTopEnd) {
      shooter.setSpeed(kLaunchpadCloseSpeedFeetPerSecond);
    } else if (distance < extremeTopEnd &&  distance > launchPadTopEnd) { 
      shooter.setSpeed(kLaunchpadFarSpeedFeetPerSecond);
    } else {
      shooter.setSpeed(kHighGoalSpeedFeetPerSecond);
    }

  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

}