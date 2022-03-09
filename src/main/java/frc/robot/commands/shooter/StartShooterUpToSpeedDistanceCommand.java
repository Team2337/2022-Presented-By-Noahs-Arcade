package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.Shooter;

/**
 * Start the shooter at low/high speed based on the distance away from the Hub. Command will
 * end when the shooter has come up to speed.
 */
public class StartShooterUpToSpeedDistanceCommand extends ConditionalCommand {

  // Set the shooter speed to the low goal speed if we're
  // kLowGoalThresholdInches or less from the goal
  private static final double kLowGoalThresholdInches = 12;
  private static final double kLowGoalSpeedFeetPerSecond = 19.25;
  private static final double kHighGoalSpeedFeetPerSecond = 38.5;

  public StartShooterUpToSpeedDistanceCommand(Supplier<Translation2d> translationSupplier, Shooter shooter) {
    super(
      new StartShooterUpToSpeedCommand(kLowGoalSpeedFeetPerSecond, shooter),
      new StartShooterUpToSpeedCommand(kHighGoalSpeedFeetPerSecond, shooter),
      () -> {
        PolarCoordinate robotCoordinate = PolarCoordinate.fromFieldCoordinate(translationSupplier.get());
        return Units.metersToInches(robotCoordinate.getRadiusMeters()) < kLowGoalThresholdInches;
      }
    );
  }

}
