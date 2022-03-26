package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.nerdyfiles.oi.NerdyOperatorStation;
import frc.robot.subsystems.Shooter;

/**
 * Start the shooter at low/high speed based on the distance away from the Hub. Command will
 * end when the shooter has come up to speed.
 */
public class StartShooterUpToSpeedWithOverride extends ConditionalCommand {

  // Set the shooter speed to the low goal speed if we're
  // kLowGoalThresholdInches or less from the goal
  private static final double kHighGoalSpeedFeetPerSecond = 39.5;
  private static final double kLaunchpadCloseSpeedFeetPerSecond = 47.5;

  public StartShooterUpToSpeedWithOverride(Supplier<Double> distanceSupplier, Supplier<Boolean> overrideSupplier, Supplier<Boolean> yellowSwitch, Shooter shooter) {
    super(
      new StartShooterUpToSpeedCommand(kHighGoalSpeedFeetPerSecond, kLaunchpadCloseSpeedFeetPerSecond, overrideSupplier, shooter),
      new StartStopShooterDynamic(distanceSupplier, overrideSupplier, shooter),
      () -> {
        return yellowSwitch.get();
      }
    );
  }

}
