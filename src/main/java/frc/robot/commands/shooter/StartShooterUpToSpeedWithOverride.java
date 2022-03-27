package frc.robot.commands.shooter;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.Shooter;

/**
 * Start the shooter at a dynamic speed unless the yellow switch is true. Command will
 * end when the shooter has come up to speed.
 */
public class StartShooterUpToSpeedWithOverride extends ConditionalCommand {

  private static final double kHighGoalSpeedFeetPerSecond = 39.5;
  private static final double kLaunchpadCloseSpeedFeetPerSecond = 47.5;

  public StartShooterUpToSpeedWithOverride(Supplier<Double> distanceSupplier, Supplier<Boolean> overrideSupplier, Supplier<Boolean> yellowSwitch, Shooter shooter) {
    super(
      new StartShooterUpToSpeedCommand(kHighGoalSpeedFeetPerSecond, kLaunchpadCloseSpeedFeetPerSecond, overrideSupplier, shooter),
      new StartStopShooterDynamic(distanceSupplier, overrideSupplier, shooter),
      () -> {
        // if true, runs the StartShooterUpToSpeedCommand
        return yellowSwitch.get();
      }
    );
  }

}
