package frc.robot.commands.shooter;

import java.util.concurrent.ScheduledExecutorService;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Vision;
import frc.robot.commands.kicker.ReverseKickerCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Kicker;

public class PrepareShooter extends SequentialCommandGroup {

  public PrepareShooter(Supplier<Translation2d> translationSupplier, Supplier<Boolean> overrideSupplier, Supplier<Double> distanceSupplier, Kicker kicker, Shooter shooter) {
    addCommands(
      new ReverseStopShooterCommand(shooter).withTimeout(0.2),
      new ReverseKickerCommand(kicker).withTimeout(0.2),
     // new ScheduleCommand(new StartShooterUpToSpeedDistanceCommand(translationSupplier, overrideSupplier, shooter))
     new ScheduleCommand(new StartStopShooterDynamic(distanceSupplier, overrideSupplier, shooter))
     //};
    );
  }
}