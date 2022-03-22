package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.kicker.ReverseKickerCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Kicker;

public class PrepareShooter extends SequentialCommandGroup {

  public PrepareShooter(Supplier<Translation2d> translationSupplier, Supplier<Boolean> overrideSupplier, Kicker kicker, Shooter shooter) {
    addCommands(
      new ReverseStopShooterCommand(shooter).withTimeout(0.2),
      new ReverseKickerCommand(kicker).withTimeout(0.2),
      new WaitCommand(0.4),
      new StartShooterUpToSpeedDistanceCommand(translationSupplier, overrideSupplier, shooter)
    );
  }
}