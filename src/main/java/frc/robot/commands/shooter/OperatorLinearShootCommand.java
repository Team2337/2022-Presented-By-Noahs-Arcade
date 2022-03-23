package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.delivery.StartDelivery;
import frc.robot.commands.kicker.ForwardKickerCommand;
import frc.robot.commands.kicker.ReverseKickerCommand;
import frc.robot.commands.kicker.StartKicker;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Kicker;

public class OperatorLinearShootCommand extends SequentialCommandGroup {

  public OperatorLinearShootCommand(Supplier<Translation2d> translationSupplier, Supplier<Boolean> overrideSupplier, Delivery delivery, Kicker kicker, Shooter shooter) {
    addCommands(
      new ReverseStopShooterCommand(shooter).withTimeout(0.15),
      //new ReverseKickerCommand(kicker).withTimeout(0.15),
      new WaitCommand(0.15),
      new StartShooterInstantCommand(19.5, shooter),
      new StartKicker(kicker),
      new StartDelivery(delivery)
    );
  }

}