package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.delivery.StartDelivery;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Kicker;

public class LinearShootCommand extends SequentialCommandGroup {

  public LinearShootCommand(double speedFeetPerSecond, Delivery delivery, Kicker kicker, Shooter shooter) {
    addCommands(
      new ReverseShooter(shooter).withTimeout(0.2),
      new ReverseKicker(kicker).withTimeout(0.2),
      new QuickStartShooterCommand(speedFeetPerSecond, shooter),
      new InstantCommand(() -> kicker.start(0.5), kicker),
      new StartDelivery(delivery)
    );
  }

}