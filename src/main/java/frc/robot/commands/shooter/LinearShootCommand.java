package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.delivery.StartDelivery;
import frc.robot.commands.kicker.ForwardKickerCommand;
import frc.robot.commands.kicker.ReverseKickerCommand;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Kicker;

public class LinearShootCommand extends SequentialCommandGroup {

  public LinearShootCommand(double speedFeetPerSecond, Delivery delivery, Kicker kicker, Shooter shooter) {
    addCommands(
      new ReverseStopShooterCommand(shooter).withTimeout(0.2),
      new ReverseKickerCommand(kicker).withTimeout(0.2),
      new WaitCommand(0.2),
      new StartShooterUpToSpeedCommand(speedFeetPerSecond, shooter),
      new ForwardKickerCommand(kicker),
      new StartDelivery(delivery)
    );
  }

}