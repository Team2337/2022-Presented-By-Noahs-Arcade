package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.delivery.StartDelivery;
import frc.robot.commands.kicker.StartKicker;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Kicker;

public class PerpetualBloopOperatorLinearShoot extends SequentialCommandGroup {

  public PerpetualBloopOperatorLinearShoot(Delivery delivery, Kicker kicker, Shooter shooter) {
    addCommands(
        new ReverseStopShooterCommand(shooter).withTimeout(0.15),
        new WaitCommand(0.1),
        new StartShooterInstantCommand(19.5, shooter),
        new StartKicker(kicker),
        new StartDelivery(delivery)
        );
  }

}