package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.delivery.StartDelivery;
import frc.robot.commands.kicker.StartKicker;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class Shoot extends SequentialCommandGroup {

  public Shoot(Delivery delivery, Kicker kicker) {
    addCommands(
      //new ShooterUpToSpeedCommand(shooter),
      new StartKicker(kicker),
      new StartDelivery(delivery)
    );
  }

}