package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.delivery.AutoStopDelivery;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class StopAllShooterSystemsCommand extends ParallelCommandGroup {

  public StopAllShooterSystemsCommand(Delivery delivery, Kicker kicker, Shooter shooter) {
    addCommands(
      new AutoStopDelivery(delivery),
      new AutoStopShooter(shooter),
      new AutoStopKicker(kicker)
    );
  }

}