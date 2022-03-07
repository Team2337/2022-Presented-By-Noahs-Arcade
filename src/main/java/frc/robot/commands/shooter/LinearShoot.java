package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.delivery.StartDelivery;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Kicker;

public class LinearShoot extends SequentialCommandGroup {
  public LinearShoot(double speedFeetPerSecond, Delivery delivery, Kicker kicker, Shooter shooter) {
    addCommands(
      new QuickStartShooter(speedFeetPerSecond, shooter),
      new InstantCommand(() -> kicker.start(0.5), kicker),
      new StartDelivery(delivery)
    );
  }
}