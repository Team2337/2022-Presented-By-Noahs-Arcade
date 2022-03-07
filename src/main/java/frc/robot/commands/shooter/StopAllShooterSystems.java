package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class StopAllShooterSystems extends ParallelCommandGroup {

  public StopAllShooterSystems(Delivery delivery, Kicker kicker, Shooter shooter) {
    addCommands(
    new InstantCommand(delivery::stopDelivery, delivery),
    new InstantCommand(shooter::stop, shooter),
    new InstantCommand(kicker::stop, kicker)
  );

  }
}