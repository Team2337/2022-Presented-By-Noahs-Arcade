package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.delivery.AutoStopDelivery;
import frc.robot.commands.intake.AutoStopIntake;
import frc.robot.commands.shooter.AutoStopKicker;
import frc.robot.commands.shooter.AutoStopShooter;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class StopAllShooterSystems extends ParallelCommandGroup {

  public StopAllShooterSystems(Delivery delivery, Kicker kicker, Shooter shooter) {
    addCommands(
      new AutoStopDelivery(delivery),
      new AutoStopShooter(shooter),
      new AutoStopKicker(kicker)
    );

  }
}