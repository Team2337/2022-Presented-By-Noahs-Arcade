package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.delivery.AutoStartDelivery;
import frc.robot.commands.kicker.ForwardKickerCommand;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Kicker;

public class ParallelTest extends ParallelCommandGroup {
  
  public ParallelTest(Delivery delivery, Kicker kicker) {
    addCommands(
      new AutoStartDelivery(delivery).withTimeout(1),
      new ForwardKickerCommand(kicker).withTimeout(1)
    );
  }
}