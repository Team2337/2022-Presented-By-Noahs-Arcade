package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.delivery.BottomToTopCommand;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Kicker;


public class RecenterWithKicker extends SequentialCommandGroup {

  public RecenterWithKicker(Kicker kicker, Delivery delivery) {
    // Schedule commands
    addCommands(
      new BottomToTopCommand(delivery, kicker).withTimeout(5),
      new AutoKickerCommand(kicker, 0).withTimeout(0.075),
      new WaitCommand(.1),
      new AutoKickerReverse(kicker, 0).withTimeout(0.15)

    );
  }
  
}