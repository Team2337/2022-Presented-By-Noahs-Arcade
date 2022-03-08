package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.delivery.BottomToTopCommand;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Kicker;


public class RecenterWithKicker extends SequentialCommandGroup {

  public RecenterWithKicker(Delivery delivery, Kicker kicker) {
    addCommands(
      new BottomToTopCommand(delivery),
      new AutoKickerCommand(kicker).withTimeout(0.1),
      new WaitCommand(.1),
      new AutoKickerReverse(kicker).withTimeout(0.15)
    );
  }

}