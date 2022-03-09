package frc.robot.commands.kicker;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.delivery.BottomToTopCommand;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Kicker;


public class RecenterWithKickerCommand extends SequentialCommandGroup {

  public RecenterWithKickerCommand(Delivery delivery, Kicker kicker) {
    addCommands(
      new BottomToTopCommand(delivery),
      new ForwardKickerCommand(kicker).withTimeout(0.1),
      new WaitCommand(.1),
      new ReverseKickerCommand(kicker).withTimeout(0.15)
    );
  }

}