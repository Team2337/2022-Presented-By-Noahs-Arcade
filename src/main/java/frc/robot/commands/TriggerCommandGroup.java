package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.delivery.BottomToTopCommand;
import frc.robot.subsystems.Delivery;

public class TriggerCommandGroup extends ParallelCommandGroup {

  public TriggerCommandGroup(XboxController controller, Delivery delivery) {
    addCommands(
      new Rumble(controller).withTimeout(0.5),
      new BottomToTopCommand(delivery)
    );
  }

}