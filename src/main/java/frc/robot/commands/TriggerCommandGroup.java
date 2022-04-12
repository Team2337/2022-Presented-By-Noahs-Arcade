package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.delivery.BottomToTopCommand;
import frc.robot.commands.delivery.ReverseDelivery;
import frc.robot.subsystems.Delivery;

public class TriggerCommandGroup extends ParallelCommandGroup {

  public TriggerCommandGroup(Supplier<Boolean> isUpToSpeed, Supplier<String> centerColorSensorStatus, XboxController controller, Delivery delivery) {
    addCommands(
      new Rumble(controller).withTimeout(0.5),
      new BottomToTopCommand(delivery).andThen(new ReverseDelivery(centerColorSensorStatus, 0.2, delivery))
    );
  }

}