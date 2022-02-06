package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BallColor;
import frc.robot.subsystems.Delivery;

public class PrepareShooterCommandGroup extends SequentialCommandGroup {
  
  public PrepareShooterCommandGroup(BallColor ballColor, Delivery delivery) {
    // Check which way we need to rotate
    if (delivery.getTopPositionColor() == ballColor) {
      addCommands(
        new LineupTopCommand(delivery)
      );
    } else if (delivery.getBottomPositionColor() == ballColor){
      addCommands(
        new BottomToSideCommand(delivery),
        new SideToTopCommand(delivery, ballColor),
        new LineupTopCommand(delivery)
      );
    } else {
      addCommands(
        new SideToTopCommand(delivery, ballColor),
        new LineupTopCommand(delivery)
      );
    }
  }
}
