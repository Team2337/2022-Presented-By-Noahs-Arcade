package frc.robot.commands.delivery.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.delivery.*;
import frc.robot.Constants.BallColor;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Kicker;

public class PrepareShooterCommandGroup extends SequentialCommandGroup {

  public PrepareShooterCommandGroup(BallColor ballColor, Delivery delivery, Kicker kicker) {
    // Check which way we need to rotate
    if (delivery.getTopPositionColor() == ballColor) {
      // Ball is at top
      return; // remove this if we want to call a shooter command after
    } else if (delivery.getBottomPositionColor() == ballColor){
      // Ball is at bottom
      addCommands(
        new BottomToSideCommand(delivery, kicker),
        new SideToTopCommand(ballColor, delivery, kicker)
      );
    } else {
      // Ball is at side
      addCommands(
        new SideToTopCommand(ballColor, delivery, kicker)
      );
    }
  }
}
