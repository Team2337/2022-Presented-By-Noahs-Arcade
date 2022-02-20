package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BallColor;
import frc.robot.subsystems.Delivery;

public class PrepareShooterCommandGroup extends SequentialCommandGroup {
  
  public PrepareShooterCommandGroup(BallColor ballColor, Delivery delivery) {
    // If there are issues, schedule a different one first
    if (delivery.hasIssues()) {
      CommandScheduler.getInstance().schedule(false, new CorrectDeliveryCommandGroup(delivery));
    }

    // Check which way we need to rotate
    if (delivery.getTopPositionColor() == ballColor) {
      // Ball is at top
      addCommands(
        new LineupTopCommand(delivery)
      );
    } else if (delivery.getBottomPositionColor() == ballColor){
      // Ball is at bottom
      addCommands(
        new BottomToSideCommand(delivery),
        new SideToTopCommand(delivery, ballColor),
        new LineupTopCommand(delivery)
      );
    } else {
      // Ball is at side
      addCommands(
        new SideToTopCommand(delivery, ballColor),
        new LineupTopCommand(delivery)
      );
    }
  }
}
