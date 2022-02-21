package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BallColor;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Kicker;

public class PrepareShooterCommandGroup extends SequentialCommandGroup {
  
  public PrepareShooterCommandGroup(BallColor ballColor, Kicker kicker, Delivery delivery) {
    // Check which way we need to rotate
    if (delivery.getTopPositionColor() == ballColor) {
      // Ball is at top
      addCommands(
        new InstantCommand(kicker::slowReverse),
        new LineupTopCommand(delivery),
        new InstantCommand(kicker::stop)
      );
    } else if (delivery.getBottomPositionColor() == ballColor){
      // Ball is at bottom
      addCommands(
        new BottomToSideCommand(delivery),
        new InstantCommand(kicker::slowReverse),
        new SideToTopCommand(delivery, ballColor),
        new LineupTopCommand(delivery),
        new InstantCommand(kicker::stop)
      );
    } else {
      // Ball is at side
      addCommands(
        new InstantCommand(kicker::slowReverse),
        new SideToTopCommand(delivery, ballColor),
        new LineupTopCommand(delivery),
        new InstantCommand(kicker::stop)
      );
    }
  }
}
