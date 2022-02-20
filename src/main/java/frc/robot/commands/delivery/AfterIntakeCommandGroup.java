package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;

public class AfterIntakeCommandGroup extends SequentialCommandGroup {

  public AfterIntakeCommandGroup(Intake intake, Delivery delivery) {
    // If there are issues, schedule a different one first
    if (delivery.hasIssues()) {
      CommandScheduler.getInstance().schedule(false, new CorrectDeliveryCommandGroup(delivery));
    }

    // Schedule commands
    addCommands(
      new InstantCommand(delivery::addNewBall),
      new BottomToSideCommand(delivery)
    );
  }
  
}
