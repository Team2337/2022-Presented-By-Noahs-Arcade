package frc.robot.commands.delivery.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.delivery.BottomToSideCommand;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;

public class AfterIntakeCommandGroup extends SequentialCommandGroup {

  public AfterIntakeCommandGroup(Intake intake, Delivery delivery, Kicker kicker) {
    // Schedule commands
    addCommands(
      new InstantCommand(delivery::addNewBall),
      // new InstantCommand(intake::reverse),
      new BottomToSideCommand(delivery, kicker)
    );
  }
  
}