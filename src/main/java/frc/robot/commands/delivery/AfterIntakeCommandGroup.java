package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;

public class AfterIntakeCommandGroup extends SequentialCommandGroup {

  public AfterIntakeCommandGroup(Intake intake, Delivery delivery) {
    addCommands(
      new InstantCommand(delivery::addNewBall),
      new BottomToSideCommand(delivery)
    );
  }
  
}
