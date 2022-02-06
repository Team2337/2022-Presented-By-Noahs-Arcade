package frc.robot.commands.delivery;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;

public class IntakeBallCommandGroup extends SequentialCommandGroup {

  public IntakeBallCommandGroup(Intake intake, Delivery delivery) {
    addCommands(
      new IntakeCommand(intake, delivery),
      new WaitCommand(0.5),
      new InstantCommand(delivery::addNewBall),
      new BottomToSideCommand(delivery)
    );
  }
  
}
