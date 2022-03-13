package frc.robot.commands.delivery.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.delivery.BallToSideCommand;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;

public class AfterIntakeCommandGroup extends SequentialCommandGroup {

  public AfterIntakeCommandGroup(Intake intake, Delivery delivery, Kicker kicker) {
    // Schedule commands
    addCommands(
      new InstantCommand(delivery::addBall, delivery),
      new BallToSideCommand(delivery, kicker)
    );
  }

}