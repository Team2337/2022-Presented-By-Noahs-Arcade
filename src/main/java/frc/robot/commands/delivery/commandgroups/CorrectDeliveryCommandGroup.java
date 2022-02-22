package frc.robot.commands.delivery.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.delivery.BottomToSideCommand;
import frc.robot.commands.delivery.SpinUntilSideSeesBallCommand;
import frc.robot.subsystems.Delivery;

/**
 * Fixes inconsistencies in the {@link Delivery} subsystem.
 * 
 * Only called when {@code delivery.hasIssues()} returns true in either the
 * {@link AfterIntakeCommandGroup} or {@link PrepareShooterCommandGroup}
 * 
 * @author Michael F
 */
public class CorrectDeliveryCommandGroup extends SequentialCommandGroup {

  public CorrectDeliveryCommandGroup(Delivery delivery) {
    addCommands(
      race(
        new SpinUntilSideSeesBallCommand(delivery),
        new WaitCommand(3) //FIXME: is this value too high?
      ),
      new InstantCommand(delivery::stop), // this might be redundant
      new InstantCommand(delivery::resetArray),
      new ConditionalCommand(
        new BottomToSideCommand(delivery),
        new InstantCommand(),
        () -> delivery.getNumberOfBalls() == 2
      )
    );
  }
}