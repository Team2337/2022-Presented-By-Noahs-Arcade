package frc.robot.commands.delivery.commandgroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.delivery.*;
import frc.robot.commands.shooter.StartShooterUpToSpeedCommand;
import frc.robot.Constants.BallColor;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class PrepareShooterCommandGroup extends SequentialCommandGroup {

  public PrepareShooterCommandGroup(BallColor ballColor, Delivery delivery, Kicker kicker, Shooter shooter) {
    addCommands(
      new ConditionalCommand(
        sequence(
          new SideToTopCommand(ballColor, delivery, kicker),
          new StartShooterUpToSpeedCommand(48.0, shooter)
          // number taken from the far distance for the very long-named StartShooterUpToSpeedDistanceCommand
          // Should probably use that instead but oh well
        ),
        new InstantCommand(),
        () -> (delivery.getNumberOfBalls() > 0)
      )
    );
  }
}
