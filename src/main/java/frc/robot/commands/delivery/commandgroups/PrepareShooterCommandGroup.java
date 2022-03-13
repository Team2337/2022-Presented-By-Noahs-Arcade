package frc.robot.commands.delivery.commandgroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.delivery.*;
import frc.robot.commands.kicker.ForwardKickerCommand;
import frc.robot.commands.shooter.StartShooterUpToSpeedCommand;
import frc.robot.commands.shooter.StopShooterInstantCommand;
import frc.robot.Constants.BallColor;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class PrepareShooterCommandGroup extends SequentialCommandGroup {

  public PrepareShooterCommandGroup(BallColor ballColor, Delivery delivery, Kicker kicker, Shooter shooter) {
    addCommands(
      new ConditionalCommand(
        sequence(
          // Move the ball up to top
          new SideToTopCommand(ballColor, delivery, kicker),
          // Shoot ball
          new StartShooterUpToSpeedCommand(48.0, shooter), // far distance from StartShooterUpToSpeedDistanceCommand
          new ForwardKickerCommand(kicker),
          new WaitCommand(0.5),
          // Stop shooter
          new InstantCommand(kicker::stop, kicker),
          new StopShooterInstantCommand(shooter),
          new InstantCommand(delivery::removeBall),
          // Move other ball back so it is out of the way of intake if we have another one
          new ConditionalCommand(
            new BallToSideCommand(delivery, kicker),
            new InstantCommand(), // do nothing
            () -> (delivery.getNumberOfBalls() > 0)
          )
        ),
        new InstantCommand(), // do nothing
        () -> (delivery.getNumberOfBalls() > 0)
      )
    );
  }
}
