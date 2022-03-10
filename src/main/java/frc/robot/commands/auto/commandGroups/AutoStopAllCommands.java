package frc.robot.commands.auto.commandGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.delivery.AutoStopDelivery;
import frc.robot.commands.intake.AutoStopIntake;
import frc.robot.commands.shooter.AutoStopShooter;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class AutoStopAllCommands extends ParallelCommandGroup {

  public AutoStopAllCommands(Delivery delivery, Intake intake, Kicker kicker, Shooter shooter) {
    addCommands(
      new AutoStopDelivery(delivery),
      new AutoStopIntake(intake),
      new AutoStopShooter(shooter),
      new InstantCommand(kicker::stop, kicker)
    );

  }
}