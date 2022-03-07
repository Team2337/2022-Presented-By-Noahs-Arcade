package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.delivery.BottomToSideCommand;
import frc.robot.commands.delivery.StartDelivery;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Kicker;

public class LinearShoot extends SequentialCommandGroup {

  public LinearShoot(double speed, Shooter shooter, Delivery delivery, Kicker kicker) {
    // Schedule commands
    addCommands(
      new InstantCommand(() -> shooter.setSpeed(speed)),
      new InstantCommand(() -> kicker.start(0.5)),
      new StartDelivery(delivery).withTimeout(1)
    );
  }
  
}