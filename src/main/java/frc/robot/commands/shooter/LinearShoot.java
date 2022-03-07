package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.delivery.AutoStopDelivery;
import frc.robot.commands.delivery.BottomToSideCommand;
import frc.robot.commands.delivery.StartDelivery;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Kicker;

public class LinearShoot extends SequentialCommandGroup {

  public LinearShoot(Delivery delivery, Kicker kicker, Shooter shooter, double speed) {
    // Schedule commands
    addCommands(
      new QuickStartShooter(shooter, speed),
      new InstantCommand(() -> kicker.start(0.5)),
      new StartDelivery(delivery).withTimeout(1),
      new ParallelCommandGroup(
        new AutoStopKicker(kicker),
        new AutoStopShooter(shooter),
        new AutoStopDelivery(delivery)
      )
    );
  }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return super.isFinished();
  }
  
}