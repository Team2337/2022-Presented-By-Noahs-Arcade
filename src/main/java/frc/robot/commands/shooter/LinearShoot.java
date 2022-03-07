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
  private final Shooter shooter;
  private final Kicker kicker;
  private final Delivery delivery;
  private double speed;
  //Speed is in ft/s
  public LinearShoot(double speed, Delivery delivery, Kicker kicker, Shooter shooter) {
    this.shooter = shooter;
    this.kicker = kicker;
    this.delivery = delivery;
    this.speed = speed;

    addCommands(
      new QuickStartShooter(speed, shooter),
      new InstantCommand(() -> kicker.start(0.5)),
      new StartDelivery(delivery)
    );
  }

  @Override
  public void end(boolean interrupted) {
    new ParallelCommandGroup(
        new AutoStopKicker(kicker),
        new AutoStopShooter(shooter),
        new AutoStopDelivery(delivery)
      );
  }

  @Override
  public boolean isFinished() {
    return false;
  }
  
}