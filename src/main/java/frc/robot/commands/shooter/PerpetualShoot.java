package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.delivery.PerpetualDelivery;
import frc.robot.commands.kicker.PerpetualKicker;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Kicker;

public class PerpetualShoot extends ParallelCommandGroup {

  public PerpetualShoot(Delivery delivery, Kicker kicker) {
    addCommands(
      new PerpetualKicker(delivery::getLeftColorSensorAllianceBallColor, kicker),
      new PerpetualDelivery(delivery)
    );
  }

}