package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.climber.PerpetualClimberBloopShoot;
import frc.robot.commands.delivery.PerpetualBloopDelivery;
import frc.robot.commands.kicker.PerpetualBloopKicker;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Kicker;

public class PerpetualBloopShoot extends ParallelCommandGroup {

  public PerpetualBloopShoot(Climber climber, Delivery delivery, Kicker kicker) {
    addCommands(
      new PerpetualBloopKicker(climber::isAtSetpoint, delivery::getLeftColorSensorAllianceBallColor, kicker),
      new PerpetualBloopDelivery(climber::isAtSetpoint, delivery::getLeftColorSensorAllianceBallColor, delivery),
      new PerpetualClimberBloopShoot(delivery::getRightColorSensorAllianceBallColor, delivery::getLeftColorSensorAllianceBallColor, climber)
    );
  }

}