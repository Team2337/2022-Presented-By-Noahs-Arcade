package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class PerpetualConditionalBloopShoot extends ConditionalCommand {

  public PerpetualConditionalBloopShoot(Supplier<Boolean> redSwitchLeftStatus, Climber climber, Delivery delivery, Kicker kicker, Shooter shooter) {
    super (
      new PerpetualBloopOperatorLinearShoot(delivery, kicker, shooter).withTimeout(0.5),
      new WaitCommand(0.0),
      () -> {
        return (DriverStation.getAlliance().toString() != delivery.getLeftColorSensorAllianceBallColor() && delivery.getLeftColorSensorAllianceBallColor() != "null" && !redSwitchLeftStatus.get());
      }
    );
  }

}