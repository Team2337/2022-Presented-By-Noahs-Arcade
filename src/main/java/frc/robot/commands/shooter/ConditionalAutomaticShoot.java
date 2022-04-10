package frc.robot.commands.shooter;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class ConditionalAutomaticShoot extends ConditionalCommand {

  public ConditionalAutomaticShoot(Supplier<Boolean> redSwitchLeftStatus, Climber climber, Delivery delivery, Kicker kicker, Shooter shooter) {
    super (
      new PerpetualBloopShoot(climber, delivery, kicker),
      new Shoot(delivery, kicker),
      () -> {
        return (!redSwitchLeftStatus.get());
      }
    );
  }

}