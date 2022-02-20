package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.ProfiledPointToPointCommand;
import frc.robot.commands.delivery.AutoStartDelivery;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class Test extends SequentialCommandGroup {

  private Drivetrain drivetrain;
  private Delivery delivery;

  public Test(AutoDrive autoDrive, Delivery delivery, Drivetrain drivetrain, Heading heading) {
    this.drivetrain = drivetrain;
    this.delivery = delivery;

    addCommands(
      new AutoStartDelivery(delivery).withTimeout(0.6)
    );
  }
}