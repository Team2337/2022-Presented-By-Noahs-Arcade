package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.commandGroups.AutoStopAllCommands;
import frc.robot.commands.auto.commandGroups.FirstMove;
import frc.robot.commands.delivery.AutoStartDelivery;
import frc.robot.commands.kicker.ForwardKickerCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class Pos3RightFiveBall extends SequentialCommandGroup {

  public Pos3RightFiveBall(AutoDrive autoDrive, Delivery delivery, Drivetrain drivetrain, Heading heading, Intake intake, Kicker kicker, Shooter shooter) {
    addCommands(
      new FirstMove(Constants.Auto.kBallR3RunOver, autoDrive, drivetrain, heading, intake, shooter),
      new ForwardKickerCommand(kicker).withTimeout(0.5),
      new ParallelCommandGroup(
        new ProfiledPointToPointCommand(Constants.Auto.kBallR2Pickup, drivetrain::getTranslation, 3.0, 0.05, Units.inchesToMeters(120), 12, autoDrive, heading).withTimeout(3),
        new AutoStartDelivery(delivery).withTimeout(0.75)
      ),
      new ForwardKickerCommand(kicker).withTimeout(0.5),
      new ProfiledPointToPointCommand(Constants.Auto.kBallR2ShootPosition, drivetrain::getTranslation, 3.0, 0.05, Units.inchesToMeters(120), 15, autoDrive, heading).withTimeout(1),
      new ParallelCommandGroup(
        new ForwardKickerCommand(kicker).withTimeout(1.5),
        new AutoStartDelivery(delivery).withTimeout(1.5)
      ),
      new ProfiledPointToPointCommand(Constants.Auto.kBallR4Pickup, drivetrain::getTranslation, 3.0, 0.025, Units.inchesToMeters(120), 8, autoDrive, heading).withTimeout(4),
      new ProfiledPointToPointCommand(Constants.Auto.kBallR4RunOver, drivetrain::getTranslation, 3.0, 0.025, Units.inchesToMeters(120), 8, autoDrive, heading).withTimeout(4),
      new WaitCommand(2),
      new ParallelCommandGroup(
        new ProfiledPointToPointCommand(Constants.Auto.kFiveBallShootPosition, drivetrain::getTranslation, 3.0, 0.01, Units.inchesToMeters(120), 8, autoDrive, heading).withTimeout(4),
        new AutoStartDelivery(delivery).withTimeout(0.75)
      ),
      new ForwardKickerCommand(kicker).withTimeout(2),
      new WaitCommand(3),
      new WaitCommand(5),
      new ProfiledPointToPointCommand(Constants.Auto.kPosition3RightStart, drivetrain::getTranslation, 1.0, 0.05, Units.inchesToMeters(120), 8, autoDrive, heading).withTimeout(3),
      new AutoStopAllCommands(delivery, intake, kicker, shooter)
    );
  }
}