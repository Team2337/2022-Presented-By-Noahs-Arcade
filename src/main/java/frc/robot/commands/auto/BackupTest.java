package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.commandGroups.AutoStopAllCommands;
import frc.robot.commands.auto.commandGroups.FirstMove;
import frc.robot.commands.delivery.AutoStartDelivery;
import frc.robot.commands.intake.AutoStartIntake;
import frc.robot.commands.kicker.ForwardKickerCommand;
import frc.robot.commands.shooter.AutoStartShooter;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class BackupTest extends SequentialCommandGroup {

  public BackupTest(AutoDrive autoDrive, Delivery delivery, Drivetrain drivetrain, Heading heading, Intake intake, Kicker kicker, Shooter shooter) {
    addCommands(
      // new FirstMove(Constants.Auto.kBallR3RunOver, autoDrive, drivetrain, heading, intake, shooter),
      new WaitCommand(3),
      new AutoStartIntake(intake),
      new ProfiledPointToPointCommand(Constants.Auto.kBallR3RunOver, drivetrain::getTranslation, 1.5, 0.05, Units.inchesToMeters(90), 8, autoDrive, heading).withTimeout(5),
      new WaitCommand(1),
      new AutoStartShooter(38.5, shooter),
      new WaitCommand(1),
      new ParallelTest(delivery, kicker),
      new AutoStartDelivery(delivery).withTimeout(1),
      new WaitCommand(1),
      new ForwardKickerCommand(kicker).withTimeout(1),
      new ProfiledPointToPointCommand(Constants.Auto.kPosition3RightStart, drivetrain::getTranslation, 1.5, 0.05, Units.inchesToMeters(120), 8, autoDrive, heading).withTimeout(3),
      new AutoStopAllCommands(delivery, intake, kicker, shooter)
    );
  }

}