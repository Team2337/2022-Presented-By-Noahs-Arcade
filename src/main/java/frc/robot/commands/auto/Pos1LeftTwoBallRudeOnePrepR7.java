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
import frc.robot.commands.shooter.StartShooterInstantCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class Pos1LeftTwoBallRudeOnePrepR7 extends SequentialCommandGroup {

  public Pos1LeftTwoBallRudeOnePrepR7(AutoDrive autoDrive, Delivery delivery, Drivetrain drivetrain, Heading heading, Intake intake, Kicker kicker, Shooter shooter) {
    addCommands(
      new FirstMove(Constants.Auto.kBallR1RunOver, autoDrive, drivetrain, heading, intake, shooter),
      new ForwardKickerCommand(kicker).withTimeout(0.3),
      new ParallelCommandGroup(
        new AutoStartDelivery(delivery).withTimeout(0.75),
        new ForwardKickerCommand(kicker).withTimeout(0.75)
      ),
      new ProfiledPointToPointCommand(Constants.Auto.kBallD1Pickup, drivetrain::getTranslation, 1.5, 0.05, Units.inchesToMeters(60), 8, autoDrive, heading).withTimeout(2.5),
      new WaitCommand(0.25),
      new ProfiledPointToPointCommand(Constants.Auto.kBallD1RunOver, drivetrain::getTranslation, intake::getAutoBeamBreakSensorStatus, 1.5, 0.05, Units.inchesToMeters(60), 8, true, autoDrive, heading).withTimeout(1),
      new WaitCommand(0.25),
      new StartShooterInstantCommand(15, shooter),
      new ProfiledPointToPointCommand(Constants.Auto.kBallR1RunOver, drivetrain::getTranslation, 1.5, 0.05, Units.inchesToMeters(60), 8, autoDrive, heading).withTimeout(1),
      new ParallelCommandGroup(
        new AutoStartDelivery(delivery).withTimeout(0.75),
        new ForwardKickerCommand(kicker).withTimeout(0.75)
      ),
      new WaitCommand(0.25),
      new ProfiledPointToPointCommand(Constants.Auto.kBallR7Pickup, drivetrain::getTranslation, 1.5, 0.05, Units.inchesToMeters(60), 8, autoDrive, heading).withTimeout(2.5),
      // new ProfiledPointToPointCommand(Constants.Auto.kPosition1LeftStart, drivetrain::getTranslation, 1.0, 0.05, Units.inchesToMeters(120), 8, autoDrive, heading).withTimeout(3),
      new AutoStopAllCommands(delivery, intake, kicker, shooter)
    );
  }

}