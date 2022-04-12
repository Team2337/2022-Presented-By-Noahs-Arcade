package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.commandGroups.AutoStopAllCommands;
import frc.robot.commands.auto.commandGroups.FirstMove;
import frc.robot.commands.delivery.AutoStartDelivery;
import frc.robot.commands.delivery.AutoStartDeliveryLeft;
import frc.robot.commands.intake.AutoStartIntake;
import frc.robot.commands.kicker.ForwardKickerCommand;
import frc.robot.commands.shooter.StartShooterInstantCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class Pos2MidFiveBall extends SequentialCommandGroup {
  private double forwardP = 1.5;
  private double strafeP = 0.05;
  private double forwardAcceleration = Units.inchesToMeters(45);
  private double strafeAcceleration = 8;

  public Pos2MidFiveBall(AutoDrive autoDrive, Delivery delivery, Drivetrain drivetrain, Heading heading, Intake intake, Kicker kicker, Shooter shooter) {
    addCommands(
      new StartShooterInstantCommand(42, shooter),
      new ParallelCommandGroup(
        new AutoStartIntake(intake),
        new ProfiledPointToPointCommand(Constants.Auto.kBallR2ShootPosition, drivetrain::getTranslation, intake::getAutoBeamBreakSensorStatus, forwardP, strafeP, forwardAcceleration, strafeAcceleration, true, autoDrive, heading).withTimeout(2)),
      new ParallelCommandGroup(
        new ForwardKickerCommand(kicker).withTimeout(0.75),
        new AutoStartDeliveryLeft(delivery).withTimeout(0.75)
      ),
      new ProfiledPointToPointCommand(Constants.Auto.TransitionBetweenBallR2AndBallR4, drivetrain::getTranslation, intake::getAutoBeamBreakSensorStatus,1.5, 0.05, Units.inchesToMeters(60), 8, true, autoDrive, heading).withTimeout(2.5),
      new ProfiledPointToPointCommand(Constants.Auto.kBallR4Pickup, drivetrain::getTranslation, intake::getAutoBeamBreakSensorStatus, 1.5, 0.05, Units.inchesToMeters(60), 8, true, autoDrive, heading).withTimeout(1.5),
      new StartShooterInstantCommand(43, shooter),
      new ParallelCommandGroup(
        new ProfiledPointToPointCommand(Constants.Auto.kFiveBallShootPosition, drivetrain::getTranslation, 1.5, 0.01, Units.inchesToMeters(60), 8, autoDrive, heading).withTimeout(2.5),
        new AutoStartDelivery(delivery).withTimeout(0.6)
      ),
      new ParallelCommandGroup(
        new ForwardKickerCommand(kicker).withTimeout(2),
        new AutoStartDelivery(delivery).withTimeout(1)
      ),
      new ProfiledPointToPointCommand(Constants.Auto.kBallR3MidPickup, drivetrain::getTranslation, 1.5, 0.05, Units.inchesToMeters(45), 6, autoDrive, heading).withTimeout(1),
      new WaitCommand(0.25),
      new StartShooterInstantCommand(42, shooter),
      new ProfiledPointToPointCommand(Constants.Auto.kBallR3Pickup, drivetrain::getTranslation, intake::getAutoBeamBreakSensorStatus, 1.5, 0.05, Units.inchesToMeters(45), 6, true, autoDrive, heading).withTimeout(1),
      new ParallelCommandGroup(
        new ForwardKickerCommand(kicker).withTimeout(2),
        new AutoStartDelivery(delivery).withTimeout(1)
      ),
      new WaitCommand(2),
      // new ProfiledPointToPointCommand(Constants.Auto.kPosition3RightStart, drivetrain::getTranslation, 1.0, 0.05, Units.inchesToMeters(120), 8, autoDrive, heading).withTimeout(3),
      new AutoStopAllCommands(delivery, intake, kicker, shooter)
    );
  }
}