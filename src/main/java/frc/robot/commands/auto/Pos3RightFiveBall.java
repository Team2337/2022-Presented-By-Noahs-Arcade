package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.ProfiledPointToPointCommand;
import frc.robot.commands.delivery.AutoStartDelivery;
import frc.robot.commands.delivery.StartDelivery;
import frc.robot.commands.intake.AutoStartIntake;
import frc.robot.commands.shooter.AutoKickerCommand;
import frc.robot.commands.shooter.AutoStartShooter;
import frc.robot.commands.shooter.RunKicker;
import frc.robot.commands.shooter.StartShooter;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class Pos3RightFiveBall extends SequentialCommandGroup {

  private Drivetrain drivetrain;
  private Kicker kicker;
  private Shooter shooter;
  private Intake intake;
  private Delivery delivery;

  public Pos3RightFiveBall(AutoDrive autoDrive, Delivery delivery, Drivetrain drivetrain, Heading heading, Intake intake, Kicker kicker, Shooter shooter) {
    this.drivetrain = drivetrain;

    addCommands(
      new AutoStartShooter(shooter),
      new ParallelCommandGroup(
        new AutoStartIntake(intake),
        new ProfiledPointToPointCommand(Constants.Auto.kBallR3Pickup, drivetrain::getTranslation, 2.5, 0.05, Units.inchesToMeters(120), 12, autoDrive, heading).withTimeout(2)
        ),
      new AutoKickerCommand(kicker, 0).withTimeout(0.5),
      new ParallelCommandGroup(
        new ProfiledPointToPointCommand(Constants.Auto.kBallR2Pickup, drivetrain::getTranslation, 3.0, 0.05, Units.inchesToMeters(120), 12, autoDrive, heading).withTimeout(3),
        new AutoStartDelivery(delivery).withTimeout(0.75)
        ),
        new AutoKickerCommand(kicker, 0).withTimeout(0.5),
      new ProfiledPointToPointCommand(Constants.Auto.kBallR2ShootPosition, drivetrain::getTranslation, 3.0, 0.05, Units.inchesToMeters(120), 15, autoDrive, heading).withTimeout(1),
      new ParallelCommandGroup(
        new AutoKickerCommand(kicker, 0).withTimeout(1.5),
        new AutoStartDelivery(delivery).withTimeout(1.5)
      ),
      new WaitCommand(5),
      new ProfiledPointToPointCommand(Constants.Auto.kPosition3RightStart, drivetrain::getTranslation, 1.0, 0.05, Units.inchesToMeters(120), 8, autoDrive, heading).withTimeout(3)
      /*
      //new ParallelCommandGroup(
        //new RunKicker(kicker).withTimeout(2),
        //new StartDelivery(delivery).withTimeout(2)
      //),
      new ProfiledPointToPointCommand(Constants.Auto.kBallR4Pickup, drivetrain::getTranslation, 3.0, 0.025, Units.inchesToMeters(120), 8, autoDrive, heading).withTimeout(4),
      new WaitCommand(2),
      new ProfiledPointToPointCommand(Constants.Auto.kFiveBallShootPosition, drivetrain::getTranslation, 3.0, 0.01, Units.inchesToMeters(120), 8, autoDrive, heading).withTimeout(4)
      //new ParallelCommandGroup(
        //new RunKicker(kicker).withTimeout(2),
        //new StartDelivery(delivery).withTimeout(2)
      //)
      */
    );
  }
}