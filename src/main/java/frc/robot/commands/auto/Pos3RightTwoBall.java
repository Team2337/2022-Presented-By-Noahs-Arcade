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
import frc.robot.commands.intake.AutoStartIntake;
import frc.robot.commands.shooter.AutoKickerCommand;
import frc.robot.commands.shooter.AutoStartShooter;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class Pos3RightTwoBall extends SequentialCommandGroup {

  private Drivetrain drivetrain;
  private Kicker kicker;
  private Shooter shooter;
  private Intake intake;
  private Delivery delivery;

  

  public Pos3RightTwoBall(AutoDrive autoDrive, Delivery delivery, Drivetrain drivetrain, Heading heading, Intake intake, Kicker kicker, Shooter shooter) {
    this.drivetrain = drivetrain;

    addCommands(
      new AutoStartShooter(shooter),
      new ParallelCommandGroup(
        new AutoStartIntake(intake),
        new ProfiledPointToPointCommand(Constants.Auto.kBallR3Pickup, drivetrain::getTranslation, 2.5, 0.05, Units.inchesToMeters(120), 12, autoDrive, heading).withTimeout(2)
        ),
      new ParallelCommandGroup(
        new AutoStartDelivery(delivery).withTimeout(1),
        new AutoKickerCommand(kicker, 0).withTimeout(1)
      ),
      new WaitCommand(5),
      new ProfiledPointToPointCommand(Constants.Auto.kPosition3RightStart, drivetrain::getTranslation, 1.0, 0.05, Units.inchesToMeters(120), 8, autoDrive, heading).withTimeout(3)  
    );
  }
}