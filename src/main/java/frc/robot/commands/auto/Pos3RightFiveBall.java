package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.ProfiledPointToPointCommand;
import frc.robot.commands.shooter.RunKicker;
import frc.robot.commands.shooter.StartShooter;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class Pos3RightFiveBall extends SequentialCommandGroup {

  private Drivetrain drivetrain;
  private Kicker kicker;
  private Shooter shooter;

  public Pos3RightFiveBall(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading, Kicker kicker, Shooter shooter) {
    this.drivetrain = drivetrain;

    addCommands(
      new StartShooter(shooter),
      new ParallelCommandGroup(
        new ProfiledPointToPointCommand(Constants.Auto.kBallR3Pickup, drivetrain::getPose, heading, autoDrive, 3.0, 0.05, Units.inchesToMeters(120), 8).withTimeout(2),
        new WaitCommand(0.2),
        new RunKicker(kicker).withTimeout(1)
      ),
      new ProfiledPointToPointCommand(Constants.Auto.kBallR2Pickup, drivetrain::getPose, heading, autoDrive, 3.0, 0.05, Units.inchesToMeters(120), 15).withTimeout(3),
      new WaitCommand(1),
      new ProfiledPointToPointCommand(Constants.Auto.kBallR2ShootPosition, drivetrain::getPose, heading, autoDrive, 3.0, 0.05, Units.inchesToMeters(120), 15).withTimeout(2),
      new WaitCommand(1),
      new ProfiledPointToPointCommand(Constants.Auto.kBallR4Pickup, drivetrain::getPose, heading, autoDrive, 3.0, 0.025, Units.inchesToMeters(120), 8).withTimeout(4),
      new WaitCommand(1),
      new ProfiledPointToPointCommand(Constants.Auto.kFiveBallShootPosition, drivetrain::getPose, heading, autoDrive, 3.0, 0.01, Units.inchesToMeters(120), 8).withTimeout(4)
    );
  }
  public void initialize() {
    drivetrain.resetPosition(new Pose2d(Constants.Auto.position3RightStart.toFieldCoordinate(), Rotation2d.fromDegrees(0)));
  }
}