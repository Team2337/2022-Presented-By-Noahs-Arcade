package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.ProfiledPointToPointCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class RedPos3FiveBall extends SequentialCommandGroup {

  private Drivetrain drivetrain;

  public RedPos3FiveBall(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
    this.drivetrain = drivetrain;

    addCommands(
      new ProfiledPointToPointCommand(Constants.Auto.kBall1Pickup, drivetrain::getPose, heading, autoDrive, 3.0, 0.05, Units.inchesToMeters(120), 8),
      new WaitCommand(1),
      new ProfiledPointToPointCommand(Constants.Auto.kBall2Pickup, drivetrain::getPose, heading, autoDrive, 3.0, 0.05, Units.inchesToMeters(120), 15),
      new WaitCommand(1),
      new ProfiledPointToPointCommand(Constants.Auto.kBall2ShootPosition, drivetrain::getPose, heading, autoDrive, 3.0, 0.05, Units.inchesToMeters(120), 15),
      new WaitCommand(1),
      new ProfiledPointToPointCommand(Constants.Auto.kBall3Pickup, drivetrain::getPose, heading, autoDrive, 3.0, 0.025, Units.inchesToMeters(120), 8),
      new WaitCommand(1),
      new ProfiledPointToPointCommand(Constants.Auto.kFiveBallShootPosition, drivetrain::getPose, heading, autoDrive, 3.0, 0.01, Units.inchesToMeters(120), 8)
    );
  }
  public void initialize() {
    drivetrain.resetPosition(new Pose2d(Constants.Auto.redPosition3Start.toFieldCoordinate(), Rotation2d.fromDegrees(0)));
  }
}