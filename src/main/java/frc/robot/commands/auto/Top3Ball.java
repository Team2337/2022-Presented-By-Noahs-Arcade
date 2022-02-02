package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.teleop.PointToPointCommand;
import frc.robot.commands.teleop.ProfiledPointToPointCommand;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class Top3Ball extends SequentialCommandGroup {
  public Top3Ball(Drivetrain drivetrain, Heading heading, AutoDrive autoDrive) {
    addCommands(
      new ProfiledPointToPointCommand(Constants.Auto.kBall1Pickup, drivetrain::getPose, drivetrain::getChassisSpeeds, heading, autoDrive, 3.0, 0.05, Units.inchesToMeters(120), 8),
      new WaitCommand(1),
      new ProfiledPointToPointCommand(Constants.Auto.kBall2Pickup, drivetrain::getPose, drivetrain::getChassisSpeeds, heading, autoDrive, 3.0, 0.05, Units.inchesToMeters(120), 15),
      new WaitCommand(1),
      new ProfiledPointToPointCommand(Constants.Auto.kBall3Pickup, drivetrain::getPose, drivetrain::getChassisSpeeds, heading, autoDrive, 3.0, 0.025, Units.inchesToMeters(120), 8),
      new WaitCommand(1),
      new ProfiledPointToPointCommand(Constants.Auto.kFiveBallShootPosition, drivetrain::getPose, drivetrain::getChassisSpeeds, heading, autoDrive, 3.0, 0.01, Units.inchesToMeters(120), 8)
    );
  }

}
