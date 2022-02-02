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

public class TestStraightDistance extends SequentialCommandGroup {
  public TestStraightDistance(Drivetrain drivetrain, Heading heading, AutoDrive autoDrive) {
    addCommands(
      new ProfiledPointToPointCommand(Constants.Auto.testStrafe, drivetrain::getPose, drivetrain::getChassisSpeeds, heading, autoDrive, 3.0, 0.01, 15, 15)
    );
  }

}
