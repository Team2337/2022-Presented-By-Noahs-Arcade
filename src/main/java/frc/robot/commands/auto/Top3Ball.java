package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoDrivableCommand;
import frc.robot.commands.teleop.PointToPointCommand;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.AutoDrive.State;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class Top3Ball extends SequentialCommandGroup implements AutoDrivableCommand {
  public Top3Ball(Drivetrain drivetrain, Heading heading, AutoDrive autoDrive) {
    addCommands(
      new PointToPointCommand(Constants.Auto.kBall1, drivetrain::getPose, drivetrain::getChassisSpeeds, heading, autoDrive),
      new WaitCommand(3),
      new PointToPointCommand(new PolarCoordinate(Constants.Auto.kBall2.getRadiusMeters() - Units.inchesToMeters(17), Constants.Auto.kBall2.getTheta()), drivetrain::getPose, drivetrain::getChassisSpeeds, heading, autoDrive)
    );
  }

  @Override
  public State calculate(double forward, double strafe, boolean isFieldOriented) {
    return m_currentCommand.calculate(forward, strafe, isFieldOriented);
  }

}
