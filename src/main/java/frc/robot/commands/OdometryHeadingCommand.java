package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

/**
 * Use the Drivetrain odometry to update the Heading subsystem to
 * keep the robot facing a single point. By default that point is
 * the center of the field aka the Hub.
 */
public class OdometryHeadingCommand extends CommandBase {
  
  private Translation2d point;
  private Drivetrain drivetrain;
  private Heading heading;

  public OdometryHeadingCommand(Drivetrain drivetrain, Heading heading) {
    this(Constants.kHub, drivetrain, heading);
  }

  public OdometryHeadingCommand(Translation2d point, Drivetrain drivetrain, Heading heading) {
    this.point = point;
    this.drivetrain = drivetrain;
    this.heading = heading;

    addRequirements(heading);
  }

  @Override
  public void execute() {
    // Might be able to use PolarCoordinate.fromFieldCoordinate here
    Pose2d pose = drivetrain.getPose();
    double x = pose.getX() - point.getX();
    double y = pose.getY() - point.getY();
    double angle = Math.atan2(y, x);  // (-π, π] radians
    // The angle is the angle outward from our center point. In order to face our center
    // point, we need to rotate our angle by 180 degrees.
    heading.setMaintainHeading(new Rotation2d(angle).unaryMinus());
  }

}