package frc.robot.commands.teleop;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.AutoDrivableCommand;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Heading;

/**
 * Drive a certain distance from a point - regardless of angle - and maintain that
 * distance away from the point.
 */
public class DistanceToTargetCommand extends CommandBase implements AutoDrivableCommand {

  private Translation2d targetMeters;
  private double distanceMeters;
  private Supplier<Pose2d> poseSupplier;
  private Supplier<ChassisSpeeds> chassisSpeedsSupplier;
  private Heading heading;
  private AutoDrive autoDrive;

  private PIDController xController = new PIDController(1.0, 0, 0);
  private PIDController yController = new PIDController(1.0, 0, 0);

  private double output = 0.0;

  public DistanceToTargetCommand(double distanceMeters, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier, Heading heading, AutoDrive autoDrive) {
    this(Constants.kHub, distanceMeters, poseSupplier, chassisSpeedsSupplier, heading, autoDrive);
  }

  public DistanceToTargetCommand(Translation2d targetMeters, double distanceMeters, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier, Heading heading, AutoDrive autoDrive) {
    this.targetMeters = targetMeters;
    this.distanceMeters = distanceMeters;
    this.poseSupplier = poseSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    this.heading = heading;
    this.autoDrive = autoDrive;

    addRequirements(heading, autoDrive);
  }

  @Override
  public void initialize() {
    heading.enableMaintainHeading();
  }

  @Override
  public void execute() {
    // Find our robot polar coordinate relative to the Hub
    // -90 (robot is underneath the reference point)
    PolarCoordinate robotCoordinate = PolarCoordinate.fromFieldCoordinate(
      poseSupplier.get().getTranslation(),
      targetMeters
    );
    output = xController.calculate(
      robotCoordinate.getRadiusMeters(),
      distanceMeters
    );
    // outputY = yController.calculate(
      
    // );
    SmartDashboard.putNumber("Robot Coordinate Angle", robotCoordinate.getTheta().getDegrees());
    SmartDashboard.putNumber("Robot Coordinate Distance", Units.metersToFeet(robotCoordinate.getRadiusMeters()));
    SmartDashboard.putNumber("Output", output);
    
    Pose2d pose = poseSupplier.get();
    SmartDashboard.putNumber("Robot Pose X", pose.getX());
    SmartDashboard.putNumber("Robot Pose Y", pose.getY());
    // Translation2d distanceFieldCoordinate = distanceCoordinate.toFieldCoordinate();
    // SmartDashboard.putNumber("Target X", distanceFieldCoordinate.getX());
    // SmartDashboard.putNumber("Target Y", distanceFieldCoordinate.getY());
    // Add our current chassis speed as a FF value. This will allow us to gradually transition
    // in to our new speed as opposed to pretending the robot is always starting from zero and jerking
    //double xCurrentPercentage = chassisSpeedsSupplier.get().vxMetersPerSecond / Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
    //output += xCurrentPercentage;

    // Clamp to some max speed (should be between [0.0, 1.0])
    final double maxSpeed = 0.3;
    output = MathUtil.clamp(
      output,
      -maxSpeed,
      maxSpeed
    );
    // Face the robot TOWARDS the point we're heading towards
    double x = pose.getX() - targetMeters.getX();
    double y = pose.getY() - targetMeters.getY();
    double angle = Math.atan2(y, x); // (-π, π] radians
    // The angle is the angle outward from our center point. In order to face our center
    // point, we need to rotate our angle by 180 degrees.
    Rotation2d desiredRotation = Rotation2d.fromDegrees(Units.radiansToDegrees(angle) - 180);
    SmartDashboard.putNumber("Desired Rotation", desiredRotation.getDegrees());
    heading.setMaintainHeading(desiredRotation);

    // heading.setMaintainHeading(Rotation2d.fromDegrees(-1 * (90 + robotCoordinate.getTheta().getDegrees())));
  }

  public AutoDrive.State calculate(double forward, double strafe, boolean isFieldOriented) {
    // Use our xController output calculation as our forward value.
    // Driver can strafe during this command. Forward should be forward to the robot.
    // Note that this command assumes we're facing the target (use with Heading)
    return new AutoDrive.State(
      -output,
      strafe,
      true
    );
  }

}
