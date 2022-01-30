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
    // Clamp to some max speed (should be between [0.0, 1.0])
    final double maxSpeed = 0.3;
    output = MathUtil.clamp(
      output,
      -maxSpeed,
      maxSpeed
    );
    
    Pose2d pose = poseSupplier.get();
    double x = pose.getX() - targetMeters.getX();
    double y = pose.getY() - targetMeters.getY();
    double towardsCenterDegrees = Math.atan2(y, x);
    Rotation2d desiredRotation = Rotation2d.fromDegrees(Units.radiansToDegrees(towardsCenterDegrees) - 180);
    heading.setMaintainHeading(desiredRotation);
  }

  public AutoDrive.State calculate(double forward, double strafe, boolean isFieldOriented) {
    // Use our xController output calculation as our forward value.
    // Driver can strafe during this command. Forward should be forward to the robot.
    // Note that this command assumes we're facing the target (use with Heading)
    return new AutoDrive.State(
      -output,
      strafe,
      false
    );
  }

}
