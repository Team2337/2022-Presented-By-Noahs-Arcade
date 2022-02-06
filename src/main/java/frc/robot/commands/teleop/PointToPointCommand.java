package frc.robot.commands.teleop;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.interfaces.AutoDrivableCommand;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Heading;

/**
 * Drive a certain distance from a point - regardless of angle - and maintain that
 * distance away from the point.
 */
public class PointToPointCommand extends CommandBase implements AutoDrivableCommand {

  private PolarCoordinate target;
  private Supplier<Pose2d> poseSupplier;
  private Heading heading;
  private AutoDrive autoDrive;

  private PIDController distanceController = new PIDController(1.0, 0.0, 0.0);
  private PIDController thetaController = new PIDController(0.05, 0.0, 0.0);

  private double forwardOutput = 0.0;
  private double strafeOutput = 0.0;

  public PointToPointCommand(PolarCoordinate target, Supplier<Pose2d> poseSupplier, Heading heading, AutoDrive autoDrive) {
    this.target = target;
    this.poseSupplier = poseSupplier;
    this.heading = heading;
    this.autoDrive = autoDrive;

    distanceController.setTolerance(Units.inchesToMeters(1));
    thetaController.setTolerance(0.05); //In degrees

    SmartDashboard.putNumber("Target Distance (feet)", Units.metersToFeet(target.getRadiusMeters()));
    SmartDashboard.putNumber("Target Theta (Degrees)", target.getTheta().getDegrees());

    addRequirements(heading, autoDrive);
  }

  @Override
  public void initialize() {
    heading.enableMaintainHeading();
    autoDrive.setDelegate(this);
  }

  @Override
  public void execute() {
    // Find our robot polar coordinate relative to the Hub
    // -90 (robot is underneath the reference point)
    PolarCoordinate robotCoordinate = PolarCoordinate.fromFieldCoordinate(
      poseSupplier.get().getTranslation(),
      target.getReferencePoint()
    );
    forwardOutput = distanceController.calculate(
      robotCoordinate.getRadiusMeters(),
      target.getRadiusMeters()
    );
    strafeOutput = thetaController.calculate(
      robotCoordinate.getTheta().getDegrees(),
      target.getTheta().getDegrees()
    );

    SmartDashboard.putBoolean("distanceController atSetpoint", distanceController.atSetpoint());
    SmartDashboard.putBoolean("thetaController atSetpoint", thetaController.atSetpoint());
    Logger.getInstance().recordOutput("PointToPoint/Distance At Setpoint", distanceController.atSetpoint());
    Logger.getInstance().recordOutput("PointToPoint/Theta At Setpoint", thetaController.atSetpoint());

    SmartDashboard.putNumber("Target Distance (feet)", Units.metersToFeet(target.getRadiusMeters()));
    SmartDashboard.putNumber("Target Theta (Degrees)", target.getTheta().getDegrees());

    SmartDashboard.putNumber("Robot Coordinate Angle", robotCoordinate.getTheta().getDegrees());
    SmartDashboard.putNumber("Robot Coordinate Distance", Units.metersToFeet(robotCoordinate.getRadiusMeters()));
    Logger.getInstance().recordOutput("PointToPoint/Robot Distance (inches)", Units.metersToInches(robotCoordinate.getRadiusMeters()));
    Logger.getInstance().recordOutput("PointToPoint/Robot Angle (degrees)", robotCoordinate.getTheta().getDegrees());
    
    SmartDashboard.putNumber("Output", forwardOutput);
    Logger.getInstance().recordOutput("PointToPoint/Distance Error", distanceController.getPositionError());
    Logger.getInstance().recordOutput("PointToPoint/Angle Error", thetaController.getPositionError());
    Logger.getInstance().recordOutput("PointToPoint/Forward Output", forwardOutput);
    Logger.getInstance().recordOutput("PointToPoint/Strafe Output", strafeOutput);

    Pose2d pose = poseSupplier.get();
    Logger.getInstance().recordOutput("PointToPoint/Pose",
      new double[] { pose.getX(), pose.getY(), pose.getRotation().getRadians() });

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
    final double maxSpeed = 0.35;
    forwardOutput = MathUtil.clamp(
      forwardOutput,
      -maxSpeed,
      maxSpeed
    );

    strafeOutput = MathUtil.clamp(
      strafeOutput,
      -maxSpeed,
      maxSpeed
    );

    SmartDashboard.putNumber("Initial X", pose.getX());
    SmartDashboard.putNumber("Initial Y", pose.getY());
    double x = pose.getX() - target.getReferencePoint().getX();
    double y = pose.getY() - target.getReferencePoint().getY();
    double towardsCenterDegrees = Math.atan2(y, x);
    SmartDashboard.putNumber("Towards Center Degrees (Tangent)", Units.radiansToDegrees(towardsCenterDegrees));
    SmartDashboard.putNumber("X", x);
    SmartDashboard.putNumber("Y", y);
    Rotation2d desiredRotation = Rotation2d.fromDegrees(Units.radiansToDegrees(towardsCenterDegrees) - 180);
    SmartDashboard.putNumber("Odometry Desired Rotation", desiredRotation.getDegrees());
    SmartDashboard.putNumber("Forward Output", forwardOutput);
    SmartDashboard.putNumber("Strafe Output", strafeOutput);
    heading.setMaintainHeading(desiredRotation);
  }

  public AutoDrive.State calculate(double forward, double strafe, boolean isFieldOriented) {
    // Use our xController output calculation as our forward value.
    // Driver can strafe during this command. Forward should be forward to the robot.
    // Note that this command assumes we're facing the target (use with Heading)
    return new AutoDrive.State(
      -forwardOutput,
      strafeOutput,
      false
    );
  }

  @Override
  public void end(boolean interrupted) {
    autoDrive.clearDelegate();
  }

  @Override
  public boolean isFinished() {
    return distanceController.atSetpoint() && thetaController.atSetpoint();
  }

}
