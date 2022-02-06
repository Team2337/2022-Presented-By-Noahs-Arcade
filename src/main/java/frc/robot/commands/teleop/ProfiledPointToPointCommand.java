package frc.robot.commands.teleop;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
public class ProfiledPointToPointCommand extends CommandBase implements AutoDrivableCommand {

  private PolarCoordinate target;
  private Supplier<Pose2d> poseSupplier;
  private Heading heading;
  private AutoDrive autoDrive;

  private ProfiledPIDController distanceController = new ProfiledPIDController(3, 0.0, 0.0, new TrapezoidProfile.Constraints(Units.inchesToMeters(120), Units.inchesToMeters(120)));
  private ProfiledPIDController thetaController = new ProfiledPIDController(0.05, 0.0, 0.0, new TrapezoidProfile.Constraints(45, Math.pow(15, 2)));

  private double forwardOutput = 0.0;
  private double strafeOutput = 0.0;
  private double driveP = 3.0;
  private double strafeP = 0.05;
  private double forwardAcceleration = Units.inchesToMeters(120);
  private double strafeAcceleration = Math.pow(15, 2);

  public ProfiledPointToPointCommand(PolarCoordinate target, Supplier<Pose2d> poseSupplier, Heading heading, AutoDrive autoDrive, double driveP, double strafeP, double forwardAcceleration, double strafeAcceleration) {
    this.target = target;
    this.poseSupplier = poseSupplier;
    this.heading = heading;
    this.autoDrive = autoDrive;
    this.driveP = driveP;
    this.strafeP = strafeP;
    this.forwardAcceleration = forwardAcceleration;
    this.strafeAcceleration = strafeAcceleration;

    distanceController.setTolerance(Units.inchesToMeters(1));
    thetaController.setTolerance(0.1); // In degrees

    SmartDashboard.putNumber("Target Distance (feet)", Units.metersToFeet(target.getRadiusMeters()));
    SmartDashboard.putNumber("Target Theta (Degrees)", target.getTheta().getDegrees());

    addRequirements(heading, autoDrive);
  }

  @Override
  public void initialize() {
    heading.enableMaintainHeading();
    autoDrive.setDelegate(this);

    PolarCoordinate robotCoordinate = PolarCoordinate.fromFieldCoordinate(
      poseSupplier.get().getTranslation(),
      target.getReferencePoint()
    );
    thetaController.reset(robotCoordinate.getTheta().getDegrees());
    distanceController.reset(robotCoordinate.getRadiusMeters());

    distanceController.setP(driveP);
    thetaController.setP(strafeP);
    distanceController.setConstraints(new TrapezoidProfile.Constraints(Units.inchesToMeters(120), forwardAcceleration));
    thetaController.setConstraints(new TrapezoidProfile.Constraints(45, Math.pow(strafeAcceleration, 2)));
  }

  @Override
  public void execute() {
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

    SmartDashboard.putNumber("Robot Coordinate Angle", robotCoordinate.getTheta().getDegrees());
    SmartDashboard.putNumber("Robot Coordinate Distance", Units.metersToInches(robotCoordinate.getRadiusMeters()));

    SmartDashboard.putNumber("Target Distance (inches)", Units.metersToInches(target.getRadiusMeters()));
    SmartDashboard.putNumber("Target Theta (Degrees)", target.getTheta().getDegrees());

    SmartDashboard.putBoolean("distanceController atSetpoint", distanceController.atSetpoint());
    SmartDashboard.putBoolean("thetaController atSetpoint", thetaController.atSetpoint());
    
    SmartDashboard.putBoolean("distanceController atGoal", distanceController.atGoal());
    SmartDashboard.putBoolean("thetaController atGoal", thetaController.atGoal());

    SmartDashboard.putNumber("Distance Position (inches)", Units.metersToInches(distanceController.getSetpoint().position));
    SmartDashboard.putNumber("Theta Position", thetaController.getSetpoint().position);

    Logger.getInstance().recordOutput("PointToPoint/Distance Setpoint (Position)", Units.metersToInches(distanceController.getSetpoint().position));
    Logger.getInstance().recordOutput("PointToPoint/Distance Setpoint (Velocity)", Units.metersToInches(distanceController.getSetpoint().velocity));
    Logger.getInstance().recordOutput("PointToPoint/Theta Setpoint (Position)", thetaController.getSetpoint().position);
    Logger.getInstance().recordOutput("PointToPoint/Theta Setpoint (Velocity)", thetaController.getSetpoint().velocity);

    Logger.getInstance().recordOutput("PointToPoint/Robot Distance (inches)", Units.metersToInches(robotCoordinate.getRadiusMeters()));
    Logger.getInstance().recordOutput("PointToPoint/Robot Angle (degrees)", robotCoordinate.getTheta().getDegrees());
    
    Logger.getInstance().recordOutput("PointToPoint/Distance Error", distanceController.getPositionError());
    Logger.getInstance().recordOutput("PointToPoint/Angle Error", thetaController.getPositionError());
    Logger.getInstance().recordOutput("PointToPoint/Forward Output", forwardOutput);
    Logger.getInstance().recordOutput("PointToPoint/Strafe Output", strafeOutput);

    SmartDashboard.putNumber("Theta Error", thetaController.getPositionError());

    // Translation2d distanceFieldCoordinate = distanceCoordinate.toFieldCoordinate();
    // SmartDashboard.putNumber("Target X", distanceFieldCoordinate.getX());
    // SmartDashboard.putNumber("Target Y", distanceFieldCoordinate.getY());
    // Add our current chassis speed as a FF value. This will allow us to gradually transition
    // in to our new speed as opposed to pretending the robot is always starting from zero and jerking
    //double xCurrentPercentage = chassisSpeedsSupplier.get().vxMetersPerSecond / Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
    //output += xCurrentPercentage;

    // Clamp to some max speed (should be between [0.0, 1.0])
    final double maxSpeed = 1.0;
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

    Pose2d pose = poseSupplier.get();
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
    return distanceController.atGoal() && thetaController.atGoal();
  }

}
