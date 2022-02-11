package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utilities;
import frc.robot.commands.HeadingToTargetCommand;
import frc.robot.commands.interfaces.AutoDrivableCommand;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Heading;

/**
 * Drive a certain distance from a point - regardless of angle - and maintain that
 * distance away from the point.
 */
public class ProfiledPointToPointCommand extends HeadingToTargetCommand implements AutoDrivableCommand {

  private PolarCoordinate target;
  private Supplier<Pose2d> poseSupplier;
  private Heading heading;
  private AutoDrive autoDrive;

  private ProfiledPIDController distanceController = new ProfiledPIDController(3, 0.0, 0.0, new TrapezoidProfile.Constraints(Units.inchesToMeters(120), Units.inchesToMeters(120)));
  private ProfiledPIDController thetaController = new ProfiledPIDController(0.05, 0.0, 0.0, new TrapezoidProfile.Constraints(45, Math.pow(15, 2)));

  private double forwardOutput = 0.0;
  private double strafeOutput = 0.0;

  public ProfiledPointToPointCommand(PolarCoordinate target, Supplier<Pose2d> poseSupplier, Heading heading, AutoDrive autoDrive, double driveP, double strafeP, double forwardAcceleration, double strafeAcceleration) {
     super(
      target.toFieldCoordinate(),
      () -> poseSupplier.get().getTranslation(),
      heading
    );

    this.target = target;
    this.poseSupplier = poseSupplier;
    this.heading = heading;
    this.autoDrive = autoDrive;

    distanceController.setTolerance(Units.inchesToMeters(1));
    thetaController.setTolerance(0.1); // In degrees

    distanceController.setP(driveP);
    thetaController.setP(strafeP);

    distanceController.setConstraints(new TrapezoidProfile.Constraints(Units.inchesToMeters(120), forwardAcceleration));
    thetaController.setConstraints(new TrapezoidProfile.Constraints(45, Math.pow(strafeAcceleration, 2)));

    SmartDashboard.putNumber("ProfiledP2P/Target Distance (inches)", Units.metersToInches(target.getRadiusMeters()));
    SmartDashboard.putNumber("ProfiledP2P/Target Theta (Degrees)", target.getTheta().getDegrees());

    addRequirements(autoDrive);
  }

  @Override
  public void initialize() {
    super.initialize();

    heading.enableMaintainHeading();
    autoDrive.setDelegate(this);

    // Set our initial setpoint for our profiled PID controllers
    // to avoid a JUMP to their starting values on first run
    PolarCoordinate robotCoordinate = getRobotCoordinate();
    // Use a relative heading when talking to our theta controller.
    // This allows us to take the shortest distance to our desired theta.
    // Ex: 0 -> 270 would get converted to 0 -> -90 - a shorter path.
    thetaController.reset(
      Utilities.convertRotationToRelativeRotation(
        robotCoordinate.getTheta()
      ).getDegrees()
    );
    distanceController.reset(robotCoordinate.getRadiusMeters());
  }

  private PolarCoordinate getRobotCoordinate() {
    return PolarCoordinate.fromFieldCoordinate(
      poseSupplier.get().getTranslation(),
      target.getReferencePoint()
    );
  }

  @Override
  public void execute() {
    super.execute();

    PolarCoordinate robotCoordinate = getRobotCoordinate();
    forwardOutput = distanceController.calculate(
      robotCoordinate.getRadiusMeters(),
      target.getRadiusMeters()
    );
    strafeOutput = thetaController.calculate(
      Utilities.convertRotationToRelativeRotation(
        robotCoordinate.getTheta()
      ).getDegrees(),
      Utilities.convertRotationToRelativeRotation(
        target.getTheta()
      ).getDegrees()
    );

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

    log(robotCoordinate);
  }

  public AutoDrive.State calculate(double forward, double strafe, boolean isFieldOriented) {
    return new AutoDrive.State(
      -forwardOutput,
      strafeOutput,
      false
    );
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    autoDrive.clearDelegate();
  }

  @Override
  public boolean isFinished() {
    return distanceController.atGoal() && thetaController.atGoal();
  }

  private void log(PolarCoordinate robotCoordinate) {
    SmartDashboard.putNumber("ProfiledP2P/Robot Angle (degrees)", robotCoordinate.getTheta().getDegrees());
    SmartDashboard.putNumber("ProfiledP2P/Robot Distance (inches)", Units.metersToInches(robotCoordinate.getRadiusMeters()));

    SmartDashboard.putNumber("ProfiledP2P/Forward Output", forwardOutput);
    SmartDashboard.putNumber("ProfiledP2P/Strafe Output", strafeOutput);

    SmartDashboard.putNumber("ProfiledP2P/Distance Position (inches)", Units.metersToInches(distanceController.getSetpoint().position));
    SmartDashboard.putNumber("ProfiledP2P/Distance Error (inches)", Units.metersToInches(distanceController.getPositionError()));

    SmartDashboard.putNumber("ProfiledP2P/Theta Position", thetaController.getSetpoint().position);
    SmartDashboard.putNumber("ProfiledP2P/Theta Error", thetaController.getPositionError());

    SmartDashboard.putBoolean("ProfiledP2P/distanceController atGoal", distanceController.atGoal());
    SmartDashboard.putBoolean("ProfiledP2P/thetaController atGoal", thetaController.atGoal());
  }

}
