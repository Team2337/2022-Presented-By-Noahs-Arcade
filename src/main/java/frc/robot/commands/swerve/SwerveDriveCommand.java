package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.nerdyfiles.utilities.Utilities;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;


public class SwerveDriveCommand extends CommandBase {

  private final XboxController controller;

  private final AutoDrive autoDrive;
  private final Heading heading;
  private final Drivetrain drivetrain;

  // Smooth our our joystick values
  private static double kSlewRateOfChangePerSecond = 1.75;
  SlewRateLimiter forwardSlew = new SlewRateLimiter(kSlewRateOfChangePerSecond);
  SlewRateLimiter strafeSlew = new SlewRateLimiter(kSlewRateOfChangePerSecond);
  SlewRateLimiter rotationSlew = new SlewRateLimiter(kSlewRateOfChangePerSecond);
  PIDController rotationController = new PIDController(0.07, 0.0, 0.0);
  private PolarCoordinate joystickCoordinates = new PolarCoordinate(0, Rotation2d.fromDegrees(0));

  /**
   * Command running the swerve calculations with the joystick
   *
   * @param subsystem - SwerveDrivetrain subsystem object
   */
  public SwerveDriveCommand(XboxController controller, AutoDrive autoDrive, Heading heading, Drivetrain drivetrain) {
    this.controller = controller;
    this.autoDrive = autoDrive;
    this.heading = heading;
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    double forward = -Utilities.deadbandAndSquare(controller.getLeftY());
    double strafe = -Utilities.deadbandAndSquare(controller.getLeftX());
    double rotation = -Utilities.deadbandAndSquare(controller.getRightX(), 0.09);
    boolean isFieldOriented = !controller.getLeftBumper();

    AutoDrive.State autoDriveState = autoDrive.calculate(forward, strafe, isFieldOriented);
    if (autoDriveState != null) {
      forward = autoDriveState.forward;
      strafe = autoDriveState.strafe;
      isFieldOriented = autoDriveState.isFieldOriented;
    }

    if (DriverStation.isTeleopEnabled()) {
      forward = forwardSlew.calculate(forward);
      strafe = strafeSlew.calculate(strafe);
      rotation = rotationSlew.calculate(rotation);
    }
    
    // If a driver-initiated rotation is provided, disable our rotation
    // controller to let the driver rotate freely.
    // Never allow this flow in autonomous (protect against floating joystick
    // values coming from our controllers)
    if (DriverStation.isTeleopEnabled() && rotation != 0 && heading.isEnabled()) {
      heading.disableMaintainHeading();
      double y = controller.getLeftX();
      double x = controller.getLeftY();
      Rotation2d joystickAngle = joystickCoordinates.fromFieldCoordinate(new Translation2d(x, y), new Translation2d(0, 0)).getTheta();
      Rotation2d gAngle = drivetrain.getGyroscopeRotation();
      Rotation2d dAngle = joystickAngle.minus(gAngle);
      SmartDashboard.putNumber("Jangle", joystickAngle.getDegrees());
      SmartDashboard.putNumber("Gangle", gAngle.getDegrees());
      SmartDashboard.putNumber("Dangle", dAngle.getDegrees());

      Rotation2d currentHeading = Utilities.convertRotationToRelativeRotation(drivetrain.getGyroscopeRotation());
      double output = rotationController.calculate(
        currentHeading.getDegrees(),
        dAngle.getDegrees());
      // If our controller is within our tolerance - do not provide a nominal output
      if (rotationController.atSetpoint()) {
      // SmartDashboard.putNumber("Heading/Rotation Controller Output", 0.0);
        output = 0.0;
      }
      // Clamp to some max speed (should be between [0.0, 1.0])
      final double maxSpeed = 0.3;
      double clampedOutput = MathUtil.clamp(
        output,
        -maxSpeed,
        maxSpeed);
      double nominalClampedOutput;
      nominalClampedOutput = Math.copySign(
          Math.max(
              Math.abs(clampedOutput),
              drivetrain.isMoving() ? 0.03 : 0.06),
          clampedOutput);
      // SmartDashboard.putNumber("Heading/Rotation Controller Output",
      // nominalClampedOutput);
      SmartDashboard.putNumber("Output", nominalClampedOutput);
      rotation =  nominalClampedOutput;

    }

    /**
     * Calculate a rotation value for the robot to achieve it's
     * maintained heading - if the robot should be maintaining a heading.
     * Will not be calculated if the rotation joystick has an input.
     */
    if (heading.shouldMaintainHeading()) {
      rotation = heading.calculateRotation();
    }

    double vxMetersPerSecond = forward * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
    double vyMetersPerSecond = strafe * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
    double omegaRadiansPerSecond = rotation * Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    if (isFieldOriented) {
      drivetrain.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          vxMetersPerSecond,
          vyMetersPerSecond,
          omegaRadiansPerSecond,
          drivetrain.getGyroscopeRotation()
        )
      );
    } else {
      if (controller.getLeftBumper()) {
        drivetrain.drive(new ChassisSpeeds(
          -vxMetersPerSecond,
          -vyMetersPerSecond,
          omegaRadiansPerSecond
        ));
        } else {
          drivetrain.drive(new ChassisSpeeds(
          vxMetersPerSecond,
          vyMetersPerSecond,
          omegaRadiansPerSecond
        ));
        }
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
