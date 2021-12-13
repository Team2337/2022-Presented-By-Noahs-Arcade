package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Utilities;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;


public class SwerveDriveCommand extends CommandBase {

  private final XboxController controller;

  private final Heading heading;
  private final Drivetrain drivetrain;

  /**
   * Command running the swerve calculations with the joystick
   *
   * @param subsystem - SwerveDrivetrain subsystem object
   */
  public SwerveDriveCommand(XboxController controller, Heading heading, Drivetrain drivetrain) {
    this.controller = controller;
    this.heading = heading;
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    double forward = -Utilities.modifyAxis(controller.getLeftY());
    double strafe = -Utilities.modifyAxis(controller.getLeftX());
    double rotation = -Utilities.modifyAxis(controller.getRightX());

    double vxMetersPerSecond = forward * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
    double vyMetersPerSecond = strafe * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
    double omegaRadiansPerSecond = rotation * Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    boolean isFieldOriented = !controller.getLeftBumper();

    /**
     * Set the rotational value of the robot. Fallback to using joysticks for
     * rotational value. Check to see if we're attempting to maintain some
     * static heading provided by our Heading subsystem.
     */
    Rotation2d desiredHeading = heading.getDesiredHeading();
    if (desiredHeading != null) {
      /**
       * Code largely lifted from 2019 -
       * https://github.com/Team2337/2020-Perpetual-Supercharger/blob/b4366826ed0e9842878486df3a2a3898ca411ebe/src/main/java/frc/robot/subsystems/OperatorAngleAdjustment.java#L202-L220
       */
      Rotation2d rotationError = drivetrain.getGyroscopeRotation().minus(desiredHeading);

      /** Rotational P while not rotating */
      double stationaryP = 0.015;
      /** Rotational P while rotating */
      double movingP = 0.01;

      double kP = forward == 0 && strafe == 0 ? stationaryP : movingP;

      double calculatedRotation = rotationError.getDegrees() * kP;
      // Max out our maximum automatic rotational speed at 0.6 out of 1.0
      calculatedRotation = (Math.abs(calculatedRotation) > 0.6) ? Math.copySign(0.6, calculatedRotation) : calculatedRotation;
      omegaRadiansPerSecond = calculatedRotation * Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

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
      drivetrain.drive(new ChassisSpeeds(
        vxMetersPerSecond,
        vyMetersPerSecond,
        omegaRadiansPerSecond
      ));
    }
  }

  @Override
  public void end(boolean interrupted) {
    // In the event this command stops, we don't want the motors to move
    drivetrain.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
