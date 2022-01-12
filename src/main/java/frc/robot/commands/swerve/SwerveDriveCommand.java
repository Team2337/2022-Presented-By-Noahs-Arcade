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
    Rotation2d desiredHeading = heading.getCurrentHeading();
    if (desiredHeading != null && Utilities.deadband(rotation, 0.1) == 0) {
      //desiredHeading = null;
    }

    if (isFieldOriented) {
      if (desiredHeading != null) {
        drivetrain.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
            vxMetersPerSecond,
            vyMetersPerSecond,
            omegaRadiansPerSecond,
            drivetrain.getGyroscopeRotation()
          ),
          desiredHeading
        ); 
      } else {
        drivetrain.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
            vxMetersPerSecond,
            vyMetersPerSecond,
            omegaRadiansPerSecond,
            drivetrain.getGyroscopeRotation()
          )
        ); 
      }
    } else {
      if (desiredHeading != null) {
        drivetrain.drive(
          new ChassisSpeeds(
            vxMetersPerSecond,
            vyMetersPerSecond,
            omegaRadiansPerSecond
          ),
          desiredHeading
        );
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
    // In the event this command stops, we don't want the motors to move
    drivetrain.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
