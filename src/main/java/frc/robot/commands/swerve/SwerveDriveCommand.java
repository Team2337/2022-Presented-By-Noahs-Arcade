package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    boolean isFieldOriented = !controller.getLeftBumper();

    // If a driver-initiated rotationis provided, disable our rotation
    // controller to let the driver rotate freely.
    if (rotation != 0 && heading.isEnabled()) {
      heading.disableMaintainHeading();
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
      drivetrain.drive(new ChassisSpeeds(
        vxMetersPerSecond,
        vyMetersPerSecond,
        omegaRadiansPerSecond
      ));
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
