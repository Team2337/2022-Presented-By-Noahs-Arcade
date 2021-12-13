package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Utilities;
import frc.robot.subsystems.Drivetrain;


public class SwerveDriveCommand extends CommandBase {

  private final XboxController controller;

  private final Drivetrain drivetrain;

  /**
   * Command running the swerve calculations with the joystick
   *
   * @param subsystem - SwerveDrivetrain subsystem object
   */
  public SwerveDriveCommand(XboxController controller, Drivetrain drivetrain) {
    this.controller = controller;
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
