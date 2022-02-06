package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.teleop.ProfiledPointToPointCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class Top3Ball extends SequentialCommandGroup {

  private Drivetrain drivetrain;

  public Top3Ball(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
    this.drivetrain = drivetrain;

    addCommands(
      new ProfiledPointToPointCommand(Constants.Auto.kBall1Pickup, drivetrain::getPose, heading, autoDrive, 3.0, 0.05, Units.inchesToMeters(120), 8),
      new WaitCommand(1),
      new ProfiledPointToPointCommand(Constants.Auto.kBall2Pickup, drivetrain::getPose, heading, autoDrive, 3.0, 0.05, Units.inchesToMeters(120), 15),
      new WaitCommand(1),
      new ProfiledPointToPointCommand(Constants.Auto.kBall3Pickup, drivetrain::getPose, heading, autoDrive, 3.0, 0.025, Units.inchesToMeters(120), 8),
      new WaitCommand(1),
      new ProfiledPointToPointCommand(Constants.Auto.kFiveBallShootPosition, drivetrain::getPose, heading, autoDrive, 3.0, 0.01, Units.inchesToMeters(120), 8)
    );
  }

  @Override
  public void initialize() {
    super.initialize();

    // TODO: This angle is wrong - we probably won't actually be at this angle
    drivetrain.resetPosition(new Pose2d(Constants.Auto.startTopPosition.toFieldCoordinate(), Rotation2d.fromDegrees(0)));
  }

}
