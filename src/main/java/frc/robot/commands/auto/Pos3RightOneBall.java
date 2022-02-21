package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.ProfiledPointToPointCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class Pos3RightOneBall extends SequentialCommandGroup {

  private Drivetrain drivetrain;

  public Pos3RightOneBall(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
    this.drivetrain = drivetrain;

    addCommands(
      new ProfiledPointToPointCommand(Constants.Auto.kBallR3Pickup, drivetrain::getTranslation, 2.5, 0.05, Units.inchesToMeters(120), 8, autoDrive, heading),
      new WaitCommand(1)
    );
  }
}