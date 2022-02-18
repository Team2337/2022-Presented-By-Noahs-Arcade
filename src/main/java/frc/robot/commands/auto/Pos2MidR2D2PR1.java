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

public class Pos2MidR2D2PR1 extends SequentialCommandGroup {

  private Drivetrain drivetrain;

  public Pos2MidR2D2PR1(AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
    this.drivetrain = drivetrain;

    addCommands(
      new ProfiledPointToPointCommand(Constants.Auto.kBallR2Pickup, drivetrain::getTranslation, 3.0, 0.05, Units.inchesToMeters(120), 8, autoDrive, heading),
      new WaitCommand(1),
      new ProfiledPointToPointCommand(Constants.Auto.kBallR2, drivetrain::getTranslation, 3.0, 0.05, Units.inchesToMeters(120), 8, autoDrive, heading),
      new WaitCommand(1),
      new ProfiledPointToPointCommand(Constants.Auto.kBallD2, drivetrain::getTranslation, 3.0, 0.05, Units.inchesToMeters(120), 8, autoDrive, heading),
      new WaitCommand(1),
      new ProfiledPointToPointCommand(Constants.Auto.kBallR1Pickup, drivetrain::getTranslation, 3.0, 0.05, Units.inchesToMeters(120), 8, autoDrive, heading),
      new WaitCommand(1),
      new ProfiledPointToPointCommand(Constants.Auto.kBallR1, drivetrain::getTranslation, 3.0, 0.05, Units.inchesToMeters(120), 8, autoDrive, heading),
      new WaitCommand(1)
    );
  }
  public void initialize() {
    drivetrain.resetPosition(new Pose2d(Constants.Auto.kPosition2MiddleStart.toFieldCoordinate(), Rotation2d.fromDegrees(0)));
  }
}