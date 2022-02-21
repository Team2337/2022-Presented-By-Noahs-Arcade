package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.ProfiledPointToPointCommand;
import frc.robot.commands.auto.commandGroups.AutoStopAllCommands;
import frc.robot.commands.auto.commandGroups.FirstMove;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class Pos1LeftFourBall extends SequentialCommandGroup {

  private Drivetrain drivetrain;
  private Delivery delivery;
  private Intake intake;
  private Kicker kicker;
  private Shooter shooter;

  public Pos1LeftFourBall(AutoDrive autoDrive, Delivery delivery, Drivetrain drivetrain, Heading heading, Intake intake, Kicker kicker, Shooter shooter) {
    this.drivetrain = drivetrain;

    addCommands(
      new FirstMove(Constants.Auto.kBallR1, autoDrive, delivery, drivetrain, heading, intake, kicker, shooter),
      new WaitCommand(1),
      new ProfiledPointToPointCommand(Constants.Auto.kBallR4Pickup, drivetrain::getTranslation, 3.0, 0.05, Units.inchesToMeters(120), 8, autoDrive, heading),
      new WaitCommand(1),
      new ProfiledPointToPointCommand(Constants.Auto.kBallR4, drivetrain::getTranslation, 3.0, 0.05, Units.inchesToMeters(120), 8, autoDrive, heading),
      new WaitCommand(1),
      new ProfiledPointToPointCommand(Constants.Auto.kFourBallShootPosition, drivetrain::getTranslation, 3.0, 0.05, Units.inchesToMeters(120), 8, autoDrive, heading),
      new WaitCommand(1),
      new AutoStopAllCommands(autoDrive, delivery, drivetrain, heading, intake, kicker, shooter)
    );
  }
  public void initialize() {
    drivetrain.resetPosition(new Pose2d(Constants.Auto.kPosition1LeftStart.toFieldCoordinate(), Rotation2d.fromDegrees(0)));
  }
}