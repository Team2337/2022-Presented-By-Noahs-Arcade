package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.commandGroups.AutoStopAllCommands;
import frc.robot.commands.auto.commandGroups.FirstMove;
import frc.robot.commands.delivery.AutoStartDelivery;
import frc.robot.commands.shooter.AutoKickerCommand;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class Pos2MidTwoBall extends SequentialCommandGroup {

  private Drivetrain drivetrain;
  private Delivery delivery;
  private Intake intake;
  private Kicker kicker;
  private Shooter shooter;

  public Pos2MidTwoBall(AutoDrive autoDrive, Delivery delivery, Drivetrain drivetrain, Heading heading, Intake intake, Kicker kicker, Shooter shooter) {
    this.drivetrain = drivetrain;

    addCommands(
      new FirstMove(Constants.Auto.kBallR2RunOver, autoDrive, drivetrain, heading, intake, shooter),
      new AutoKickerCommand(kicker, 0).withTimeout(0.75),
      new WaitCommand(1),
      new ParallelCommandGroup(
        new AutoStartDelivery(delivery).withTimeout(1),
        new AutoKickerCommand(kicker, 0).withTimeout(1)
      ),    
      new ProfiledPointToPointCommand(Constants.Auto.kBallR2Escape, drivetrain::getTranslation, autoDrive, heading),
      new AutoStopAllCommands(delivery, intake, kicker, shooter)
    );
  }
}