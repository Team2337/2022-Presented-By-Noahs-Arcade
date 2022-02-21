package frc.robot.commands.auto.commandGroups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.ProfiledPointToPointCommand;
import frc.robot.commands.auto.commandGroups.FirstMove;
import frc.robot.commands.delivery.AutoStartDelivery;
import frc.robot.commands.delivery.AutoStopDelivery;
import frc.robot.commands.intake.AutoStartIntake;
import frc.robot.commands.intake.AutoStopIntake;
import frc.robot.commands.shooter.AutoKickerCommand;
import frc.robot.commands.shooter.AutoStartShooter;
import frc.robot.commands.shooter.AutoStopKicker;
import frc.robot.commands.shooter.AutoStopShooter;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class AutoStopAllCommands extends ParallelCommandGroup {

  private Drivetrain drivetrain;
  private Delivery delivery;
  private Intake intake;
  private Kicker kicker;
  private Shooter shooter;

  public AutoStopAllCommands(AutoDrive autoDrive, Delivery delivery, Drivetrain drivetrain, Heading heading, Intake intake, Kicker kicker, Shooter shooter) {
    this.drivetrain = drivetrain;

    addCommands(
      new AutoStopDelivery(delivery),
      new AutoStopIntake(intake),
      new AutoStopShooter(shooter),
      new AutoStopKicker(kicker) 
    );
  }
}