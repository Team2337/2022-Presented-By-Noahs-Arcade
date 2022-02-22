package frc.robot.commands.auto.commandGroups;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.ProfiledPointToPointCommand;
import frc.robot.commands.intake.AutoStartIntake;
import frc.robot.commands.shooter.AutoStartShooter;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class FirstMove extends SequentialCommandGroup {

  private Drivetrain drivetrain;
  private Shooter shooter;
  private Intake intake;
  private double forwardP = 2.5;
  private double strafeP = 0.05;
  private double forwardAcceleration = Units.inchesToMeters(120);
  private double strafeAcceleration = 12;

  

  public FirstMove(PolarCoordinate pickupLocation, AutoDrive autoDrive, Drivetrain drivetrain, Heading heading, Intake intake, Shooter shooter) {
    this.drivetrain = drivetrain;

    addCommands(
      new AutoStartShooter(shooter, 38.5),
      new ParallelCommandGroup(
        new AutoStartIntake(intake),
        new ProfiledPointToPointCommand(pickupLocation, drivetrain::getTranslation, forwardP, strafeP, forwardAcceleration, strafeAcceleration, autoDrive, heading).withTimeout(2)
        ) 
    );
  }
}