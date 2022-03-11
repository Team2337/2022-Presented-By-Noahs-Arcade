package frc.robot.commands.auto.commandGroups;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.ProfiledPointToPointCommand;
import frc.robot.commands.intake.AutoStartIntake;
import frc.robot.commands.shooter.StartShooterInstantCommand;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class FirstMove extends SequentialCommandGroup {

  private double forwardP = 1.5;
  private double strafeP = 0.025;
  private double forwardAcceleration = Units.inchesToMeters(45);
  private double strafeAcceleration = 6;

  public FirstMove(PolarCoordinate pickupLocation, AutoDrive autoDrive, Drivetrain drivetrain, Heading heading, Intake intake, Shooter shooter) {
    addCommands(
      new StartShooterInstantCommand(38.5, shooter),
      new ParallelCommandGroup(
        new AutoStartIntake(intake),
        new ProfiledPointToPointCommand(pickupLocation, drivetrain::getTranslation, forwardP, strafeP, forwardAcceleration, strafeAcceleration, autoDrive, heading).withTimeout(2)
        )
    );
  }

}