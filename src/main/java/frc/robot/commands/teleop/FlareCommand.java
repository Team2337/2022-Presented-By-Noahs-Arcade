package frc.robot.commands.teleop;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class FlareCommand extends SequentialCommandGroup {

  public FlareCommand(Supplier<Translation2d> robotTranslationSupplier, AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
    this(Constants.kHub, robotTranslationSupplier, autoDrive, drivetrain, heading);
  }

  public FlareCommand(Translation2d targetMeters, Supplier<Translation2d> robotTranslationSupplier, AutoDrive autoDrive, Drivetrain drivetrain, Heading heading) {
    addCommands(
      new HeadingToTargetSequenceCommand(targetMeters, robotTranslationSupplier, heading).withTimeout(1),
      new DistanceToTargetCommand(targetMeters, Units.inchesToMeters(10), robotTranslationSupplier, autoDrive, heading)
    );
  }

}
