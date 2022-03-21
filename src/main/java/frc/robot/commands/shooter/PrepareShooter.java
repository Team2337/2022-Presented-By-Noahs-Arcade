package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.kicker.ReverseKickerCommand;
import frc.robot.commands.swerve.EnableMaintainHeading;
import frc.robot.commands.vision.LimeLightHeadingCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.Kicker;

public class PrepareShooter extends SequentialCommandGroup {

  public PrepareShooter(Supplier<Translation2d> translationSupplier, Supplier<Boolean> overrideSupplier, Drivetrain drivetrain, Heading heading, Kicker kicker, Shooter shooter, Vision vision) {
    addCommands(
      new ReverseStopShooterCommand(shooter).withTimeout(0.2),
      new ReverseKickerCommand(kicker).withTimeout(0.2),
      new WaitCommand(0.4),
      new StartShooterUpToSpeedDistanceCommand(translationSupplier,  overrideSupplier, shooter),
      new EnableMaintainHeading(heading),
      new LimeLightHeadingCommand(drivetrain, heading, vision)
    );
  }
}