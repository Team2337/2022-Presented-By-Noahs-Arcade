package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.Heading;

/**
 * Update the Heading subsystem to keep the robot facing a single point. By
 * default that point is the center of the field aka the Hub.
 */
public class HeadingToTargetCommand extends CommandBase {

  private Translation2d target;
  private Supplier<Translation2d> robotTranslationSupplier;
  private Heading heading;

  public HeadingToTargetCommand(Supplier<Translation2d> robotTranslationSupplier, Heading heading) {
    this(Constants.kHub, robotTranslationSupplier, heading);
  }

  public HeadingToTargetCommand(Translation2d target, Supplier<Translation2d> robotTranslationSupplier, Heading heading) {
    this.target = target;
    this.robotTranslationSupplier = robotTranslationSupplier;
    this.heading = heading;

    addRequirements(heading);
  }

  @Override
  public void execute() {
    PolarCoordinate coordinate = PolarCoordinate.fromFieldCoordinate(
      robotTranslationSupplier.get(),
      target
    );
    heading.setMaintainHeading(coordinate.getTheta());
  }

}