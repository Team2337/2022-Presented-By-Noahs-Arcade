package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.Vision;

/**
 * Update the Heading subsystem to keep the robot facing a single point. By
 * default that point is the center of the field aka the Hub.
 */
public class HeadingToTargetCommand extends CommandBase {

  private final Translation2d targetMeters;
  private final Supplier<Translation2d> robotTranslationSupplier;
  private final Supplier<Boolean> overrideSupplier; // Moves us to "vision drive"
  private final Drivetrain drivetrain;
  private final Heading heading;
  private final Vision vision;

  public HeadingToTargetCommand(Supplier<Translation2d> robotTranslationSupplier, Supplier<Boolean> overrideSupplier, Drivetrain drivetrain, Heading heading, Vision vision) {
    this(Constants.kHub, robotTranslationSupplier, overrideSupplier, drivetrain, heading, vision);
  }

  public HeadingToTargetCommand(Translation2d targetMeters, Supplier<Translation2d> robotTranslationSupplier, Supplier<Boolean> overrideSupplier, Drivetrain drivetrain, Heading heading, Vision vision) {
    this.targetMeters = targetMeters;
    this.robotTranslationSupplier = robotTranslationSupplier;
    this.overrideSupplier = overrideSupplier;
    this.drivetrain = drivetrain;
    this.heading = heading;
    this.vision = vision;

    addRequirements(heading);
  }

  protected PolarCoordinate getRobotCoordinate() {
    // Find our robot polar coordinate relative to the target
    return PolarCoordinate.fromFieldCoordinate(
      robotTranslationSupplier.get(),
      targetMeters
    );
  }

  @Override
  public void execute() {
    if (overrideSupplier.get()) {
      // We're in "vision drive" - drive to pull the Limelight tx value to zero
      double towardsCenterDegrees = (vision.getTx() * -1);
      Rotation2d desiredRotation =  drivetrain.getGyroscopeRotation()
        .plus(Rotation2d.fromDegrees(towardsCenterDegrees));
      heading.setMaintainHeading(desiredRotation);
    } else {
      PolarCoordinate coordinate = getRobotCoordinate();
      // The angle is the angle outward from our center point. In order to face our center
      // point, we need to rotate our angle by 180 degrees.
      heading.setMaintainHeading(
        coordinate.getTheta().rotateBy(Rotation2d.fromDegrees(180))
      );
    }
  }
}