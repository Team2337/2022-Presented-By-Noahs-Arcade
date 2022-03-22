package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;
import frc.robot.subsystems.Vision;

/**
* Use the LimeLight to update the Heading subsystem.
*/
public class LimeLightHeadingNeverEnding extends CommandBase {

  private Drivetrain drivetrain;
  private Heading heading;
  private Vision vision;

  boolean shouldRotateToLimelightHeading = true;

  public LimeLightHeadingNeverEnding(Drivetrain drivetrain, Heading heading, Vision vision) {
    this.drivetrain = drivetrain;
    this.heading = heading;
    this.vision = vision;

    addRequirements(heading);
  }

  @Override
  public void initialize() {
    heading.enableMaintainHeading();
  }

  @Override
  public void execute() {
    if (vision.hasActiveTarget()) {
      double towardsCenterDegrees = (vision.getTx() * -1);
      Rotation2d desiredRotation =  drivetrain.getGyroscopeRotation()
        .plus(Rotation2d.fromDegrees(towardsCenterDegrees));
      heading.setMaintainHeading(desiredRotation);
    } else {
      shouldRotateToLimelightHeading = false;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
