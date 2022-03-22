package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.EnableMaintainHeading;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Heading;

public class VisionToTarget extends SequentialCommandGroup {

  public VisionToTarget(Drivetrain drivetrain, Heading heading, Vision vision) {
    addCommands(
      new EnableMaintainHeading(heading),
      new LimeLightHeadingNeverEnding(drivetrain, heading, vision)
    );
  }
}