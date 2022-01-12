package frc.robot.commands.teleop;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Heading;

/**
 * Tell the robot to maintain a heading. The heading may be a
 * static heading (ex: 90 degrees) or dynamic (ex: continiously
 * provided by a LimeLight or vision system).
 */
public class HeadingCommand extends InstantCommand {
  
  private Heading heading;

  public HeadingCommand(Heading heading) {
    this.heading = heading;
    
    addRequirements(heading);
  }

  @Override
  public void initialize() {
    updateDesiredHeading();
  }

  private void updateDesiredHeading() {
    Rotation2d desiredHeading = heading.dequeueHeading();
    if (desiredHeading == null) {
      return;
    }
    heading.setCurrentHeading(desiredHeading);
  }

}
