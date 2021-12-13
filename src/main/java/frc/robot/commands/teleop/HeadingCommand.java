package frc.robot.commands.teleop;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helpers.HeadingQueue;
import frc.robot.subsystems.Heading;

/**
 * Tell the robot to maintain a heading. The heading may be a
 * static heading (ex: 90 degrees) or dynamic (ex: continiously
 * provided by a LimeLight or vision system).
 */
public class HeadingCommand extends CommandBase {
  
  private HeadingQueue headingQueue;
  private Heading heading;

  public HeadingCommand(HeadingQueue headingQueue, Heading heading) {
    this.headingQueue = headingQueue;
    this.heading = heading;
    
    addRequirements(heading);
  }

  @Override
  public void initialize() {
    updateDesiredHeading();
  }

  @Override
  public void execute() {
    updateDesiredHeading();
  }

  private void updateDesiredHeading() {
    Rotation2d desiredHeading = headingQueue.dequeueHeading();
    if (desiredHeading == null) {
      return;
    }
    heading.setDesiredHeading(desiredHeading);
  }

  @Override
  public void end(boolean interrupted) {
    heading.setDesiredHeading(null);
  }

}
