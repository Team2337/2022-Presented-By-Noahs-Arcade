package frc.robot.helpers;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Used for an Operator to enqueue a new heading for the
 * robot. Heading is used by the Driver later to execute
 * on the requested heading.
 */
public class HeadingQueue {
  
  private Rotation2d heading;

  public void enqueueHeading(Rotation2d heading) {
    this.heading = heading;
  }
  
  public Rotation2d dequeueHeading() {
    Rotation2d heading = this.heading;
    this.heading = null;
    return heading;
  }

}
