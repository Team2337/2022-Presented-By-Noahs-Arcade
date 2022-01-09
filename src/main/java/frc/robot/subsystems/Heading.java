package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Heading subsystem is a "fake" subsystem that is used to coordinate
 * mutual exclusivity around maintaining the heading of the robot.
 * Ex: Commands that request the robot to be set to a specific heading
 * should require the Heading subsystem. Only one command should be
 * updating the robot's heading at a time.
 */
public class Heading extends SubsystemBase {

  private Rotation2d nextHeading;
  private Rotation2d currentHeading;

  public void setCurrentHeading(Rotation2d currentHeading) {
    this.currentHeading = currentHeading;
  }

  public Rotation2d getCurrentHeading() {
    return currentHeading;
  }

  public void enqueueHeading(Rotation2d nextHeading) {
    this.nextHeading = nextHeading;
  }
  
  public Rotation2d dequeueHeading() {
    Rotation2d heading = this.nextHeading;
    this.nextHeading = null;
    return heading;
  }

}
