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

  Rotation2d desiredHeading;

  public void setDesiredHeading(Rotation2d desiredHeading) {
    this.desiredHeading = desiredHeading;
  }

  public Rotation2d getDesiredHeading() {
    return desiredHeading;
  }

}
