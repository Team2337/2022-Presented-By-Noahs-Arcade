package frc.robot.subsystems;

import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.Heading;

@RunWith(JUnit4.class)
public class HeadingTest {
  
  double rotationDegrees = -175.0;

  @Test
  public void test() {
    Heading heading = new Heading(() -> {
      // Go from something like 185 -> -175 and -185 -> 175
      double rotationDegrees = this.rotationDegrees;
      if (rotationDegrees > 180) {
        rotationDegrees -= 360;
      } else if (rotationDegrees < -180) {
        rotationDegrees += 360;
      }
      return Rotation2d.fromDegrees(rotationDegrees);
    });
    heading.setMaintainHeading(Rotation2d.fromDegrees(45));
    
    double rotation = -1;
    while (rotation != 0.0) {
      System.out.println(rotationDegrees);
      rotation = heading.calculateRotation();
      double omegaRadiansPerSecond = rotation * Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
      rotationDegrees += omegaRadiansPerSecond;
    }
    System.out.println("Reached heading within 1 degree");
  }
}
