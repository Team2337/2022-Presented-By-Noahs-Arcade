package frc.robot.coordinates;

import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

@RunWith(JUnit4.class)
public class PolarCoordinateTest {

  @Test
  public void testTheta() {
    Rotation2d theta = Rotation2d.fromDegrees(45);
    PolarCoordinate coord = new PolarCoordinate(theta, 0);
    Assert.assertEquals(
      coord.getTheta(), theta
    );
  }

  @Test
  public void testDistance() {
    double distance = 137;
    PolarCoordinate coord = new PolarCoordinate(new Rotation2d(), distance);
    Assert.assertEquals(
      coord.getDistanceMeters(), distance, 0.0
    );
  }

  @Test
  public void testTranslationMeters() {
    Translation2d center = new Translation2d(0, 0);
    PolarCoordinate ballOne = new PolarCoordinate(Rotation2d.fromDegrees(80.3), Units.inchesToMeters(153));
    Translation2d ballOneTranslation = ballOne.toTranslationMeters(center);
    // 153 inches to meters -> 3.8862
    // 80.3 degrees to radians -> 1.401499
    // x = 3.8862 * cos(1.401499) = 0.65478491833
    // y = 3.8862 * sin(1.401499) = 3.83064056663
    Assert.assertEquals(
      ballOneTranslation.getX(), 0.65478491833, 0.00001
    );
    Assert.assertEquals(
      ballOneTranslation.getY(), 3.83064056663, 0.00001
    );
  }


  @Test
  public void testTranslationMetersFieldCentricZero() {
    Translation2d center = new Translation2d(13.5, 27);
    PolarCoordinate coord = new PolarCoordinate(new Rotation2d(), 0);
    Translation2d translation = coord.toTranslationMeters(center);
    Assert.assertEquals(
        translation.getX(), Units.feetToMeters(27), 0.00001
    );
    Assert.assertEquals(
        translation.getY(), Units.feetToMeters(13.5), 0.00001
    );
  }

  @Test
  public void testTranslationMetersFieldCentric() {
    Translation2d center = new Translation2d(13.5, 27);
    PolarCoordinate ballOne = new PolarCoordinate(Rotation2d.fromDegrees(80.3), Units.inchesToMeters(153));
    Translation2d ballOneTranslation = ballOne.toTranslationMeters(center);
    // 153 inches to meters -> 3.8862
    // 80.3 degrees to radians -> 1.401499
    // 13.5 ft to meters -> 4.1148
    // 27 ft to meters -> 8.2296
    // x = 3.8862 * cos(1.401499) = 0.65478491833 + 4.1148
    // y = 3.8862 * sin(1.401499) = 3.83064056663 + 8.2296
    Assert.assertEquals(
      ballOneTranslation.getX(), 0.65478491833 + 8.2296, 0.00001
    );
    Assert.assertEquals(
      ballOneTranslation.getY(), 3.83064056663 + 4.1148, 0.00001
    );
  }


}
