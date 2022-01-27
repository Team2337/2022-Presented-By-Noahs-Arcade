package frc.robot.coordinates;

import static org.junit.Assert.assertEquals;

import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

@RunWith(JUnit4.class)
public class PolarCoordinateTest {

  @Test
  public void testGetDistance() {
    double distance = 137;
    PolarCoordinate coord = new PolarCoordinate(distance, new Rotation2d());
    Assert.assertEquals(distance, coord.getRadiusMeters(), 0.0);
  }

  @Test
  public void testGetTheta() {
    Rotation2d theta = Rotation2d.fromDegrees(45);
    PolarCoordinate coord = new PolarCoordinate(0, theta);
    Assert.assertEquals(
      theta, coord.getTheta()
    );
  }

  @Test
  public void testCartesianCoordinateTranslation() {
    // Create a polar coordinate using our information from Ball 1 but with a 0, 0 center
    PolarCoordinate ballOne = new PolarCoordinate(
      Constants.Auto.kBall1.getRadiusMeters(),
      Constants.Auto.kBall1.getTheta(),
      new Translation2d()
    );
    Translation2d coordinate = ballOne.toFieldCoordinate();
    // 153 inches to meters -> 3.8862
    // 9.75 degrees to radians -> 0.17016960206944712
    // x = 3.8862 * cos(0.17016960206944712) = 3.83007
    // y = 3.8862 * sin(0.17016960206944712) = 0.658126
    // Note since it's a field coordinate, we swap our X and Y
    Assert.assertEquals(
      3.83007, coordinate.getY(), 0.00001
    );
    Assert.assertEquals(
      0.658126, coordinate.getX(), 0.00001
    );
  }

  @Test
  public void testFieldCoordinateZero() {
    Translation2d fieldCenter = Constants.kHub;
    PolarCoordinate coord = new PolarCoordinate(0, new Rotation2d());
    Translation2d translation = coord.toFieldCoordinate();
    Assert.assertEquals(
        translation.getX(), fieldCenter.getX(), 0.00001
    );
    Assert.assertEquals(
        translation.getY(), fieldCenter.getY(), 0.00001
    );
  }

  @Test
  public void testFieldCoordinateBall1() {
    Translation2d fieldCenter = Constants.kHub;
    Translation2d location = Constants.Auto.kBall1.toFieldCoordinate();
    // This is a rough test - we don't actually need to know that Ball 1
    // is right on the X, Y - the important part is that it's +, + to the
    // center of the field.
    Assert.assertTrue(location.getX() > fieldCenter.getX());
    Assert.assertTrue(location.getY() > fieldCenter.getY());
    // Ball 1 is shifted UP from field center by 2.15921 ft
    Assert.assertEquals(
      fieldCenter.getX() + Units.feetToMeters(2.15921),
      location.getX(),
      0.00001
    );
    // Ball 1 is shifted LEFT from the field center by 12.56584 ft
    Assert.assertEquals(
      fieldCenter.getY() + Units.feetToMeters(12.56584),
      location.getY(),
      0.00001
    );
  }

  @Test
  public void testFieldCoordinateBall5() {
    Translation2d fieldCenter = Constants.kHub;
    Translation2d location = Constants.Auto.kBall5.toFieldCoordinate();
    // This is a rough test - we don't actually need to know that Ball 5
    // is right on the X, Y - the important part is that it's +, - to the
    // center of the field.
    Assert.assertTrue(location.getX() > fieldCenter.getX());
    Assert.assertTrue(location.getY() < fieldCenter.getY());
    // Ball 5 is shifted UP from field center by 10.78303 ft
    Assert.assertEquals(
      fieldCenter.getX() + Units.feetToMeters(10.78303),
      location.getX(),
      0.00001
    );
    // Ball 5 is shifted RIGHT from the field center by -6.80359 ft
    Assert.assertEquals(
      fieldCenter.getY() + Units.feetToMeters(-6.80359),
      location.getY(),
      0.00001
    );
  }

  @Test
  public void testFieldCoordinateLeft() {
    Translation2d fieldCenter = Constants.kHub;
    PolarCoordinate leftCoordinate = new PolarCoordinate(1.0, Rotation2d.fromDegrees(0));
    Translation2d leftFieldCoordinate = leftCoordinate.toFieldCoordinate();
    Assert.assertTrue(leftFieldCoordinate.getX() == fieldCenter.getX());
    Assert.assertTrue(leftFieldCoordinate.getY() > fieldCenter.getY());
  }

  @Test
  public void testFieldCoordinateForward() {
    Translation2d fieldCenter = Constants.kHub;
    PolarCoordinate forwardCoordinate = new PolarCoordinate(1.0, Rotation2d.fromDegrees(90));
    Translation2d forwardFieldCoordinate = forwardCoordinate.toFieldCoordinate();
    Assert.assertTrue(forwardFieldCoordinate.getX() > fieldCenter.getX());
    Assert.assertTrue(forwardFieldCoordinate.getY() == fieldCenter.getY());
  }

  @Test
  public void testFieldCoordinateRight() {
    Translation2d fieldCenter = Constants.kHub;
    PolarCoordinate rightCoordinate = new PolarCoordinate(1.0, Rotation2d.fromDegrees(180));
    Translation2d rightFieldCoordinate = rightCoordinate.toFieldCoordinate();
    Assert.assertTrue(rightFieldCoordinate.getX() == fieldCenter.getX());
    Assert.assertTrue(rightFieldCoordinate.getY() < fieldCenter.getY());
  }

  @Test
  public void testFieldCoordinateBackwards() {
    Translation2d fieldCenter = Constants.kHub;
    PolarCoordinate backwardsCoordinate = new PolarCoordinate(1.0, Rotation2d.fromDegrees(270));
    Translation2d backwardsFieldCoordinate = backwardsCoordinate.toFieldCoordinate();
    Assert.assertTrue(backwardsFieldCoordinate.getX() < fieldCenter.getX());
    Assert.assertTrue(backwardsFieldCoordinate.getY() == fieldCenter.getY());
  }

  @Test
  public void testFieldCoordinateBackwardsInverse() {
    Translation2d fieldCenter = Constants.kHub;
    PolarCoordinate backwardsCoordinate = new PolarCoordinate(1.0, Rotation2d.fromDegrees(-90));
    Translation2d backwardsFieldCoordinate = backwardsCoordinate.toFieldCoordinate();
    Assert.assertTrue(backwardsFieldCoordinate.getX() < fieldCenter.getX());
    Assert.assertTrue(backwardsFieldCoordinate.getY() == fieldCenter.getY());
  }

  @Test
  public void testFieldCoordinateInverse() {
    // Going -90 degrees should be the same as going 270 degrees
    PolarCoordinate coordinate = new PolarCoordinate(1.0, Rotation2d.fromDegrees(270));
    PolarCoordinate inverseCoordinate = new PolarCoordinate(1.0, Rotation2d.fromDegrees(-90));
    Translation2d fieldCoordinate = coordinate.toFieldCoordinate();
    Translation2d inverseFieldCoordinate = inverseCoordinate.toFieldCoordinate();
    Assert.assertTrue(fieldCoordinate.getX() == inverseFieldCoordinate.getX());
    Assert.assertTrue(fieldCoordinate.getY() == inverseFieldCoordinate.getY());
  }

  @Test
  public void testFromFieldCoordinate() {
    PolarCoordinate ball = PolarCoordinate.fromFieldCoordinate(Constants.Auto.kBall1.toFieldCoordinate());
    Assert.assertEquals(
      ball.getRadiusMeters(),
      Constants.Auto.kBall1.getRadiusMeters(),
      0.0001
    );
    Assert.assertEquals(
      ball.getTheta(),
      Constants.Auto.kBall1.getTheta()
    );
    assertEquals(
      ball.getReferencePoint(),
      Constants.Auto.kBall1.getReferencePoint()
    );
  }

}
