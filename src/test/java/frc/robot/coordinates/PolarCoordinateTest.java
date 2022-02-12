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
import frc.robot.Utilities;

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
    Assert.assertEquals(theta, coord.getTheta());
  }

  @Test
  public void testGetThetaBig() {
    Rotation2d theta = Rotation2d.fromDegrees(270);
    PolarCoordinate coord = new PolarCoordinate(0, theta);
    Assert.assertEquals(theta, coord.getTheta());
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
    // 260.25 degrees to radians -> 4.54221938
    // x = 3.8862 * cos(4.54221938) = -0.658126
    // y = 3.8862 * sin(4.54221938) = -3.83007
    Assert.assertEquals(
      -0.658126,
      coordinate.getX(),
      0.00001
    );
    Assert.assertEquals(
      -3.83007,
      coordinate.getY(),
      0.00001
    );
  }

  @Test
  public void testCartesianCoordinateTranslationRelativeRotation() {
    // Create a polar coordinate using our information from Ball 1 but with a 0, 0 center
    // Flip our 260.25 to use a relative rotation (-99.75)
    PolarCoordinate ballOne = new PolarCoordinate(
      Constants.Auto.kBall1.getRadiusMeters(),
      Utilities.convertRotationToRelativeRotation(Constants.Auto.kBall1.getTheta()),
      new Translation2d()
    );
    Assert.assertEquals(
      Rotation2d.fromDegrees(-99.75),
      ballOne.getTheta()
    );
    Translation2d coordinate = ballOne.toFieldCoordinate();
    // 153 inches to meters -> 3.8862
    // -99.75 degrees to radians -> -1.74096593
    // x = 3.8862 * cos(-1.74096593) = -0.658126
    // y = 3.8862 * sin(-1.74096593) = -3.83007
    Assert.assertEquals(
      -0.658126, coordinate.getX(), 0.00001
    );
    Assert.assertEquals(
      -3.83007, coordinate.getY(), 0.00001
    );
  }

  @Test
  public void testFieldCoordinateZero() {
    Translation2d fieldCenter = Constants.kHub;
    PolarCoordinate coord = new PolarCoordinate(0, new Rotation2d());
    Translation2d translation = coord.toFieldCoordinate();
    Assert.assertEquals(
      fieldCenter.getX(), translation.getX(), 0.00001
    );
    Assert.assertEquals(
      fieldCenter.getY(), translation.getY(), 0.00001
    );
  }

  @Test
  public void testFieldCoordinateBall1() {
    Translation2d fieldCenter = Constants.kHub;
    Translation2d location = Constants.Auto.kBall1.toFieldCoordinate();
    Assert.assertTrue(location.getX() < fieldCenter.getX());
    Assert.assertTrue(location.getY() < fieldCenter.getY());
    // Ball 1 is shifted RIGHT on our X axis from field center by 2.15921 ft
    Assert.assertEquals(
      fieldCenter.getX() - Units.feetToMeters(2.15921),
      location.getX(),
      0.00001
    );
    // Ball 1 is shifted DOWN on our Y axis from the field center by 12.56584 ft
    Assert.assertEquals(
      Units.metersToFeet(fieldCenter.getY() - Units.feetToMeters(12.56584)),
      Units.metersToFeet(location.getY()),
      0.00001
    );
  }

  @Test
  public void testFieldCoordinateBall5() {
    Translation2d fieldCenter = Constants.kHub;
    Translation2d location = Constants.Auto.kBall5.toFieldCoordinate();
    Assert.assertTrue(location.getX() < fieldCenter.getX());
    Assert.assertTrue(location.getY() > fieldCenter.getY());
    // Ball 5 is shifted LEFT on our X axis from field center by 10.78303 ft
    Assert.assertEquals(
      fieldCenter.getX() - Units.feetToMeters(10.78303),
      location.getX(),
      0.00001
    );
    // Ball 5 is shifted UP on our Y axis from the field center by -6.80359 ft
    Assert.assertEquals(
      fieldCenter.getY() + Units.feetToMeters(6.80359),
      location.getY(),
      0.00001
    );
  }

  @Test
  public void testFieldCoordinateLeft() {
    Translation2d fieldCenter = Constants.kHub;
    PolarCoordinate leftCoordinate = new PolarCoordinate(1.0, Rotation2d.fromDegrees(0));
    Translation2d leftFieldCoordinate = leftCoordinate.toFieldCoordinate();
    Assert.assertTrue(leftFieldCoordinate.getX() > fieldCenter.getX());
    Assert.assertTrue(leftFieldCoordinate.getY() == fieldCenter.getY());
  }

  @Test
  public void testFieldCoordinateUp() {
    Translation2d fieldCenter = Constants.kHub;
    PolarCoordinate upCoordinate = new PolarCoordinate(1.0, Rotation2d.fromDegrees(90));
    Translation2d upFieldCoordinate = upCoordinate.toFieldCoordinate();
    Assert.assertTrue(upFieldCoordinate.getX() == fieldCenter.getX());
    Assert.assertTrue(upFieldCoordinate.getY() > fieldCenter.getY());
  }

  @Test
  public void testFieldCoordinateRight() {
    Translation2d fieldCenter = Constants.kHub;
    PolarCoordinate rightCoordinate = new PolarCoordinate(1.0, Rotation2d.fromDegrees(180));
    Translation2d rightFieldCoordinate = rightCoordinate.toFieldCoordinate();
    Assert.assertTrue(rightFieldCoordinate.getX() < fieldCenter.getX());
    Assert.assertTrue(rightFieldCoordinate.getY() == fieldCenter.getY());
  }

  @Test
  public void testFieldCoordinateDown() {
    Translation2d fieldCenter = Constants.kHub;
    PolarCoordinate downCoordinate = new PolarCoordinate(1.0, Rotation2d.fromDegrees(270));
    Translation2d downFieldCoordinate = downCoordinate.toFieldCoordinate();
    Assert.assertTrue(downFieldCoordinate.getX() == fieldCenter.getX());
    Assert.assertTrue(downFieldCoordinate.getY() < fieldCenter.getY());
  }

  @Test
  public void testFieldCoordinateDownInverse() {
    Translation2d fieldCenter = Constants.kHub;
    PolarCoordinate downCoordinate = new PolarCoordinate(1.0, Rotation2d.fromDegrees(-90));
    Translation2d downFieldCoordinate = downCoordinate.toFieldCoordinate();
    Assert.assertTrue(downFieldCoordinate.getX() == fieldCenter.getX());
    Assert.assertTrue(downFieldCoordinate.getY() < fieldCenter.getY());
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
    PolarCoordinate[] balls = {Constants.Auto.kBall1, Constants.Auto.kBall2, Constants.Auto.kBall3, Constants.Auto.kBall5};
    for (PolarCoordinate b : balls) {
      // Round trip from field coordinate -> polar coordinate - make sure polar coordinates are the same
      PolarCoordinate ball = PolarCoordinate.fromFieldCoordinate(b.toFieldCoordinate());
      Assert.assertEquals(
        b.getRadiusMeters(),
        ball.getRadiusMeters(),
        0.0001
      );
      Assert.assertEquals(
        b.getTheta(),
        ball.getTheta()
      );
      assertEquals(
        b.getReferencePoint(),
        ball.getReferencePoint()
      );
    }
  }

  @Test
  public void testWithRelativeTheta() {
    int[] rotations = {45, 90, 135, 180, 270, 360, 450};
    int[] expectedRotations = {45, 90, 135, 180, -90, 0, 90};
    Assert.assertEquals(
      rotations.length,
      expectedRotations.length
    );

    for (int i = 0; i < rotations.length; i++) {
      int rotation = rotations[i];
      PolarCoordinate coordinate = new PolarCoordinate(10, Rotation2d.fromDegrees(rotation));
      double expectedRotation = expectedRotations[i];
      Assert.assertEquals(
        expectedRotation,
        coordinate.withRelativeTheta().getTheta().getDegrees(),
        0.00001
      );
    }
  }

}
