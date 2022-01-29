package frc.robot;

import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

import edu.wpi.first.math.geometry.Rotation2d;

@RunWith(JUnit4.class)
public class UtilitiesTest {
  
  @Test
  public void testConvertRotationToRelativeRotation() {
    // Zero
    Assert.assertEquals(
      0, Utilities.convertRotationToRelativeRotation(
        Rotation2d.fromDegrees(0)
      ).getDegrees(), 0.0);
    // Basic
    Assert.assertEquals(
      90, Utilities.convertRotationToRelativeRotation(
        Rotation2d.fromDegrees(90)
      ).getDegrees(), 0.0);
    Assert.assertEquals(
      -90, Utilities.convertRotationToRelativeRotation(
        Rotation2d.fromDegrees(-90)
      ).getDegrees(), 0.0);
    // Bounds testing (no wrap)
    Assert.assertEquals(
      180, Utilities.convertRotationToRelativeRotation(
        Rotation2d.fromDegrees(180)
      ).getDegrees(), 0.0);
    Assert.assertEquals(
      -180, Utilities.convertRotationToRelativeRotation(
        Rotation2d.fromDegrees(-180)
      ).getDegrees(), 0.0);
    // Wrap testing (basic)
    Assert.assertEquals(
      -179, Utilities.convertRotationToRelativeRotation(
        Rotation2d.fromDegrees(181)
      ).getDegrees(), 0.0);
    Assert.assertEquals(
      179, Utilities.convertRotationToRelativeRotation(
        Rotation2d.fromDegrees(-181)
      ).getDegrees(), 0.0);
    // Bounds testing (wrap)
    Assert.assertEquals(
      0, Utilities.convertRotationToRelativeRotation(
        Rotation2d.fromDegrees(360)
      ).getDegrees(), 0.0);
    Assert.assertEquals(
      0, Utilities.convertRotationToRelativeRotation(
        Rotation2d.fromDegrees(-360)
      ).getDegrees(), 0.0);
    // Wrap testing (advanced)
    Assert.assertEquals(
      45, Utilities.convertRotationToRelativeRotation(
        Rotation2d.fromDegrees(405)
      ).getDegrees(), 0.0);
    Assert.assertEquals(
      -45, Utilities.convertRotationToRelativeRotation(
        Rotation2d.fromDegrees(-405)
      ).getDegrees(), 0.0);
  }

}
