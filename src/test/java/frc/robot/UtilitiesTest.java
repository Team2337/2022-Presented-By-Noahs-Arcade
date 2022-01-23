package frc.robot;

import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

import edu.wpi.first.math.geometry.Rotation2d;

@RunWith(JUnit4.class)
public class UtilitiesTest {
  
  @Test
  public void testRelativeRotationFromAbsoluteRotation() {
    // Zero
    Assert.assertEquals(
      Utilities.relativeRotationFromAbsoluteRotation(
        Rotation2d.fromDegrees(0)
      ).getDegrees(), 0, 0.0);
    // Basic
    Assert.assertEquals(
      Utilities.relativeRotationFromAbsoluteRotation(
        Rotation2d.fromDegrees(90)
      ).getDegrees(), 90, 0.0);
    Assert.assertEquals(
      Utilities.relativeRotationFromAbsoluteRotation(
        Rotation2d.fromDegrees(-90)
      ).getDegrees(), -90, 0.0);
    // Bounds testing (no wrap)
    Assert.assertEquals(
      Utilities.relativeRotationFromAbsoluteRotation(
        Rotation2d.fromDegrees(180)
      ).getDegrees(), 180, 0.0);
    Assert.assertEquals(
      Utilities.relativeRotationFromAbsoluteRotation(
        Rotation2d.fromDegrees(-180)
      ).getDegrees(), -180, 0.0);
    // Wrap testing (basic)
    Assert.assertEquals(
      Utilities.relativeRotationFromAbsoluteRotation(
        Rotation2d.fromDegrees(181)
      ).getDegrees(), -179, 0.0);
    Assert.assertEquals(
      Utilities.relativeRotationFromAbsoluteRotation(
        Rotation2d.fromDegrees(-181)
      ).getDegrees(), 179, 0.0);
    // Bounds testing (wrap)
    Assert.assertEquals(
      Utilities.relativeRotationFromAbsoluteRotation(
        Rotation2d.fromDegrees(360)
      ).getDegrees(), 0, 0.0);
    Assert.assertEquals(
      Utilities.relativeRotationFromAbsoluteRotation(
        Rotation2d.fromDegrees(-360)
      ).getDegrees(), 0, 0.0);
    // Wrap testing (advanced)
    Assert.assertEquals(
      Utilities.relativeRotationFromAbsoluteRotation(
        Rotation2d.fromDegrees(405)
      ).getDegrees(), 45, 0.0);
    Assert.assertEquals(
      Utilities.relativeRotationFromAbsoluteRotation(
        Rotation2d.fromDegrees(-405)
      ).getDegrees(), -45, 0.0);
  }

}
