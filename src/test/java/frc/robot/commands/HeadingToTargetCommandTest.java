package frc.robot.commands;

import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.Heading;

@RunWith(JUnit4.class)
public class HeadingToTargetCommandTest {

  private Rotation2d headingToTargetMaintainHeading(PolarCoordinate coordinate) {
    Heading heading = new Heading(() -> new Rotation2d());
    Translation2d target = coordinate.toFieldCoordinate();
    HeadingToTargetCommand command = new HeadingToTargetCommand(
      () -> target,
      heading
    );
    command.execute();
    return heading.getMaintainHeading();
  }

  @Test
  public void testHeadingToTargetBall1Hub() {
    Rotation2d maintainHeading = headingToTargetMaintainHeading(Constants.Auto.kBallR3);

    // When we're at Ball 1, heading should be 80.25
    Assert.assertEquals(
      Rotation2d.fromDegrees(80.25),
      maintainHeading
    );
  }

  @Test
  public void testHeadingToTargetBall2Hub() {
    Rotation2d maintainHeading = headingToTargetMaintainHeading(Constants.Auto.kBallR2);

    // When we're at Ball 2, heading should be 35.25
    Assert.assertEquals(
      Rotation2d.fromDegrees(35.25),
      maintainHeading
    );
  }

  @Test
  public void testHeadingToTargetBall5Hub() {
    Rotation2d maintainHeading = headingToTargetMaintainHeading(Constants.Auto.kBallR1);

    // When we're at Ball 5, heading should be -32.25
    Assert.assertEquals(
      Rotation2d.fromDegrees(-32.25),
      maintainHeading
    );
  }

  @Test
  public void testHeadingToTargetOpposingBall1Hub() {
    // Flip theta by 180 to get position on other side of field
    Rotation2d maintainHeading = headingToTargetMaintainHeading(
      Constants.Auto.kBallR3.rotateBy(Rotation2d.fromDegrees(180))
    );

    // When we're at Opposing Ball 1, heading should be -99.75
    Assert.assertEquals(
      Rotation2d.fromDegrees(-99.75),
      maintainHeading
    );
  }

  @Test
  public void testHeadingToTargetOpposingBall2Hub() {
    // Flip theta by 180 to get position on other side of field
    Rotation2d maintainHeading = headingToTargetMaintainHeading(
      Constants.Auto.kBallR2.rotateBy(Rotation2d.fromDegrees(180))
    );

    // When we're at Opposing Ball 2, heading should be -144.75
    Assert.assertEquals(
      Rotation2d.fromDegrees(-144.75),
      maintainHeading
    );
  }

  @Test
  public void testHeadingToTargetOpposingBall5Hub() {
    // Flip theta by 180 to get position on other side of field
    Rotation2d maintainHeading = headingToTargetMaintainHeading(
      Constants.Auto.kBallR1.rotateBy(Rotation2d.fromDegrees(180))
    );

    // When we're at Opposing Ball 5, heading should be 147.75
    Assert.assertEquals(
      Rotation2d.fromDegrees(147.75),
      maintainHeading
    );
  }

}
