package frc.robot.commands;

import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.Heading;

@RunWith(JUnit4.class)
public class HeadingToTargetCommandTest {

  private static final PolarCoordinate BallR1 = new PolarCoordinate(
    Units.inchesToMeters(153),
    Rotation2d.fromDegrees(147.75)
  );
  private static final PolarCoordinate BallR2 = new PolarCoordinate(
    Units.inchesToMeters(153),
    Rotation2d.fromDegrees(215.25)
  );
  private static final PolarCoordinate BallR3 = new PolarCoordinate(
    Units.inchesToMeters(153),
    Rotation2d.fromDegrees(260.25)
  );

  private Rotation2d headingToTargetMaintainHeading(PolarCoordinate coordinate) {
    Heading heading = new Heading(() -> new Rotation2d(), () -> false);
    Translation2d target = coordinate.toFieldCoordinate();
    HeadingToTargetCommand command = new HeadingToTargetCommand(
      () -> target,
      heading
    );
    command.execute();
    return heading.getMaintainHeading();
  }

  @Test
  public void testHeadingToTargetBallR3Hub() {
    Rotation2d maintainHeading = headingToTargetMaintainHeading(BallR3);

    // When we're at Ball 3, heading should be 80.25
    Assert.assertEquals(
      Rotation2d.fromDegrees(80.25),
      maintainHeading
    );
  }

  @Test
  public void testHeadingToTargetBallR2Hub() {
    Rotation2d maintainHeading = headingToTargetMaintainHeading(BallR2);

    // When we're at Ball R2, heading should be 35.25
    Assert.assertEquals(
      Rotation2d.fromDegrees(35.25),
      maintainHeading
    );
  }

  @Test
  public void testHeadingToTargetBallR1Hub() {
    Rotation2d maintainHeading = headingToTargetMaintainHeading(BallR1);

    // When we're at Ball R1, heading should be -32.25
    Assert.assertEquals(
      Rotation2d.fromDegrees(-32.25),
      maintainHeading
    );
  }

  @Test
  public void testHeadingToTargetOpposingBallR3Hub() {
    // Flip theta by 180 to get position on other side of field
    Rotation2d maintainHeading = headingToTargetMaintainHeading(
      BallR3.rotateBy(Rotation2d.fromDegrees(180))
    );

    // When we're at Opposing Ball R3, heading should be -99.75
    Assert.assertEquals(
      Rotation2d.fromDegrees(-99.75),
      maintainHeading
    );
  }

  @Test
  public void testHeadingToTargetOpposingBallR2Hub() {
    // Flip theta by 180 to get position on other side of field
    Rotation2d maintainHeading = headingToTargetMaintainHeading(
      BallR2.rotateBy(Rotation2d.fromDegrees(180))
    );

    // When we're at Opposing Ball R2, heading should be -144.75
    Assert.assertEquals(
      Rotation2d.fromDegrees(-144.75),
      maintainHeading
    );
  }

  @Test
  public void testHeadingToTargetOpposingBallR1Hub() {
    // Flip theta by 180 to get position on other side of field
    Rotation2d maintainHeading = headingToTargetMaintainHeading(
      BallR1.rotateBy(Rotation2d.fromDegrees(180))
    );

    // When we're at Opposing Ball R1, heading should be 147.75
    Assert.assertEquals(
      Rotation2d.fromDegrees(147.75),
      maintainHeading
    );
  }

}
