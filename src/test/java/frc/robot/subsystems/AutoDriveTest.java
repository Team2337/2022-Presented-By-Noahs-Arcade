package frc.robot.subsystems;

import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

import edu.wpi.first.math.geometry.Rotation2d;

@RunWith(JUnit4.class)
public class AutoDriveTest {

  @Test
  public void testAutoDriveScaling() {
    AutoDrive.State state = new AutoDrive.State(
      0.5,
      0.5
    );
    // No scaling should occur - values add to one
    Assert.assertEquals(0.5, state.forward, 0.000001);
    Assert.assertEquals(0.5, state.strafe, 0.000001);
  }

  @Test
  public void testAutoDriveScalingPartial() {
    AutoDrive.State state = new AutoDrive.State(
      1.0,
      0.5
    );
    // Scaling should occur - forward should be prioritized
    Assert.assertEquals(0.666666, state.forward, 0.000001);
    Assert.assertEquals(0.333333, state.strafe, 0.000001);
  }

  @Test
  public void testAutoDriveScalingLarge() {
    AutoDrive.State state = new AutoDrive.State(
      3.0,
      3.0
    );
    // Giving each value a 3 should scale each value to 0.5 (adds to 1)
    Assert.assertEquals(0.5, state.forward, 0.000001);
    Assert.assertEquals(0.5, state.strafe, 0.000001);
  }

  @Test
  public void testAutoDriveScalingNegative() {
    AutoDrive.State state = new AutoDrive.State(
      -0.5,
      -0.5
    );
    // No scaling should occur - values add to -1
    Assert.assertEquals(-0.5, state.forward, 0.000001);
    Assert.assertEquals(-0.5, state.strafe, 0.000001);
  }

  @Test
  public void testAutoDriveScalingNegativePartial() {
    AutoDrive.State state = new AutoDrive.State(
      -0.5,
      -1.0
    );
    // Scaling should occur - strafe should be prioritized
    Assert.assertEquals(-0.3333333, state.forward, 0.000001);
    Assert.assertEquals(-0.6666666, state.strafe, 0.000001);
  }

  @Test
  public void testAutoDriveScalingNegativeLarge() {
    AutoDrive.State state = new AutoDrive.State(
      -3.0,
      -3.0
    );
    // Giving each value a -3 should scale each value to -0.5 (adds to -1)
    Assert.assertEquals(-0.5, state.forward, 0.000001);
    Assert.assertEquals(-0.5, state.strafe, 0.000001);
  }

  @Test
  public void testAutoDriveScalingNegativePositive() {
    AutoDrive.State state = new AutoDrive.State(
      1.0,
      -1.0
    );
    // Scaling should occur - values should keep their signs
    Assert.assertEquals(0.5, state.forward, 0.000001);
    Assert.assertEquals(-0.5, state.strafe, 0.000001);
  }

  @Test
  public void testToFieldRelativeState180() {
    // A robot facing the driver station being told
    // to move "forward" and "left" should have a negative
    // field-oriented forward and strafe
    AutoDrive.State robotState = new AutoDrive.State(
      0.6,
      0.4
    );
    AutoDrive.State fieldOrientedState = robotState.toFieldRelativeState(
      Rotation2d.fromDegrees(180)
    );
    Assert.assertEquals(-0.6, fieldOrientedState.forward, 0.000001);
    Assert.assertEquals(-0.4, fieldOrientedState.strafe, 0.000001);
  }

  @Test
  public void testToFieldRelativeState90() {
    // A robot facing the left side of the field being told
    // to move "forward" and "left" should have a field-oriented
    // values pulling it towards the left side of the field and
    // to the alliance wall
    AutoDrive.State robotState = new AutoDrive.State(
      0.6,
      0.4
    );
    AutoDrive.State fieldOrientedState = robotState.toFieldRelativeState(
      Rotation2d.fromDegrees(90)
    );
    Assert.assertEquals(-0.4, fieldOrientedState.forward, 0.000001);
    Assert.assertEquals(0.6, fieldOrientedState.strafe, 0.000001);
  }

  @Test
  public void testToFieldRelativeState45() {
    // A robot on a 45 degree angle being told to move "forward"
    // and "left" should move the robot slightly in one direction
    // and not at all in the other - since one axis move will drive
    // the robot in both directions.
    AutoDrive.State robotState = new AutoDrive.State(
      0.5,
      0.5
    );
    AutoDrive.State fieldOrientedState = robotState.toFieldRelativeState(
      Rotation2d.fromDegrees(45)
    );
    Assert.assertEquals(0.0, fieldOrientedState.forward, 0.000001);
    Assert.assertEquals(0.7071067812, fieldOrientedState.strafe, 0.000001);
  }

  @Test
  public void testToFieldRelativeStateNegative45() {
    // A robot on a -45 degree angle being told to move "forward"
    // and "left" should move the robot slightly in one direction
    // and not at all in the other - since one axis move will drive
    // the robot in both directions.
    AutoDrive.State robotState = new AutoDrive.State(
      0.5,
      0.5
    );
    AutoDrive.State fieldOrientedState = robotState.toFieldRelativeState(
      Rotation2d.fromDegrees(-45)
    );
    Assert.assertEquals(0.7071067812, fieldOrientedState.forward, 0.000001);
    Assert.assertEquals(0.0, fieldOrientedState.strafe, 0.000001);
  }

  @Test
  public void testToFieldRelativeStateNegative90() {
    // A robot facing the right side of the field being told
    // to move "forward" and "left" should have a field-oriented
    // values pulling it towards the right side of the field and
    // to the opposing alliance wall
    AutoDrive.State robotState = new AutoDrive.State(
      0.6,
      0.4
    );
    AutoDrive.State fieldOrientedState = robotState.toFieldRelativeState(
      Rotation2d.fromDegrees(-90)
    );
    Assert.assertEquals(0.4, fieldOrientedState.forward, 0.000001);
    Assert.assertEquals(-0.6, fieldOrientedState.strafe, 0.000001);
  }

}
