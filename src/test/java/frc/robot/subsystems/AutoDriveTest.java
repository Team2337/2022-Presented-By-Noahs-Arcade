package frc.robot.subsystems;

import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)
public class AutoDriveTest {

  @Test
  public void testAutoDriveScaling() {
    AutoDrive.State state = new AutoDrive.State(
      0.5,
      0.5,
      false
    );
    // No scaling should occur - values add to one
    Assert.assertEquals(0.5, state.forward, 0.000001);
    Assert.assertEquals(0.5, state.strafe, 0.000001);
  }

  @Test
  public void testAutoDriveScalingPartial() {
    AutoDrive.State state = new AutoDrive.State(
      1.0,
      0.5,
      false
    );
    // Scaling should occur - forward should be prioritized
    Assert.assertEquals(0.666666, state.forward, 0.000001);
    Assert.assertEquals(0.333333, state.strafe, 0.000001);
  }

  @Test
  public void testAutoDriveScalingLarge() {
    AutoDrive.State state = new AutoDrive.State(
      3.0,
      3.0,
      false
    );
    // Giving each value a 3 should scale each value to 0.5 (adds to 1)
    Assert.assertEquals(0.5, state.forward, 0.000001);
    Assert.assertEquals(0.5, state.strafe, 0.000001);
  }

  @Test
  public void testAutoDriveScalingNegative() {
    AutoDrive.State state = new AutoDrive.State(
      -0.5,
      -0.5,
      false
    );
    // No scaling should occur - values add to -1
    Assert.assertEquals(-0.5, state.forward, 0.000001);
    Assert.assertEquals(-0.5, state.strafe, 0.000001);
  }

  @Test
  public void testAutoDriveScalingNegativePartial() {
    AutoDrive.State state = new AutoDrive.State(
      -0.5,
      -1.0,
      false
    );
    // Scaling should occur - strafe should be prioritized
    Assert.assertEquals(-0.3333333, state.forward, 0.000001);
    Assert.assertEquals(-0.6666666, state.strafe, 0.000001);
  }

  @Test
  public void testAutoDriveScalingNegativeLarge() {
    AutoDrive.State state = new AutoDrive.State(
      -3.0,
      -3.0,
      false
    );
    // Giving each value a -3 should scale each value to -0.5 (adds to -1)
    Assert.assertEquals(-0.5, state.forward, 0.000001);
    Assert.assertEquals(-0.5, state.strafe, 0.000001);
  }

  @Test
  public void testAutoDriveScalingNegativePositive() {
    AutoDrive.State state = new AutoDrive.State(
      1.0,
      -1.0,
      false
    );
    // Scaling should occur - values should keep their signs
    Assert.assertEquals(0.5, state.forward, 0.000001);
    Assert.assertEquals(-0.5, state.strafe, 0.000001);
  }

}
