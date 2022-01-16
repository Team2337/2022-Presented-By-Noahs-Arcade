package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Heading subsystem is a "fake" subsystem (no hardware) that is used to coordinate
 * mutual exclusivity around maintaining the heading of the robot.
 * Ex: Commands that request the robot to be set to a specific heading
 * should require the Heading subsystem. Only one command should be
 * updating the robot's heading at a time.
 */
public class Heading extends SubsystemBase {

  /**
   * Supplier to provide the ACTUAL rotation of the robot. Should come from
   * the gyro on the Drivetrain. Used to calculate our maintain heading calculation.
   */
  private Supplier<Rotation2d> actualRotationSupplier;
  /**
   * The "current" heading is the heading we would like the robot to maintain.
   * Not that actual heading of the robot (see Drivetrain for that information).
   * Can be null if the robot should not maintain any specific heading.
   */
  private Rotation2d currentHeading;
  /**
   * Set when we first provide a new heading to maintain.
   */
  private boolean newHeadingSet = false;
  /**
   * The "next" heading is the enqueued heading we would like the robot to maintain.
   * Swapping from the next heading -> the current heading is done manually via the
   * `setCurrentHeading` method.
   * Next heading may be null if the robot has no next heading to maintain.
   */
  private Rotation2d nextHeading;

  /**
   * PID used to converge the robot to the currentHeading from it's actual heading.
   */
  private PIDController rotationController = new PIDController(1.0, 0.0, 0.1);
  // TODO: Use a ProfiledPIDController so we can use a motion profile to smooth our turning

  /**
   * Heading subsystem to maintain a static heading of the robot.
   * 
   * @param actualRotationSupplier A Supplier to provide the actual rotation of
   *                               the robot.
   *                               Should come from a gyro. Values are expected to
   *                               be in the range of [0, 360] degrees.
   */
  public Heading(Supplier<Rotation2d> actualRotationSupplier) {
    this.actualRotationSupplier = actualRotationSupplier;

    rotationController.enableContinuousInput(0, 360);
  }

  public void setCurrentHeading(Rotation2d currentHeading) {
    if (this.currentHeading != currentHeading) {
      newHeadingSet = true;
    }
    this.currentHeading = currentHeading;
  }

  public boolean shouldMaintainHeading() {
    return currentHeading != null;
  }

  public void enqueueHeading(Rotation2d nextHeading) {
    this.nextHeading = nextHeading;
  }
  
  public Rotation2d dequeueHeading() {
    Rotation2d heading = this.nextHeading;
    this.nextHeading = null;
    return heading;
  }

  // TODO: Write docs here...
  // TODO: This needs a better name - it's calculating the error, but that's not descriptive
  public Rotation2d calculateRotation() {
    // If we've gotten a new rotation since our last calculation, we should reset
    // the internal state of our controller so we don't end up with huge swings
    if (newHeadingSet) {
      rotationController.reset();
      newHeadingSet = false;
    }

    // If we've reached our setpoint (within a tolerance) don't move the robot
    if (rotationController.atSetpoint()) {
      return new Rotation2d();
    }

    // TODO: Note to Zach - we should see what kind of values the Pigeon will give us.
    // Someplace in 2337's 2020 code they have a comment that the Pigeon is giving us
    // a value from -360 to 360. I need to confirm if that's true.
    double error = rotationController.calculate(
      actualRotationSupplier.get().getDegrees(),
      currentHeading.getDegrees()
    );
    return Rotation2d.fromDegrees(error);
  }

}
