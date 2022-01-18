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
   * The "next" heading is the enqueued heading we would like the robot to maintain.
   * Swapping from the next heading -> the current heading is done manually via the
   * `setCurrentHeading` method.
   * Next heading may be null if the robot has no next heading to maintain.
   */
  private Rotation2d nextHeading;

  /**
   * PID used to converge the robot to the currentHeading from it's actual heading.
   */
  private PIDController rotationController = new PIDController(8.0, 4.0, 0.75);

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
      resetRotationController();
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

  /**
   * Uses the rotation PID controller to calculate the
   * offset from the current location the robot should
   * rotate in order to achieve the desired heading.
   * Is not clamped by maximum rotational speeds.
   */
  public Rotation2d calculateRotation() {
    double error = rotationController.calculate(
      actualRotationSupplier.get().getDegrees(),
      currentHeading.getDegrees()
    );
    if (rotationController.atSetpoint()) {
      return new Rotation2d();
    }
    return Rotation2d.fromDegrees(error);
  }

  /**
   * Resets the state of the internal rotation PID controller.
   * Should be called when the controller's setpoint moves, or
   * when we stop calling `calculate` on a regular basis.
   */
  public void resetRotationController() {
    rotationController.reset();
  }

}
