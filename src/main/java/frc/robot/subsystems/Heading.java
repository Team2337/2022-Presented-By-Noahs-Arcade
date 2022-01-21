package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
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
   * P value should be calculated based on when we would like the robot to start
   * slowing down. Ex: If we should be rotating "full speed" (not 100% but 80%)
   * until get get to 10 degrees and then start slowing in to our final position,
   * our P value can be calculated with the following-
   * Max Speed = 0.8
   * Slow Down Error Value = 10%
   * 0.8 = 10 * kP
   * 0.8 / 10 = kP
   * kP = 0.08
   */
  private PIDController rotationController = new PIDController(0.08, 0.0, 0.0);

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
    // rotationController.setTolerance(1.0);
  }

  /**
   * Set the heading the robot should attempt to maintain. If you're
   * calling this method, be sure to require the Heading subsystem.
   * Only one Command should be updating the robot's desired Heading
   * at a time.
   */
  public void setCurrentHeading(Rotation2d currentHeading) {
    if (this.currentHeading != currentHeading) {
      resetRotationController();
    }
    this.currentHeading = currentHeading;
  }

  public boolean shouldMaintainHeading() {
    return currentHeading != null;
  }

  /**
   * Enqueue a heading to be swapped at a later point via
   * `setEnqueuedHeadingToCurrentHeading`. You do not need to
   * require the Heading subsystem when calling this method -
   * this command is safe(ish) to be called from different
   * Commands.
   */
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
   * rotational value as some percentage from
   * [-max speed, max speed] where max speed is a value
   * we would expect from a joystick (ex: [-1, 1]).
   */
  public double calculateRotation() {
    double output = rotationController.calculate(
      actualRotationSupplier.get().getDegrees(),
      currentHeading.getDegrees()
    );
    if (rotationController.atSetpoint()) {
      return 0.0;
    }
    // Clamp to some max speed (between [0, 1])
    final double maxSpeed = 0.8;
    double clamedOutput = MathUtil.clamp(
      output,
      -maxSpeed,
      maxSpeed
    );
    return clamedOutput;
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
