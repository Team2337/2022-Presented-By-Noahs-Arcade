package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Heading subsystem is a "fake" subsystem (no hardware) that is used to coordinate
 * mutual exclusivity around maintaining the heading of the robot.
 * Ex: Commands that request the robot to maintain a heading should require
 * the Heading subsystem. Only one command should be updating the robot's heading 
 * at a time.
 */
public class Heading extends SubsystemBase {

  /**
   * Supplier to provide the heading of the robot. Should come from
   * the gyro on the Drivetrain. Used to calculate our maintain heading calculation.
   */
  private Supplier<Rotation2d> currentHeadingSupplier;
  /**
   * The maintain heading is the heading we would like the robot to maintain.
   * Can be null if the robot should not maintain any specific heading.
   */
  private Rotation2d maintainHeading;
  /**
   * The next heading is the enqueued heading we would like the robot to maintain.
   * Swapping from the next heading -> the maintain heading is done manually via the
   * `setCurrentHeading` method.
   * Next heading may be null if the robot has no next heading to maintain.
   */
  private Rotation2d nextHeading;

  /**
   * PID used to converge the robot to the currentHeading from it's actual heading.
   */
  private PIDController rotationController = new PIDController(4.0, 0.0, 0.0);

  /**
   * Heading subsystem to maintain a static heading of the robot.
   * 
   * @param actualRotationSupplier A Supplier to provide the current heading of
   *                               the robot.
   *                               Should come from a gyro. Values are expected to
   *                               be in the range of [0, 360] degrees.
   */
  public Heading(Supplier<Rotation2d> currentHeadingSupplier) {
    this.currentHeadingSupplier = currentHeadingSupplier;

    rotationController.enableContinuousInput(0, 360);
    rotationController.setTolerance(1.0);
  }

  /**
   * Set the heading for the robot to maintain. If calling this function
   * from a Command, be sure to require the Heading subsystem. Only one
   * Command should be updating the robot's heading at a time.
   *
   * @param maintainHeading The heading the robot should attempt to
   *                        maintain.Expects a value from 0 to 360 degrees. Can be
   *                        null if the robot should not maintain a heading.
   */
  public void setMaintainHeading(Rotation2d maintainHeading) {
    if (this.maintainHeading != maintainHeading) {
      resetRotationController();
    }
    this.maintainHeading = maintainHeading;
  }

  public boolean shouldMaintainHeading() {
    return maintainHeading != null;
  }

  /**
   * Enqueue the next heading for the robot. There can only be one next
   * heading enqueued at a time. The next heading must be swapped manually
   * to the maintain heading by calling `setNextHeadingToMaintainHeading`.
   * 
   * @param nextHeading The next heading the robot should maintain. Expects a
   *                    value from 0 to 360 degrees. Can be null if the robot*
   *                    should not maintain a new heading.
   */
  public void setNextHeading(Rotation2d nextHeading) {
    this.nextHeading = nextHeading;
  }

  /**
   * Set the next heading of the robot to be the robot's new maintained heading.
   * If the robot's next heading is null, the new maintain heading will be null
   * and the robot will not attempt to maintain a heading.
   */
  public void setNextHeadingToMaintainHeading() {
    this.maintainHeading = this.nextHeading;
    this.nextHeading = null;
  }

  /**
   * Uses the rotation PID controller to calculate the
   * offset from the current location the robot should
   * rotate in order to achieve the desired heading.
   * Is not clamped by maximum rotational speeds.
   */
  public Rotation2d calculateRotation() {
    double output = rotationController.calculate(
      currentHeadingSupplier.get().getDegrees(),
      maintainHeading.getDegrees()
    );
    SmartDashboard.putNumber("Rotation Controller Error", rotationController.getPositionError());
    SmartDashboard.putNumber("Rotation Controller Output", output);
    if (rotationController.atSetpoint()) {
      return new Rotation2d();
    }
    return Rotation2d.fromDegrees(output);
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
