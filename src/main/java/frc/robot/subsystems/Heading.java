package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities;

/**
 * The Heading subsystem is a "fake" subsystem (no hardware) that is used to coordinate
 * mutual exclusivity around maintaining the heading of the robot.
 * Ex: Commands that request the robot to maintain a heading should require
 * the Heading subsystem. Only one command should be updating the robot's heading 
 * at a time.
 */
public class Heading extends SubsystemBase {

  /**
   * Supplier to provide the gyro angle of the robot. Should come from
   * the gyro on the Drivetrain. Used to calculate our maintain heading calculation.
   */
  private Supplier<Rotation2d> gyroAngleSupplier;
  /**
   * The maintain heading is the heading we would like the robot to maintain.
   * Can be null if the robot should not maintain any specific heading.
   */
  private Rotation2d maintainHeading;
  /**
   * The next heading is the enqueued heading we would like the robot to maintain.
   * Swapping from the next heading -> the maintain heading is done via the
   * `setNextHeadingToMaintainHeading` method.
   * Next heading may be null if the robot has no next heading to maintain.
   */
  private Rotation2d nextHeading;

  /**
   * PID used to converge the robot to the maintainHeading from it's current heading.
   */
  private PIDController rotationController = new PIDController(0.04, 0.0, 0.0);

  /**
   * Heading subsystem to maintain a static heading of the robot.
   *
   * @param gyroAngleSupplier A Supplier to provide the gyro value of the robot.
   *                          Should come from the Pigeon.
   */
  public Heading(Supplier<Rotation2d> gyroAngleSupplier) {
    this.gyroAngleSupplier = gyroAngleSupplier;

    rotationController.enableContinuousInput(-180, 180);
    // +/- 1 degree position error tolerance
    rotationController.setTolerance(1.0);
  }

  /**
   * Convert from the gyro angle supplier to a heading rotation value.
   * Gyro angles are reported as CCW rotations being a positive change,
   * and CW rotations being a negative value. We want to convert to a
   * more heading value by flipping those values.
   *
   * @return A Rotation2d representing the heading of the robot.
   */
  private Rotation2d currentHeading() {
    return gyroAngleSupplier.get().unaryMinus();
  }

  /**
   * Set the heading for the robot to maintain. If calling this function
   * from a Command, be sure to require the Heading subsystem. Only one
   * Command should be updating the robot's heading at a time.
   *
   * @param maintainHeading The heading the robot should attempt to
   *                        maintain.Expects a value (-180, 180) degrees. Can be
   *                        null if the robot should not maintain a heading.
   */
  public void setMaintainHeading(Rotation2d maintainHeading) {
    maintainHeading = Utilities.relativeRotationFromAbsoluteRotation(maintainHeading);
    if (this.maintainHeading == maintainHeading) {
      return;
    }
    this.maintainHeading = maintainHeading;
    resetRotationController();
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
   *                    value from (-180, 180) degrees. Can be null if the robot
   *                    should not maintain a new heading.
   */
  public void setNextHeading(Rotation2d nextHeading) {
    this.nextHeading = Utilities.relativeRotationFromAbsoluteRotation(nextHeading);
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
   * rotational value as some percentage from
   * [-max speed, max speed] where max speed is a value
   * we would expect from a joystick (ex: [-1, 1]).
   */
  public double calculateRotation() {
    Rotation2d currentHeading = Utilities.relativeRotationFromAbsoluteRotation(currentHeading());
    double output = rotationController.calculate(
      currentHeading.getDegrees(),
      maintainHeading.getDegrees()
    );
    SmartDashboard.putNumber("Rotation Controller Error", rotationController.getPositionError());
    SmartDashboard.putNumber("Rotation Controller Output", output);
    if (rotationController.atSetpoint()) {
      return 0.0;
    }
    // Clamp to some max speed (should be between [0.0, 1.0])
    final double maxSpeed = 0.3;
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
