package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class Utilities {
  /**
   * Convert a (-360, 360) Rotation2d to a rotation value between (-180, 180).
   * Ex:
   * 90 degrees -> 90 degrees
   * -90 degrees -> -90 degrees
   * 181 degrees -> -179 degrees
   * -181 degrees -> 179 degrees
   * 350 degrees -> -10 degrees
   * -350 degrees -> 10 degrees
   *
   * @return A Rotation2d from (-180, 180) degrees.
   */
  public static Rotation2d relativeRotationFromAbsoluteRotation(Rotation2d rotation) {
    // Convert from from (-360, 360) to (-180, 180)
    double rotationDegrees = rotation.getDegrees();
    if (rotationDegrees > 180) {
      rotationDegrees -= 360;
    } else if (rotationDegrees < -180) {
      rotationDegrees += 360;
    }
    return Rotation2d.fromDegrees(rotationDegrees);
  }

    /**
     * Checks to see if the absolute value of the input is less than the deadband
     * @param input - Value in which the deadband will be applied (0 < input < 1)
     * @param deadband - Deadband to set on the input (double)
     * @return - input double value adjusted for the deadband
     */
    public static double deadband(double input, double deadband) {
      if (Math.abs(input) < deadband) return 0;
      return Math.copySign((Math.abs(input) - deadband) / (1 - deadband), input);
    }

    /**
     * Squares the input value
     * @param value - double value wanting to be squared
     * @return - squared input double value
     */
    public static double squareValues(double value) {
      return Math.copySign(Math.pow(value, 2), value);
    }

    public static double modifyAxis(double value) {
      // Deadband
      value = deadband(value, 0.05);
      // Square the axis
      return Math.copySign(value * value, value);
    }

    /**
     * Calculates derivative based on the current and previous sensor inputs
     * @param error - double value reading the current sensor error (current - target)
     * @param lastError - double value reading the previous sensor error (current - target)
     * @param dt - Change in time from when the previous sensor error was gotten to the time the current was gotten
     * @return - returns derivative double value to add to the speed of the motor
     */
    public static double calculateDerivative(double error, double lastError, double dt) {
      if (Double.isFinite(lastError)) {
        return (error - lastError) / dt;
      } else {
        return 0;
      }
    }

  /**
   * Determines whether or not the given value is within a certain amount of a target
   *
   * @param target The desired value
   * @param current The current value
   * @param tolerance A range that the given value can be within the target value before returning true
   * @return Whether or not the current value is within a tolerance of the target
   */
  public static boolean withinTolerance(double target, double current, double tolerance) {
    return Math.abs(target - current) <= tolerance;
  }
}