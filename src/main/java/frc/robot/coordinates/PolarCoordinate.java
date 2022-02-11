package frc.robot.coordinates;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class PolarCoordinate {

  private double radiusMeters;
  private Rotation2d theta;
  // The reference point is the "center" in our polar coordinate system
  private Translation2d referencePoint;

  /**
   * Construct a polar coordinate using the center of the field (the Hub)
   * as the reference point.
   *
   * @param radiusMeters - The radius from the reference point in meters.
   * @param theta        - The rotation from the reference point. In our
   *                     field-centric system, this is the left-hand side.
   */
  public PolarCoordinate(double radiusMeters, Rotation2d theta) {
    this(radiusMeters, theta, Constants.kHub);
  }

  /**
   * Construct a polar coordinate using a specific center point.
   *
   * @param radiusMeters   - The radius from the reference point in meters.
   * @param theta          - The rotation from the reference point. In our
   *                       field-centric system, this is the left-hand side.
   * @param referencePoint - The x, y to use as the center for our polar
   *                       coordinate. Should be a field coordinate.
   */
  public PolarCoordinate(double radiusMeters, Rotation2d theta, Translation2d referencePoint) {
    this.radiusMeters = radiusMeters;
    this.theta = theta;
    this.referencePoint = referencePoint;
  }

  public static PolarCoordinate fromFieldCoordinate(Translation2d coordinate) {
    return fromFieldCoordinate(coordinate, Constants.kHub);
  }

  public static PolarCoordinate fromFieldCoordinate(Translation2d coordinate, Translation2d referencePoint) {
    double x = coordinate.getX() - referencePoint.getX();
    double y = coordinate.getY() - referencePoint.getY();
    // Distance is our hypotenuse of our triangle
    // Angle is our tangent of our two components
    double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    double angle = Math.atan2(y, x); // (-π, π] radians
    return new PolarCoordinate(
      distance,
      new Rotation2d(angle),
      referencePoint
    );
  }

  public double getRadiusMeters() {
    return radiusMeters;
  }

  public Rotation2d getTheta() {
    return theta;
  }

  public Translation2d getReferencePoint() {
    return referencePoint;
  }

  /**
   * Translates our polar coordinate to a x, y in a cartestian
   * coordinate system relative to our reference point on the field.
   *
   * @return A x, y of our polar coordinate represented in a cartesian coordinate
   *         system relative to the reference point.
   */
  public Translation2d toFieldCoordinate() {
    double x = radiusMeters * Math.cos(theta.getRadians());
    double y = radiusMeters * Math.sin(theta.getRadians());
    return new Translation2d(
      referencePoint.getX() + x,
      referencePoint.getY() + y
    );
  }

}
