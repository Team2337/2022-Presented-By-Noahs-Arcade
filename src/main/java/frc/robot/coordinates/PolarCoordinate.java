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
    Translation2d translation = this.toCartesianCoordinate();
    /**
     * We need to add our CC's X (width) to our field Y (width)
     * and our CC's Y (height) to our field X (height).
     */
    return new Translation2d(
      referencePoint.getX() + translation.getY(),
      referencePoint.getY() + translation.getX()
    );
  }

  /**
   * Translates our polar coordinate to a cartesian coordinate
   *
   * @return A x, y of our polar coordinate represented in a cartesian coordinate
   *         system relative.
   */
  private Translation2d toCartesianCoordinate() {
    double x = radiusMeters * Math.cos(theta.getRadians());
    double y = radiusMeters * Math.sin(theta.getRadians());
    return new Translation2d(x, y);
  }
}
