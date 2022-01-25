package frc.robot.coordinates;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class PolarCoordinate {

  private double radiusMeters;
  private Rotation2d theta;

  public PolarCoordinate(double radiusMeters, Rotation2d theta) {
    this.radiusMeters = radiusMeters;
    this.theta = theta;
  }

  public double getRadiusMeters() {
    return radiusMeters;
  }

  public Rotation2d getTheta() {
    return theta;
  }

  /**
   * Translates our polar coordinate to a x, y in a cartestian
   * coordinate system relative to the center of the field (the Hub).
   * 
   * @return A x, y of our polar coordinate represented in a cartesian coordinate
   *         system relative to the center of the field.
   */
  public Translation2d toFieldCoordinate() {
    // Hub is in the center of the field
    // Center of field is (27ft, 13.5ft)
    Translation2d center = new Translation2d(
      Units.feetToMeters(27),
      Units.feetToMeters(13.5)
    );
    Translation2d translation = this.toCartesianCoordinate();
    /**
     * We need to add our CC's X (width) to our field Y (width)
     * and our CC's Y (height) to our field X (height).
     */
    return new Translation2d(
      center.getX() + translation.getY(),
      center.getY() + translation.getX()
    );
  }

  /**
   * Translates our polar coordinate to a cartesian coordinate
   * 
   * @return A x, y of our polar coordinate represented in a cartesian coordinate
   *         system relative.
   */
  public Translation2d toCartesianCoordinate() {
    double x = radiusMeters * Math.cos(theta.getRadians());
    double y = radiusMeters * Math.sin(theta.getRadians());
    return new Translation2d(x, y);
  }
}
