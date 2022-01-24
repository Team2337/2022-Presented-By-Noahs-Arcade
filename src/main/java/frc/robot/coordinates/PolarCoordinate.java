package frc.robot.coordinates;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PolarCoordinate {

  private Rotation2d theta;
  private double distanceMeters;

  public PolarCoordinate(Rotation2d theta, double distanceMeters) {
    this.theta = theta;
    this.distanceMeters = distanceMeters;
  }

  public Rotation2d getTheta() {
    return theta;
  }
  
  public double getDistanceMeters() {
    return distanceMeters;
  }

  // Pretend for now "center" is (13.5, 27)
  public Translation2d toTranslationMeters(Translation2d center) {
    double x = (distanceMeters * Math.cos(theta.getRadians())) + center.getX();
    double y = (distanceMeters * Math.sin(theta.getRadians())) + center.getY();
    return new Translation2d(x, y);
  }
}
