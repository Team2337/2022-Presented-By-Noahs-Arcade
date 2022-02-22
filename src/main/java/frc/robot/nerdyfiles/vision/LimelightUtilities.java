package frc.robot.nerdyfiles.vision;

public class LimelightUtilities {
  /**
   * Algorithm from
   * https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
   * Estimates
   * range to a target using the target's elevation. This method can produce more
   * stable results
   * than SolvePNP when well tuned, if the full 6d robot pose is not required.
   * Note that this method
   * requires the camera to have 0 roll (not be skewed clockwise or CCW relative
   * to the floor), and
   * for there to exist a height differential between goal and camera. The larger
   * this differential,
   * the more accurate the distance estimate will be.
   *
   * <p>
   * Units can be converted using the {@link edu.wpi.first.wpilibj.util.Units}
   * class.
   *
   * @param cameraHeightMeters The physical height of the camera off the floor in
   *                           meters.
   * @param targetHeightMeters The physical height of the target off the floor in
   *                           meters. This
   *                           should be the height of whatever is being targeted
   *                           (i.e. if the targeting region is set to
   *                           top, this should be the height of the top of the
   *                           target).
   * @param cameraPitchRadians The pitch of the camera from the horizontal plane
   *                           in radians.
   *                           Positive values up.
   * @param targetPitchRadians The pitch of the target in the camera's lens in
   *                           radians.
   *                           Positive values up.
   * @return The estimated distance to the target in meters.
   */
  public static double calculateDistanceToTargetMeters(
    double cameraHeightMeters,
    double targetHeightMeters,
    double cameraPitchRadians,
    double targetPitchRadians) {
    return (targetHeightMeters - cameraHeightMeters) / Math.tan(cameraPitchRadians + targetPitchRadians);
  }

}
