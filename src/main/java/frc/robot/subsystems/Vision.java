package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities;
import frc.robot.coordinates.PolarCoordinate;
import frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.sensors.PigeonIMU;


public class Vision extends SubsystemBase {

  private boolean rotateLimelight = false;
  private boolean visionDebug = false;
  private PigeonIMU pigeon;
  private Drivetrain drivetrain;

  private ShuffleboardTab tab = Shuffleboard.getTab("Vision");

  private MedianFilter medianFilterX = new MedianFilter(10);
  private MedianFilter medianFilterY = new MedianFilter(10);

  private double calculatedTx;
  private double calculatedTy;

  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(28.5);
  final double TARGET_HEIGHT_METERS = Units.inchesToMeters(103.8);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(38.1);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);
  final double VISION_TARGET_OFFSET_FROM_HUB_CENTER = 24;
  final double distanceToCenterOfHub = 0;


  public Vision(PigeonIMU pigeon, Drivetrain drivetrain) {
    this.pigeon = pigeon;
    this.drivetrain = drivetrain;

    ShuffleboardLayout dataWidget = tab.getLayout("Data", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0);

    calculatedTx = getDoubleValue("tx");
    dataWidget.addNumber("tx (Median)", () -> calculatedTx);
  }

  /**
   * Sets the LED mode to on, off, or blink
   * @param mode - the mode of the LEDs
   * Example:
   * 0: Sets the mode to what is in the current pipeline
   * 1: Turns off the LEDs
   * 2: Blink mode on LEDs
   * 3: Turns on the LEDs
   */
  public void setLEDMode(int mode) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(mode);
  }

  /**
   * Gets the Limelight mode number from the NetworkTable
   * @return - returns the mode number from the NetworkTable
   */
  public int getLEDMode() {
    return (int)NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getValue().getDouble();
  }

  /**
   * Sets the pipeline of the limelight
   * @param pipeline - sets the desired pipeline number between 0-9
   * 0 - CloseVision (0-15 ft away from the tower)
   * 1 - MediumVision (between 15-21 ft)
   * 2 - FarVision (21-26 ft)
   * 9 - Drivecam
   */
  public void switchPipeLine(int pipeline) {
    double currentPipeline = NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").getDouble(0);
    if (currentPipeline != pipeline) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    }
  }

  /**
   * Gets the current pipeline on the limelight
   * @return - Double value limelight pipeline (0 -> 9)
   */
  public double getPipeline() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").getDouble(0);
  }

  public double getTx() {
    return calculatedTx;
  }

  // TODO: Rename
  public boolean activeTarget() {
    return (int)getDoubleValue("tv") == 1.0 ? true : false;
  }

  public double getTy() {
    return getDoubleValue("ty");
  }

  /**
   * This will get the value from tx, ta, etc. by using a string
   * @param output - string double value
   * @return - returns the double value from the string for example from ta, tx, etc.
   */
  private double getDoubleValue(String output) {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry(output).getDouble(0);
  }

  /**
   * This will get the X coordinte from Opensight
   * @return - returns the x-coordinate value, which will show how far we need to rotate
   */
  public double getOpenSightXCoordinateValue() {
    return NetworkTableInstance.getDefault().getTable("PutCoordinate").getEntry("coord-x").getDouble(0);
  }

  /**
   * This will get the Y coordinate from Opensight
   * @return - returns the y-coordinate value, which will show how far the robot needs to drive
   */
  public double getOpenSightYCoordinateValue() {
    return NetworkTableInstance.getDefault().getTable("PutCoordinate").getEntry("coord-y").getDouble(0);
  }

  /**
   * Receives the NetworkTable values from Opensight
   * @return - returns the opensight networktable which will return a true/false value
   */
  public boolean getOpenSightNTValue() {
    return NetworkTableInstance.getDefault().getTable("PutNT").getEntry("succ").getBoolean(false);
  }

  /**
   * Lets us know if we are in the limelight mode, we are rotating using the limelight
   * @param rotateLimelight - Boolean value (limelight mode: true | not limelight mode: false)
   */
  public void setRotateLimelight(boolean rotateLimelight) {
    this.rotateLimelight = rotateLimelight;
  }

  /**
   * Lets us know if we are in the limelight mode, we are rotating using the limelight
   * @return - Boolean value limelight mode
   */
  public boolean getRotateLimelight() {
    return rotateLimelight;
  }

  public double getDistanceFromCenterHub() {
    return Units.metersToInches(calculateDistanceToTargetMeters(
      CAMERA_HEIGHT_METERS,
      TARGET_HEIGHT_METERS,
      CAMERA_PITCH_RADIANS,
      Units.degreesToRadians(getTy())
   )) + VISION_TARGET_OFFSET_FROM_HUB_CENTER;
  }

  public PolarCoordinate getPolarResetCoordinates() {
    //Negate gryoscope rotation if using skills bot
    return new PolarCoordinate(Units.inchesToMeters(getDistanceFromCenterHub()), Rotation2d.fromDegrees(-drivetrain.getGyroscopeCalculateOffset()));
    //return new PolarCoordinate(118, Rotation2d.fromDegrees(47));
  }



  @Override
  public void periodic() {
    double range =
    calculateDistanceToTargetMeters(
       CAMERA_HEIGHT_METERS,
       TARGET_HEIGHT_METERS,
       CAMERA_PITCH_RADIANS,
       Units.degreesToRadians(getTy())
    );

    calculatedTx = medianFilterX.calculate(getDoubleValue("tx"));
    calculatedTy = medianFilterY.calculate(getDoubleValue("ty"));
    SmartDashboard.putNumber("tx", getTx());
    SmartDashboard.putNumber("Calculated tx", calculatedTx);
    SmartDashboard.putNumber("ty", getTy());
    SmartDashboard.putNumber("Calculated ty", calculatedTy);
    SmartDashboard.putNumber("Calculate Distance to Target (feet)", Units.metersToInches(range));
    SmartDashboard.putNumber("Distance from center hub", getDistanceFromCenterHub());
    SmartDashboard.putNumber("Gyro + Tx", pigeon.getYaw() - calculatedTx);
  }
  
    /**
    * Algorithm from https://docs.limelightvision.io/en/latest/cs_estimating_distance.html Estimates
    * range to a target using the target's elevation. This method can produce more stable results
    * than SolvePNP when well tuned, if the full 6d robot pose is not required. Note that this method
    * requires the camera to have 0 roll (not be skewed clockwise or CCW relative to the floor), and
    * for there to exist a height differential between goal and camera. The larger this differential,
    * the more accurate the distance estimate will be.
    *
    * <p>Units can be converted using the {@link edu.wpi.first.wpilibj.util.Units} class.
    *
    * @param cameraHeightMeters The physical height of the camera off the floor in meters.
    * @param targetHeightMeters The physical height of the target off the floor in meters. This
    *     should be the height of whatever is being targeted (i.e. if the targeting region is set to
    *     top, this should be the height of the top of the target).
    * @param cameraPitchRadians The pitch of the camera from the horizontal plane in radians.
    *     Positive values up.
    * @param targetPitchRadians The pitch of the target in the camera's lens in radians. 
    *     Positive values up.
    * @return The estimated distance to the target in meters.
    */

    public static double calculateDistanceToTargetMeters(
            double cameraHeightMeters,
            double targetHeightMeters,
            double cameraPitchRadians,
            double targetPitchRadians) {
        return (Constants.getInstance().HUB_HEIGHT - Constants.getInstance().LIMELIGHT_CAMERA_HEIGHT)
                / Math.tan(Constants.getInstance().LIMEILGHT_CAMERA_ANGLE + targetPitchRadians);
    }

    /**
    * Estimate the {@link Translation2d} of the target relative to the camera.
    *
    * @param targetDistanceMeters The distance to the target in meters.
    * @param yaw The observed yaw of the target.
    * @return The target's camera-relative translation.
    */
    public static Translation2d estimateCameraToTargetTranslation(
            double targetDistanceMeters, Rotation2d yaw) {
        return new Translation2d(
                yaw.getCos() * targetDistanceMeters, yaw.getSin() * targetDistanceMeters);
    }

    /**
    * Estimate the position of the robot in the field.
    *
    * @param cameraHeightMeters The physical height of the camera off the floor in meters.
    * @param targetHeightMeters The physical height of the target off the floor in meters. This
    *     should be the height of whatever is being targeted (i.e. if the targeting region is set to
    *     top, this should be the height of the top of the target).
    * @param cameraPitchRadians The pitch of the camera from the horizontal plane in radians.
    *     Positive values up.
    * @param targetPitchRadians The pitch of the target in the camera's lens in radians. Positive
    *     values up.
    * @param targetYaw The observed yaw of the target. Note that this *must* be CCW-positive, and
    *     Photon returns CW-positive.
    * @param gyroAngle The current robot gyro angle, likely from odometry.
    * @param fieldToTarget A Pose2d representing the target position in the field coordinate system.
    * @param cameraToRobot The position of the robot relative to the camera. If the camera was
    *     mounted 3 inches behind the "origin" (usually physical center) of the robot, this would be
    *     Transform2d(3 inches, 0 inches, 0 degrees).
    * @return The position of the robot in the field.
    */
    public static Pose2d estimateFieldToRobot(
            double cameraHeightMeters,
            double targetHeightMeters,
            double cameraPitchRadians,
            double targetPitchRadians,
            Rotation2d targetYaw,
            Rotation2d gyroAngle,
            Pose2d fieldToTarget,
            Transform2d cameraToRobot) {
        return estimateFieldToRobot(
                    estimateCameraToTarget(
                        estimateCameraToTargetTranslation(
                                calculateDistanceToTargetMeters(
                                        cameraHeightMeters, targetHeightMeters, cameraPitchRadians, targetPitchRadians),
                                targetYaw),
                        fieldToTarget,
                        gyroAngle),
                    fieldToTarget,
                    cameraToRobot);
    }

    /**
    * Estimates a {@link Transform2d} that maps the camera position to the target position, using the
    * robot's gyro. Note that the gyro angle provided *must* line up with the field coordinate system
    * -- that is, it should read zero degrees when pointed towards the opposing alliance station, and
    * increase as the robot rotates CCW.
    *
    * @param cameraToTargetTranslation A Translation2d that encodes the x/y position of the target
    *     relative to the camera.
    * @param fieldToTarget A Pose2d representing the target position in the field coordinate system.
    * @param gyroAngle The current robot gyro angle, likely from odometry.
    * @return A Transform2d that takes us from the camera to the target.
    */
    public static Transform2d estimateCameraToTarget(
            Translation2d cameraToTargetTranslation, Pose2d fieldToTarget, Rotation2d gyroAngle) {
        // This pose maps our camera at the origin out to our target, in the robot
        // reference frame
        // The translation part of this Transform2d is from the above step, and the
        // rotation uses our robot's
        // gyro.
        return new Transform2d(
                cameraToTargetTranslation, gyroAngle.times(-1).minus(fieldToTarget.getRotation()));
    }

    /**
    * Estimates the pose of the robot in the field coordinate system, given the position of the
    * target relative to the camera, the target relative to the field, and the robot relative to the
    * camera.
    *
    * @param cameraToTarget The position of the target relative to the camera.
    * @param fieldToTarget The position of the target in the field.
    * @param cameraToRobot The position of the robot relative to the camera. If the camera was
    *     mounted 3 inches behind the "origin" (usually physical center) of the robot, this would be
    *     Transform2d(3 inches, 0 inches, 0 degrees).
    * @return The position of the robot in the field.
    */
    public static Pose2d estimateFieldToRobot(
            Transform2d cameraToTarget, Pose2d fieldToTarget, Transform2d cameraToRobot) {
        return estimateFieldToCamera(cameraToTarget, fieldToTarget).transformBy(cameraToRobot);
    }

    /**
    * Estimates the pose of the camera in the field coordinate system, given the position of the
    * target relative to the camera, and the target relative to the field. This *only* tracks the
    * position of the camera, not the position of the robot itself.
    *
    * @param cameraToTarget The position of the target relative to the camera.
    * @param fieldToTarget The position of the target in the field.
    * @return The position of the camera in the field.
    */
    public static Pose2d estimateFieldToCamera(Transform2d cameraToTarget, Pose2d fieldToTarget) {
        var targetToCamera = cameraToTarget.inverse();
        return fieldToTarget.transformBy(targetToCamera);
    }
}