package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotType.Type;
import frc.robot.coordinates.PolarCoordinate;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static final).  Do not put anything functional in this class.
 *
 * <p>It is advised to static finalally import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public int MODULE0_DRIVE_MOTOR_ID;
  public int MODULE0_ANGLE_MOTOR_ID;
  public int MODULE0_ANGLE_CANCODER_ID;
  public double MODULE0_ANGLE_OFFSET;

  public int MODULE1_DRIVE_MOTOR_ID;
  public int MODULE1_ANGLE_MOTOR_ID;
  public int MODULE1_ANGLE_CANCODER_ID;
  public double MODULE1_ANGLE_OFFSET;

  public int MODULE2_DRIVE_MOTOR_ID;
  public int MODULE2_ANGLE_MOTOR_ID;
  public int MODULE2_ANGLE_CANCODER_ID;
  public double MODULE2_ANGLE_OFFSET;

  public int MODULE3_DRIVE_MOTOR_ID;
  public int MODULE3_ANGLE_MOTOR_ID;
  public int MODULE3_ANGLE_CANCODER_ID;
  public double MODULE3_ANGLE_OFFSET;

  public double DRIVETRAIN_TRACK_WIDTH_INCHES;
  public double DRIVETRAIN_WHEEL_BASE_INCHES;

  public double LIMELIGHT_CAMERA_HEIGHT;
  public double LIMEILGHT_CAMERA_ANGLE;
  public double HUB_HEIGHT;

  private static Constants instance;

  public static Constants getInstance() {
    if (instance == null) {
      instance = new Constants();
    }
    return instance;
  }

  public Constants() {
    switch (RobotType.getRobotType()) {
      case PRACTICE:
      MODULE0_DRIVE_MOTOR_ID = 18;
        MODULE0_ANGLE_MOTOR_ID = 19;
        MODULE0_ANGLE_CANCODER_ID = 1;
        MODULE0_ANGLE_OFFSET = -Math.toRadians(130.599976);

        MODULE1_DRIVE_MOTOR_ID = 1;
        MODULE1_ANGLE_MOTOR_ID = 2;
        MODULE1_ANGLE_CANCODER_ID = 2;
        MODULE1_ANGLE_OFFSET = -Math.toRadians(175.163269);

        MODULE2_DRIVE_MOTOR_ID = 8;
        MODULE2_ANGLE_MOTOR_ID = 9;
        MODULE2_ANGLE_CANCODER_ID = 3;
        MODULE2_ANGLE_OFFSET = -Math.toRadians(278.338623);

        MODULE3_DRIVE_MOTOR_ID = 10;
        MODULE3_ANGLE_MOTOR_ID = 11;
        MODULE3_ANGLE_CANCODER_ID = 4;
        MODULE3_ANGLE_OFFSET = -Math.toRadians(355.860901);

        DRIVETRAIN_TRACK_WIDTH_INCHES = 18.75;
        DRIVETRAIN_WHEEL_BASE_INCHES = 18.75;

        LIMELIGHT_CAMERA_HEIGHT = Units.inchesToMeters(36.5);
        LIMEILGHT_CAMERA_ANGLE = Units.degreesToRadians(38.1);
        HUB_HEIGHT = Units.inchesToMeters(103.8);
        break;
      case COMPETITION:
        MODULE0_DRIVE_MOTOR_ID = 18;
        MODULE0_ANGLE_MOTOR_ID = 19;
        MODULE0_ANGLE_CANCODER_ID = 1;
        MODULE0_ANGLE_OFFSET = -Math.toRadians(130.599976);

        MODULE1_DRIVE_MOTOR_ID = 1;
        MODULE1_ANGLE_MOTOR_ID = 2;
        MODULE1_ANGLE_CANCODER_ID = 2;
        MODULE1_ANGLE_OFFSET = -Math.toRadians(175.163269);

        MODULE2_DRIVE_MOTOR_ID = 8;
        MODULE2_ANGLE_MOTOR_ID = 9;
        MODULE2_ANGLE_CANCODER_ID = 3;
        MODULE2_ANGLE_OFFSET = -Math.toRadians(278.338623);

        MODULE3_DRIVE_MOTOR_ID = 10;
        MODULE3_ANGLE_MOTOR_ID = 11;
        MODULE3_ANGLE_CANCODER_ID = 4;
        MODULE3_ANGLE_OFFSET = -Math.toRadians(355.860901);

        DRIVETRAIN_TRACK_WIDTH_INCHES = 18.75;
        DRIVETRAIN_WHEEL_BASE_INCHES = 18.75;

        LIMELIGHT_CAMERA_HEIGHT = Units.inchesToMeters(28.5);
        LIMEILGHT_CAMERA_ANGLE = Units.degreesToRadians(38.1);
        HUB_HEIGHT = Units.inchesToMeters(103.8);
        break;
      case SKILLSBOT:
        MODULE0_DRIVE_MOTOR_ID = 0; 
        MODULE0_ANGLE_MOTOR_ID = 4;
        MODULE0_ANGLE_CANCODER_ID = 1;
        MODULE0_ANGLE_OFFSET = -Math.toRadians(50.701904296875);

        MODULE1_DRIVE_MOTOR_ID = 1;
        MODULE1_ANGLE_MOTOR_ID = 5;
        MODULE1_ANGLE_CANCODER_ID = 2;
        MODULE1_ANGLE_OFFSET = -Math.toRadians(128.58123779296875);

        MODULE2_DRIVE_MOTOR_ID = 14;
        MODULE2_ANGLE_MOTOR_ID = 10;
        MODULE2_ANGLE_CANCODER_ID = 3;
        MODULE2_ANGLE_OFFSET = -Math.toRadians(346.63238525390625);

        MODULE3_DRIVE_MOTOR_ID = 15;
        MODULE3_ANGLE_MOTOR_ID = 11;
        MODULE3_ANGLE_CANCODER_ID = 4;
        MODULE3_ANGLE_OFFSET = -Math.toRadians(286.42730712890625);

        DRIVETRAIN_TRACK_WIDTH_INCHES = 17;
        DRIVETRAIN_WHEEL_BASE_INCHES = 17;

        LIMELIGHT_CAMERA_HEIGHT = Units.inchesToMeters(40.5);
        LIMEILGHT_CAMERA_ANGLE = Units.degreesToRadians(34);
        HUB_HEIGHT = Units.inchesToMeters(103.8);
        break;
    }
  }
 
  /**
   * Sets the Track width and wheel base of the robot based on the centerpoint of the swerve modules.
   * Track width is side to side
   * Wheel base is fromt to back.
   */
  // /2 since we're measuring from the center - halfway
  private static final double MODULE_DISTANCE_WIDTH_FROM_CENTER_INCHES = Constants.getInstance().DRIVETRAIN_TRACK_WIDTH_INCHES / 2;
  private static final double MODULE_DISTANCE_LENGTH_FROM_CENTER_INCHES = Constants.getInstance().DRIVETRAIN_WHEEL_BASE_INCHES / 2;

  // Radius to the wheel modules can be thought of as a triangle - width and length are the two sides
  public static final double DRIVETRAIN_RADIUS_INCHES = Math.hypot(MODULE_DISTANCE_WIDTH_FROM_CENTER_INCHES, MODULE_DISTANCE_LENGTH_FROM_CENTER_INCHES);

  // Location of the Hub on the field - the center of the field
  public static final Translation2d kHub = new Translation2d(
    Units.feetToMeters(27),
    Units.feetToMeters(13.5)
  );

  public static final class Auto {
    /**
     * Our polar coordinates for our balls are based off of the center of the field.
     * Note that the angles are measured where the left-hand field perimeter
     * (y = 27) is our 0 degrees. This is because when our center point is in the
     * middle of the field, the top-left quadrant in a cartesian system is our
     * positive-positive quadrant. Top right should be positive X values and
     * negative Y values (shifted to the right of our center).
     */
    // Ball 1 = Ball nearest to the top starting location
    public static final PolarCoordinate kBall1 = new PolarCoordinate(
      Units.inchesToMeters(153),
      Rotation2d.fromDegrees(9.75) // 80.25 using alliance wall zero
    );
    // Ball 2 = Ball nearest to the middle starting location
    public static final PolarCoordinate kBall2 = new PolarCoordinate(
      Units.inchesToMeters(153),
      Rotation2d.fromDegrees(54.75) // 35.25 using alliance wall zero
    );
    // Ball 3 = Ball just in front of the Terminal
    public static final PolarCoordinate kBall3 = new PolarCoordinate(
      Units.inchesToMeters(305.66),
      Rotation2d.fromDegrees(67.35) // 22.65 using alliance wall zero
    );
    // Ball 4 is the human player ball at the Terminal
    // Ball 5 = Ball nearest to the bottom starting location
    public static final PolarCoordinate kBall5 = new PolarCoordinate(
      Units.inchesToMeters(153),
      Rotation2d.fromDegrees(122.25) // -32.25 using alliance wall zero
    );
  }

  // Robot-specific configuration for our swerve drive algorithm
  public static final class Swerve {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;

    //  The formula for calculating the theoretical maximum velocity is:
    //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    //  By default this value is setup for a Mk4 L1 module using Falcon500s to drive.
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
      SdsModuleConfigurations.MK4_L1.getDriveReduction() *
      SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;

      /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
      Math.hypot(Units.inchesToMeters(Constants.getInstance().DRIVETRAIN_TRACK_WIDTH_INCHES) / 2.0, Units.inchesToMeters(Constants.getInstance().DRIVETRAIN_WHEEL_BASE_INCHES) / 2.0);
  }

  public static final int INTAKE_MOTOR_ID = 6;
  public static final double INTAKE_SPEED = 0.5;

  public static final int DELIVERY_MOTOR_ID = 7;
  public static final double DELIVERY_SPEED = 0.5;
  //CLIMBER CONSTANTS
  public static final class Climber {
    public static final int LEFT_CLIMBER_MOTOR_ID = 16;
    public static final int RIGHT_CLIMBER_MOTOR_ID = 3;
    public static final int CLIMBER_STRING_POT_ID = 3;
    public static final double LOW_RUNG = 1.0;
    public static final double MID_RUNG = 3.0;
    public static final double RICKABOOT = 0.5;
    public static final double START = 0;
  }

  // Color stuff
  public static enum BallColor {
    RED,
    BLUE,
    UNKNOWN
  }
}
