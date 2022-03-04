package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public static double STARTING_ANGLE = 25;

  public int MODULE0_DRIVE_MOTOR_ID;
  public int MODULE0_ANGLE_MOTOR_ID;
  public int MODULE0_ANGLE_CANCODER_ID;
  public double MODULE0_ANGLE_OFFSET;

  public final int MODULE1_DRIVE_MOTOR_ID;
  public final int MODULE1_ANGLE_MOTOR_ID;
  public final int MODULE1_ANGLE_CANCODER_ID;
  public final double MODULE1_ANGLE_OFFSET;

  public final int MODULE2_DRIVE_MOTOR_ID;
  public final int MODULE2_ANGLE_MOTOR_ID;
  public final int MODULE2_ANGLE_CANCODER_ID;
  public final double MODULE2_ANGLE_OFFSET;

  public final int MODULE3_DRIVE_MOTOR_ID;
  public final int MODULE3_ANGLE_MOTOR_ID;
  public final int MODULE3_ANGLE_CANCODER_ID;
  public final double MODULE3_ANGLE_OFFSET;

  public final double DRIVETRAIN_TRACK_WIDTH_INCHES;
  public final double DRIVETRAIN_WHEEL_BASE_INCHES;

  public final double LIMELIGHT_CAMERA_HEIGHT;
  public final double LIMEILGHT_CAMERA_ANGLE;

  private static Constants instance;

  public static final class DashboardLogging {
    public static final boolean CLIMBER = true;
    public static final boolean DELIVERY = false;
    public static final boolean DRIVETRAIN = false;
    public static final boolean HEADING = false;
    public static final boolean INTAKE = false;
    public static final boolean KICKER = false;
    public static final boolean PDH = false;
    public static final boolean PIXY = false;
    public static final boolean SHOOTER = false;
  }

  public static Constants getInstance() {
    if (instance == null) {
      instance = new Constants();
    }
    return instance;
  }

  public Constants() {
    RobotType.Type robotType = RobotType.getRobotType();
    SmartDashboard.putString("Constants Robot Type", robotType.description);
    switch (robotType) {
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

        DRIVETRAIN_TRACK_WIDTH_INCHES = 10.5;
        DRIVETRAIN_WHEEL_BASE_INCHES = 10.5;

        LIMELIGHT_CAMERA_HEIGHT = Units.inchesToMeters(40.5);
        LIMEILGHT_CAMERA_ANGLE = Units.degreesToRadians(34);
        break;
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

        LIMELIGHT_CAMERA_HEIGHT = Units.inchesToMeters(38);
        LIMEILGHT_CAMERA_ANGLE = Units.degreesToRadians(30.91193711);
        break;
      case COMPETITION:
      default:
        MODULE0_DRIVE_MOTOR_ID = 18;
        MODULE0_ANGLE_MOTOR_ID = 19;
        MODULE0_ANGLE_CANCODER_ID = 1;
        MODULE0_ANGLE_OFFSET = -Math.toRadians(76.37109375);

        MODULE1_DRIVE_MOTOR_ID = 1;
        MODULE1_ANGLE_MOTOR_ID = 2;
        MODULE1_ANGLE_CANCODER_ID = 2;
        MODULE1_ANGLE_OFFSET = -Math.toRadians(204.430078125);

        MODULE2_DRIVE_MOTOR_ID = 8;
        MODULE2_ANGLE_MOTOR_ID = 9;
        MODULE2_ANGLE_CANCODER_ID = 3;
        MODULE2_ANGLE_OFFSET = -Math.toRadians(195.37382812500002);

        MODULE3_DRIVE_MOTOR_ID = 10;
        MODULE3_ANGLE_MOTOR_ID = 11;
        MODULE3_ANGLE_CANCODER_ID = 4;
        MODULE3_ANGLE_OFFSET = -Math.toRadians(255.3140625);

        DRIVETRAIN_TRACK_WIDTH_INCHES = 18.75;
        DRIVETRAIN_WHEEL_BASE_INCHES = 18.75;

        LIMELIGHT_CAMERA_HEIGHT = Units.inchesToMeters(28.5);
        LIMEILGHT_CAMERA_ANGLE = Units.degreesToRadians(38.1);
        break;
    }
  }

  /**
   * Sets the Track width and wheel base of the robot based on the centerpoint of the swerve modules.
   * Track width is side to side
   * Wheel base is front to back.
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
  public static final double HUB_HEIGHT_METERS = Units.inchesToMeters(103.8);
  public static final double VISION_TARGET_OFFSET_FROM_HUB_CENTER_METERS = Units.feetToMeters(2);

  public static final class Auto {
    /**
     * Our polar coordinates for our balls are based off of the center of the field.
     * Note that the angles are measured where the opposing alliance wall
     * is our 0 degrees line (positive X axis). 180 is added to our thetas in order
     * to get them to be on our side of the field, as opposed to the opposing side.
     */
    public static final double kPickupDistanceInches = 22.0;
    public static final double kRunOverDistanceInches = 4.0;

    // Starting Locations

    public static final PolarCoordinate kPosition1LeftStart = new PolarCoordinate(
      Units.inchesToMeters(90),
      Rotation2d.fromDegrees(122.25)
    );
    public static final PolarCoordinate kPosition2MiddleStart = new PolarCoordinate(
      Units.inchesToMeters(90),
      Rotation2d.fromDegrees(62)
    );
    public static final PolarCoordinate kPosition3RightStart = new PolarCoordinate(
      Units.inchesToMeters(90),
      Rotation2d.fromDegrees(76 + 180)
    );

    /**
     * Alliance Balls + Shooting Positions
     */

    // Ball R1 = Ball nearest to the left starting location
    public static final PolarCoordinate kBallR1 = new PolarCoordinate(
      Units.inchesToMeters(153),
      Rotation2d.fromDegrees(147.75)
    );
    public static final PolarCoordinate kBallR1Pickup = new PolarCoordinate(
      Constants.Auto.kBallR1.getRadiusMeters() - Units.inchesToMeters(kPickupDistanceInches),
      Constants.Auto.kBallR1.getTheta()
    );
    //Shoot postition between ball R2 and ball D2
    public static final PolarCoordinate kFourBallShootPosition = new PolarCoordinate(
      Units.inchesToMeters(153),
      Rotation2d.fromDegrees(205)
    );
    //Shoot postition between ball R2 and ball D2
    public static final PolarCoordinate kFiveBallShootPosition = new PolarCoordinate(
      Units.inchesToMeters(153),
      Rotation2d.fromDegrees(205)
    );
    // Ball R2 = Ball nearest to the middle starting location
    public static final PolarCoordinate kBallR2 = new PolarCoordinate(
      Units.inchesToMeters(153),
      Rotation2d.fromDegrees(212) //215.25
    );
    public static final PolarCoordinate kBallR2Pickup = new PolarCoordinate(
      Constants.Auto.kBallR2.getRadiusMeters() - Units.inchesToMeters(kPickupDistanceInches),
      Constants.Auto.kBallR2.getTheta()
    );
    // Ball R3 = Ball nearest to the right starting location
    public static final PolarCoordinate kBallR3 = new PolarCoordinate(
      Units.inchesToMeters(153),
      Rotation2d.fromDegrees(260.25)
    );
    public static final PolarCoordinate kBallR3Pickup = new PolarCoordinate(
      Constants.Auto.kBallR3.getRadiusMeters() - Units.inchesToMeters(kRunOverDistanceInches),
      Constants.Auto.kBallR3.getTheta()
    );
    public static final PolarCoordinate kBallR2ShootPosition = new PolarCoordinate(
      Constants.Auto.kBallR2.getRadiusMeters(),
      Constants.Auto.kBallR2.getTheta()
    );
    // Ball R4 = Ball just in front of the Terminal
    public static final PolarCoordinate kBallR4 = new PolarCoordinate(
      Units.inchesToMeters(305.66),
      Rotation2d.fromDegrees(202.65)
    );
    public static final PolarCoordinate kBallR4Pickup = new PolarCoordinate(
      Constants.Auto.kBallR4.getRadiusMeters() - Units.inchesToMeters(kPickupDistanceInches),
      Rotation2d.fromDegrees(201.65)
    );

    /*
     * Opponent balls
     */
    public static final PolarCoordinate kBallD2 = new PolarCoordinate(
      Units.inchesToMeters(153),
      Rotation2d.fromDegrees(190.05)
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

  public static final class Pixy {
    public static final double RATIO_TOLERANCE = 0.2;
  }

  public static final class Vision {
    public static final double IMAGE_PROCESSING_LATENCY_MS = 11;
    public static final double VISION_TARGET_OFFSET_FROM_HUB_CENTER_METERS = Units.feetToMeters(2);
  }

  public static final int CLIMBER_LEFT_MOTOR_ID = 16;
  public static final int CLIMBER_RIGHT_MOTOR_ID = 3;
  public static final int CLIMBER_STRING_POT_ID = 3; // DIO

  public static final int KICKER_MOTOR = 20;

  public static final int SHOOTER_LEFT_MOTOR = 7;
  public static final int SHOOTER_RIGHT_MOTOR = 14;

  public static final int DELIVERY_MOTOR_ID = 21;
  public static final double DELIVERY_SPEED = 0.2;

  public static final int INTAKE_MOTOR_ID = 15;
  public static final double INTAKE_FORWARD_SPEED = 1;
  public static final double INTAKE_REVERSE_SPEED = -0.5;

  public static final int INTAKE_BEAM_ID = 0;
  public static final int SHOOTER_BEAM_ID = 2;

  public static enum BallColor {
    RED,
    BLUE,
    UNKNOWN;

    public static BallColor getAllianceColor() {
      // If we're blue, return blue. Otherwise default to red (if red or invalid).
      return DriverStation.getAlliance() == Alliance.Blue ? BLUE : RED;
    }
    public static BallColor getOpposingColor() {
      // The inverse of getAllianceColor
      return DriverStation.getAlliance() == Alliance.Blue ? RED : BLUE;
    }
  }

}
