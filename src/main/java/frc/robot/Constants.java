package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.util.Units;

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

  public static final int KICKER_MOTOR = 20;

  public static final int SHOOTER_LEFT_MOTOR = 7;
  public static final int SHOOTER_RIGHT_MOTOR = 14;

  public static final int DELIVERY_MOTOR_ID = 21;
  public static final double DELIVERY_SPEED = 0.5;

  //CLIMBER CONSTANTS
  public static final int LEFT_CLIMBER_MOTOR_ID = 16;
  public static final int RIGHT_CLIMBER_MOTOR_ID = 3;
  public static final int CLIMBER_STRING_POT_ID = 3;

  public static final class Climber {
    public static final double START = 0;
    public static final double LOW_RUNG = 1.0;
    public static final double MID_RUNG = 1.5;
    public static final double RICKABOOT = 0.7;
  }

  // Color stuff
  public static final int INTAKE_MOTOR_ID = 15;
  public static final double INTAKE_FORWARD_SPEED = 1;
  public static final double INTAKE_REVERSE_SPEED = -0.5;

  public static final int INTAKE_SENSOR_ID = 0;

  public static enum BallColor {
    RED,
    BLUE,
    UNKNOWN
  }

}
