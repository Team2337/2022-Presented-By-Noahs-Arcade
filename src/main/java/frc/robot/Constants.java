package frc.robot;

import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

  public final double LIMELIGHT_CAMERA_HEIGHT_METERS;
  public final Rotation2d LIMEILGHT_CAMERA_ANGLE;

  public final int INTAKE_BEAM_ID;

  private static Constants instance;

  public static final class DashboardLogging {
    public static final boolean CLIMBER = false;
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
        INTAKE_BEAM_ID = 0;

        LIMELIGHT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(40.5);
        LIMEILGHT_CAMERA_ANGLE = new Rotation2d(Units.degreesToRadians(34));
        break;
      case PRACTICE:
        INTAKE_BEAM_ID = 0;

        LIMELIGHT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(38);
        LIMEILGHT_CAMERA_ANGLE = new Rotation2d(Units.degreesToRadians(30.91193711));
        break;
      case COMPETITION:
      default:
        INTAKE_BEAM_ID = 9;

        LIMELIGHT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(28.5);
        LIMEILGHT_CAMERA_ANGLE = new Rotation2d(Units.degreesToRadians(38.1));
        break;
    }
  }

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

    public static final PolarCoordinate kStartAtZero = new PolarCoordinate(
      Units.inchesToMeters(153),
      Rotation2d.fromDegrees(180)
    );

  }

  public static final class Drivetrain {

    public static SwerveDriveKinematics kinematicsForCurrentRobot() {
      return kinematicsForRobotType(RobotType.getRobotType());
    }

    private static SwerveDriveKinematics kinematicsForRobotType(RobotType.Type robotType) {
      Configuration configuration = configurationForRobotType(robotType);
      double drivetrainRadiusMeters = configuration.getDrivetrainRadiusMeters();
      /**
       * Should be in the same order as the swerve modules (see above)
       * Positive x values represent moving toward the front of the robot
       * positive y values represent moving toward the left of the robot
       * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
       */
      return new SwerveDriveKinematics(
        new Translation2d(
          drivetrainRadiusMeters,
          -drivetrainRadiusMeters
        ),
        new Translation2d(
          drivetrainRadiusMeters,
          drivetrainRadiusMeters
        ),
        new Translation2d(
          -drivetrainRadiusMeters,
          drivetrainRadiusMeters
        ),
        new Translation2d(
          -drivetrainRadiusMeters,
          -drivetrainRadiusMeters
        )
      );
    }

    /**
     * Sets the Track width and wheel base of the robot based on the centerpoint of
     * the swerve modules.
     * Track width is side to side
     * Wheel base is front to back.
     */
    private static class Configuration {
      private final double trackWidthMeters;
      private final double wheelBaseMeters;

      private Configuration(double trackWidthInches, double wheelBaseInches) {
        this.trackWidthMeters = Units.inchesToMeters(trackWidthInches);
        this.wheelBaseMeters = Units.inchesToMeters(wheelBaseInches);
      }

      private double getDrivetrainRadiusMeters() {
        // / 2 since we're measuring from the center - halfway
        double moduleDistanceWidthFromCenterMeters = trackWidthMeters / 2;
        double moduleDistanceLengthFromCenterMeters = wheelBaseMeters / 2;

        // Radius to the wheel modules can be thought of as a triangle - width and
        // length are the two sides
        return Math.hypot(
          moduleDistanceWidthFromCenterMeters,
          moduleDistanceLengthFromCenterMeters
        );
      }
    }

    private static Configuration configurationForRobotType(RobotType.Type robotType) {
      switch (robotType) {
        case SKILLSBOT:
          return new Configuration(10.5, 10.5);
        case PRACTICE:
        case COMPETITION:
        default:
          return new Configuration(18.75, 18.75);
      }
    }
  }

  public static final class Swerve {

    public static SwerveModule[] modulesForCurrentRobot() {
      return modulesForRobotType(RobotType.getRobotType());
    }

    private static SwerveModule[] modulesForRobotType(RobotType.Type robotType) {
      Swerve.ModuleConfiguration[] moduleConfigurations = Constants.Swerve.moduleConfigurationsForRobotType(robotType);

      // Skillsbot uses Mk3 modules, Practice/Comp use Mk4i modules
      // All robots use Falcon 500 motors + CANcoders
      switch (robotType) {
        case SKILLSBOT:
          return new SwerveModule[] {
            Mk3SwerveModuleHelper.createFalcon500(
              Mk3SwerveModuleHelper.GearRatio.STANDARD,
              moduleConfigurations[0].driveMotorID,
              moduleConfigurations[0].angleMotorID,
              moduleConfigurations[0].angleEncoderID,
              moduleConfigurations[0].angleOffset.getRadians()
            ),
            Mk3SwerveModuleHelper.createFalcon500(
              Mk3SwerveModuleHelper.GearRatio.STANDARD,
              moduleConfigurations[1].driveMotorID,
              moduleConfigurations[1].angleMotorID,
              moduleConfigurations[1].angleEncoderID,
              moduleConfigurations[1].angleOffset.getRadians()
            ),
            Mk3SwerveModuleHelper.createFalcon500(
              Mk3SwerveModuleHelper.GearRatio.STANDARD,
              moduleConfigurations[2].driveMotorID,
              moduleConfigurations[2].angleMotorID,
              moduleConfigurations[2].angleEncoderID,
              moduleConfigurations[2].angleOffset.getRadians()
            ),
            Mk3SwerveModuleHelper.createFalcon500(
              Mk3SwerveModuleHelper.GearRatio.STANDARD,
              moduleConfigurations[3].driveMotorID,
              moduleConfigurations[3].angleMotorID,
              moduleConfigurations[3].angleEncoderID,
              moduleConfigurations[3].angleOffset.getRadians()
            )
          };
        case PRACTICE:
        case COMPETITION:
        default:
          return new SwerveModule[] {
            Mk4SwerveModuleHelper.createFalcon500(
              Mk4SwerveModuleHelper.GearRatio.L1I,
              moduleConfigurations[0].driveMotorID,
              moduleConfigurations[0].angleMotorID,
              moduleConfigurations[0].angleEncoderID,
              moduleConfigurations[0].angleOffset.getRadians()
            ),
            Mk4SwerveModuleHelper.createFalcon500(
              Mk4SwerveModuleHelper.GearRatio.L1I,
              moduleConfigurations[1].driveMotorID,
              moduleConfigurations[1].angleMotorID,
              moduleConfigurations[1].angleEncoderID,
              moduleConfigurations[1].angleOffset.getRadians()
            ),
            Mk4SwerveModuleHelper.createFalcon500(
              Mk4SwerveModuleHelper.GearRatio.L1I,
              moduleConfigurations[2].driveMotorID,
              moduleConfigurations[2].angleMotorID,
              moduleConfigurations[2].angleEncoderID,
              moduleConfigurations[2].angleOffset.getRadians()
            ),
            Mk4SwerveModuleHelper.createFalcon500(
              Mk4SwerveModuleHelper.GearRatio.L1I,
              moduleConfigurations[3].driveMotorID,
              moduleConfigurations[3].angleMotorID,
              moduleConfigurations[3].angleEncoderID,
              moduleConfigurations[3].angleOffset.getRadians()
            )
          };
      }
    }

    private static class ModuleConfiguration {
      private final int driveMotorID;
      private final int angleMotorID;
      private final int angleEncoderID;
      private final Rotation2d angleOffset;

      private ModuleConfiguration(int driveMotorID, int angleMotorID, int angleEncoderID, Rotation2d angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.angleEncoderID = angleEncoderID;
        this.angleOffset = angleOffset;
      }
    }

    private static ModuleConfiguration[] moduleConfigurationsForRobotType(RobotType.Type robotType) {
      switch (robotType) {
        case SKILLSBOT:
          return new ModuleConfiguration[] {
            new ModuleConfiguration(0, 4, 1, Rotation2d.fromDegrees(-50.701904296875)), // 0
            new ModuleConfiguration(1, 5, 2, Rotation2d.fromDegrees(-128.58123779296875)), // 1
            new ModuleConfiguration(14, 10, 3, Rotation2d.fromDegrees(-346.63238525390625)), // 2
            new ModuleConfiguration(15, 11, 4, Rotation2d.fromDegrees(-286.42730712890625)) // 3
          };
        case PRACTICE:
          return new ModuleConfiguration[] {
            new ModuleConfiguration(18, 19, 1, Rotation2d.fromDegrees(-130.599976)), // 0
            new ModuleConfiguration(1, 2, 2, Rotation2d.fromDegrees(-175.163269)), // 1
            new ModuleConfiguration(8, 9, 3, Rotation2d.fromDegrees(-278.338623)), // 2
            new ModuleConfiguration(10, 11, 4, Rotation2d.fromDegrees(-355.860901)) // 3
          };
        case COMPETITION:
        default:
          return new ModuleConfiguration[] {
            new ModuleConfiguration(18, 19, 1, Rotation2d.fromDegrees(-76.37109375)), // 0
            new ModuleConfiguration(1, 2, 2, Rotation2d.fromDegrees(-204.430078125)), // 1
            new ModuleConfiguration(8, 9, 3, Rotation2d.fromDegrees(-195.37382812500002)), // 2
            new ModuleConfiguration(10, 11, 4, Rotation2d.fromDegrees(-255.3140625)) // 3
          };
      }
    }

    public static class Speeds {

      public static final double MAX_VOLTAGE = 12.0;

      private ModuleConfiguration swerveModuleConfiguration;
      private Drivetrain.Configuration drivetrainConfiguration;

      private Speeds(swervelib.ModuleConfiguration swerveModuleConfiguration, Drivetrain.Configuration drivetrainConfiguration) {
        this.swerveModuleConfiguration = swerveModuleConfiguration;
        this.drivetrainConfiguration = drivetrainConfiguration;
      }

      /**
       * The maximum velocity of the robot in meters per second.
       * <p>
       * This is a measure of how fast the robot should be able to drive in a straight line.
       *
       * The formula for calculating the theoretical maximum velocity is:
       *   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
       * By default this value is setup for a Mk4 L1 module using Falcon500s to drive.
       */
      public double getMaxVelocityMetersPerSecond() {
        // 6380.0 == Falcon 500 free speed RPM
        return 6380.0 / 60.0 *
          swerveModuleConfiguration.getDriveReduction() *
          swerveModuleConfiguration.getWheelDiameter() * Math.PI;
      }
    }

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
    public static final double RATIO_TOLERANCE = 0.4;
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
  public static final double DELIVERY_SPEED = 0.275;

  public static final int INTAKE_MOTOR_ID = 15;
  public static final double INTAKE_FORWARD_SPEED = 1;
  public static final double INTAKE_REVERSE_SPEED = -0.5;

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
