package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotType;
import frc.robot.RobotType.Type;

public class Drivetrain extends SubsystemBase {

  /**
   * Inputs
   */
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  /**
   * Hardware (private)
   */
  private PigeonIMU pigeon;

  /**
   * Array for swerve module objects, sorted by ID
   * 0 is Front Right,
   * 1 is Front Left,
   * 2 is Back Left,
   * 3 is Back Right
   */
  private SwerveModule[] modules;

  /**
   * Logging
   */
  private ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
  private Field2d field = new Field2d();
  private Logger logger = Logger.getInstance();

  /**
   * Should be in the same order as the swerve modules (see above)
   * Positive x values represent moving toward the front of the robot
   * positive y values represent moving toward the left of the robot
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
   */
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(
      Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES),
      Units.inchesToMeters(-Constants.DRIVETRAIN_RADIUS_INCHES)
    ),
    new Translation2d(
      Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES),
      Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES)
    ),
    new Translation2d(
      Units.inchesToMeters(-Constants.DRIVETRAIN_RADIUS_INCHES),
      Units.inchesToMeters(Constants.DRIVETRAIN_RADIUS_INCHES)
    ),
    new Translation2d(
      Units.inchesToMeters(-Constants.DRIVETRAIN_RADIUS_INCHES),
      Units.inchesToMeters(-Constants.DRIVETRAIN_RADIUS_INCHES)
    )
  );

  private SwerveDriveOdometry odometry;

  /**
   * Subsystem where swerve modules are configured,
   * and the calculations from the joystick inputs is handled.
   * Field orientation is set here as well
   */
  public Drivetrain(PigeonIMU pigeon) {
    this.pigeon = pigeon;

    odometry = new SwerveDriveOdometry(kinematics, getGyroscopeRotation());

    modules = new SwerveModule[] {
      Mk4SwerveModuleHelper.createFalcon500(
        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
          .withSize(4, 8)
          .withPosition(4, 0),
        RobotType.getRobotType() == Type.SKILLSBOT ? Mk4SwerveModuleHelper.GearRatio.L1 : Mk4SwerveModuleHelper.GearRatio.L1I,
        Constants.getInstance().MODULE0_DRIVE_MOTOR_ID,
        Constants.getInstance().MODULE0_ANGLE_MOTOR_ID,
        Constants.getInstance().MODULE0_ANGLE_CANCODER_ID,
        Constants.getInstance().MODULE0_ANGLE_OFFSET
      ),
      Mk4SwerveModuleHelper.createFalcon500(
        tab.getLayout("Front Left Module", BuiltInLayouts.kList)
          .withSize(4, 8)
          .withPosition(0, 0),
        RobotType.getRobotType() == Type.SKILLSBOT ? Mk4SwerveModuleHelper.GearRatio.L1 : Mk4SwerveModuleHelper.GearRatio.L1I,
        Constants.getInstance().MODULE1_DRIVE_MOTOR_ID,
        Constants.getInstance().MODULE1_ANGLE_MOTOR_ID,
        Constants.getInstance().MODULE1_ANGLE_CANCODER_ID,
        Constants.getInstance().MODULE1_ANGLE_OFFSET
      ),
      Mk4SwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
          .withSize(4, 8)
          .withPosition(0, 8),
        RobotType.getRobotType() == Type.SKILLSBOT ? Mk4SwerveModuleHelper.GearRatio.L1 : Mk4SwerveModuleHelper.GearRatio.L1I,
        Constants.getInstance().MODULE2_DRIVE_MOTOR_ID,
        Constants.getInstance().MODULE2_ANGLE_MOTOR_ID,
        Constants.getInstance().MODULE2_ANGLE_CANCODER_ID,
        Constants.getInstance().MODULE2_ANGLE_OFFSET
      ),
      Mk4SwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Right Module", BuiltInLayouts.kList)
          .withSize(4, 8)
          .withPosition(4, 8),
        RobotType.getRobotType() == Type.SKILLSBOT ? Mk4SwerveModuleHelper.GearRatio.L1 : Mk4SwerveModuleHelper.GearRatio.L1I,
        Constants.getInstance().MODULE3_DRIVE_MOTOR_ID,
        Constants.getInstance().MODULE3_ANGLE_MOTOR_ID,
        Constants.getInstance().MODULE3_ANGLE_CANCODER_ID,
        Constants.getInstance().MODULE3_ANGLE_OFFSET
      )
    };

    SmartDashboard.putData("Field", field);

    setupShuffleboard();
  }

  private void setupShuffleboard() {
    ShuffleboardLayout chassisSpeedsWidget = tab.getLayout("Chassis Speeds", BuiltInLayouts.kList).withSize(4, 8).withPosition(12, 0);
    chassisSpeedsWidget.addNumber("vx meters/s", () -> chassisSpeeds.vxMetersPerSecond);
    chassisSpeedsWidget.addNumber("vy meters/s", () -> chassisSpeeds.vyMetersPerSecond);
    chassisSpeedsWidget.addNumber("omega radians/s", () -> chassisSpeeds.omegaRadiansPerSecond);

    ShuffleboardLayout gyroWidget = tab.getLayout("Gyro", BuiltInLayouts.kList).withSize(4, 8).withPosition(16, 0);
    gyroWidget.addNumber("Degrees", () -> getGyroscopeRotation().getDegrees());
  }

  public void resetPosition(Pose2d pose) {
    odometry.resetPosition(pose, getGyroscopeRotation());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Get the gyroscope rotation of the robot as measured by the
   * yaw value of the Pigeon. [-368,640, 368,640] degrees.
   * Counter-clockwise is interpreted as a positive change,
   * clockwise is interpreted as a negative change.
   *
   * @return The rotation of the robot.
   */
  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(pigeon.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    this.chassisSpeeds = chassisSpeeds;
  }

  public void resetOdometry() {
    odometry.resetPosition(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0));
  }

  /**
   * Stops all of the motors on each module
   */
  public void stopMotors() {
    this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);

    for (int i = 0; i < states.length; i++) {
      SwerveModule module = modules[i];
      SwerveModuleState moduleState = states[i];
      module.set(moduleState.speedMetersPerSecond / Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * Constants.Swerve.MAX_VOLTAGE, moduleState.angle.getRadians());
    }

    Pose2d pose = odometry.update(
      getGyroscopeRotation(),
      getModuleState(modules[0]),
      getModuleState(modules[1]),
      getModuleState(modules[2]),
      getModuleState(modules[3])
    );

    field.setRobotPose(getPose());

    logger.recordOutput("Odometry/Robot",
      new double[] { pose.getX(), pose.getY(), pose.getRotation().getRadians() });

      SmartDashboard.putNumber("Pose X", pose.getX());
      SmartDashboard.putNumber("Pose Y", pose.getY());
      SmartDashboard.putNumber("Pose Degrees", pose.getRotation().getDegrees());
  
      // SmartDashboard.putNumber("Target X", Constants.Auto.kBlueBall3Pickup.toFieldCoordinate().getX());
      // SmartDashboard.putNumber("Target Y", Constants.Auto.kBlueBall3Pickup.toFieldCoordinate().getY());

    logger.recordOutput("Gyro", pigeon.getYaw());
  }

  private static final SwerveModuleState getModuleState(SwerveModule module) {
    return new SwerveModuleState(
        module.getDriveVelocity(),
        new Rotation2d(module.getSteerAngle())
      );
  }

}
