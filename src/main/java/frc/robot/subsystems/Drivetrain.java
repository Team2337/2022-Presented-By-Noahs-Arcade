package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.SwerveModule;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  /**
   * Hardware (private)
   */
  private PigeonIMU pigeon;

  private SwerveDriveKinematics kinematics;

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

  private final SwerveDrivePoseEstimator odometry;

  // The chassis speeds is the actual chassis speeds object of the robot
  // calculated using the kinematics model + the module states
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  // The drive chassis speeds are the requested chassis speeds to be moving
  private ChassisSpeeds driveChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  // Update Drivetrain state only once per cycle
  private Pose2d pose = new Pose2d();
  // Array for Yaw Pitch and Roll values in degrees
  public double[] ypr_deg = { 0, 0, 0 };

  /**
   * Subsystem where swerve modules are configured,
   * and the calculations from the joystick inputs is handled.
   * Field orientation is set here as well
   */
  public Drivetrain(PigeonIMU pigeon) {
    this.pigeon = pigeon;

    kinematics = Constants.Drivetrain.kinematicsForCurrentRobot();

    odometry = new SwerveDrivePoseEstimator(
      getGyroscopeRotation(),
      pose,
      kinematics,
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(Units.degreesToRadians(0.01)),
      VecBuilder.fill(0.25, 0.25, Units.degreesToRadians(5))
    );

    modules = Constants.Swerve.modulesForCurrentRobot();

    setupShuffleboard(Constants.DashboardLogging.DRIVETRAIN);
  }

  private void setupShuffleboard(Boolean logEnable) {
    if (logEnable) {
      ShuffleboardLayout chassisSpeedsWidget = tab.getLayout("Chassis Speeds", BuiltInLayouts.kList).withSize(4, 8).withPosition(12, 0);
      chassisSpeedsWidget.addNumber("vx meters per s", () -> chassisSpeeds.vxMetersPerSecond);
      chassisSpeedsWidget.addNumber("vy meters per s", () -> chassisSpeeds.vyMetersPerSecond);
      chassisSpeedsWidget.addNumber("omega radians per s", () -> chassisSpeeds.omegaRadiansPerSecond);
    }
    ShuffleboardLayout gyroWidget = tab.getLayout("Gyro", BuiltInLayouts.kList).withSize(4, 8).withPosition(16, 0);
    gyroWidget.addNumber("Degrees", () -> getGyroscopeRotation().getDegrees());
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
    odometry.addVisionMeasurement(visionPose, timestampSeconds);
  }

  public void resetPosition(Pose2d pose) {
    odometry.resetPosition(pose, getGyroscopeRotation());
    pose = odometry.getEstimatedPosition();
  }

  public Pose2d getPose() {
    return pose;
  }

  public Translation2d getTranslation() {
    return getPose().getTranslation();
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

  public double getGyroscopeRoll(){
    return pigeon.getRoll();
  }

  public double getGyroscopePitch(){
    return pigeon.getPitch();
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    this.driveChassisSpeeds = chassisSpeeds;
  }

  /**
   * Get the current chassis speeds object for the drivetrain.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return chassisSpeeds;
  }

  /**
   * Get the combined vx + vy velocity vector for the robot.
   */
  public double velocity() {
    return Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
  }

  public boolean isMoving() {
    return velocity() > 0;
  }

  /**
   * Stops all of the motors on each module
   */
  public void stopMotors() {
    this.driveChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(driveChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);

    for (int i = 0; i < states.length; i++) {
      SwerveModule module = modules[i];
      SwerveModuleState moduleState = states[i];
      module.set(moduleState.speedMetersPerSecond / Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND * Constants.Swerve.MAX_VOLTAGE, moduleState.angle.getRadians());
    }

    SwerveModuleState[] realModuleStates = {
      getModuleState(modules[0]),
      getModuleState(modules[1]),
      getModuleState(modules[2]),
      getModuleState(modules[3]),
    };

    Pose2d pose = odometry.update(
      getGyroscopeRotation(),
      realModuleStates
    );

    chassisSpeeds = kinematics.toChassisSpeeds(
      realModuleStates
    );

    Logger.getInstance().recordOutput("Odometry/Robot",
      new double[] { pose.getX(), pose.getY(), pose.getRotation().getRadians() });

    Logger.getInstance().recordOutput("Gyro", pigeon.getYaw());
  }

  private static final SwerveModuleState getModuleState(SwerveModule module) {
    return new SwerveModuleState(
      module.getDriveVelocity(),
      new Rotation2d(module.getSteerAngle())
    );
  }

}
