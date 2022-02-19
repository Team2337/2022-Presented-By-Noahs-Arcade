package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.CTREUtils;
import frc.robot.nerdyfiles.utilities.Utilities;
/**
 * This subsystem runs the two shooter motors and gets them up to a constant set speed
 *
 * @author Nicholas S.
 */

public class Shooter extends SubsystemBase {

  private static double kMotorShutdownTempCelcius = 70;
  private static double kShooterSpeedFeetPerSecondTolerance = 0.1;

  // Top Wheel == Left Motor, Bottom Wheel == Right Motor
  public TalonFX leftMotor = new TalonFX(Constants.SHOOTER_LEFT_MOTOR);
  public TalonFX rightMotor = new TalonFX(Constants.SHOOTER_RIGHT_MOTOR);

  // This is for 40.7 ft/s, RING OF FIRE!!!
  private double kP = 0.10;
  private double kI = 0;
  private double kD = 0.000;
  private double kF = 0.055;

  private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

  private ShuffleboardLayout pid = tab.getLayout("PID Control", BuiltInLayouts.kList)
    .withSize(4, 8)
    .withPosition(0, 0);
  private NetworkTableEntry kep = pid
    .add("kP", kP)
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();
  private NetworkTableEntry kei = pid
    .add("kI", kI)
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();
  private NetworkTableEntry ked = pid
    .add("kD", kD)
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();
  private NetworkTableEntry kef = pid
    .add("kF", kF)
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();

  private ShuffleboardLayout speeds = tab.getLayout("Shooter Speeds", BuiltInLayouts.kList)
    .withSize(4, 8)
    .withPosition(4, 0);
  public NetworkTableEntry shooterSpeedFeetPerSecondWidget = speeds
    .add("Set Wheel Speed (ft/s)", 40.7)
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();

  private StatorCurrentLimitConfiguration currentLimitConfiguration = CTREUtils.defaultCurrentLimit();

  public Shooter() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    rightMotor.follow(leftMotor);
    leftMotor.setNeutralMode(NeutralMode.Coast);

    leftMotor.setInverted(TalonFXInvertType.CounterClockwise);
    rightMotor.setInverted(TalonFXInvertType.Clockwise);

    leftMotor.configStatorCurrentLimit(currentLimitConfiguration, 0);
    leftMotor.configClosedloopRamp(0.1);
    leftMotor.enableVoltageCompensation(true);

    configurePID(kP, kI, kD, kF);

    setupShuffleboard();
  }

  private void setupShuffleboard() {
    speeds.addNumber("Left Motor RPM", () -> getMotorRPM(leftMotor));
    speeds.addNumber("Right Motor RPM", () -> getMotorRPM(rightMotor));
    speeds.addNumber("Top Wheel Speed (ft/s)", () -> getMotorWheelSpeed(leftMotor));
    speeds.addNumber("Bottom Wheel Speed (ft/s)", () -> getMotorWheelSpeed(rightMotor));
    speeds.addNumber("Left Motor Velocity", () -> leftMotor.getSelectedSensorVelocity());
    speeds.addNumber("Right Motor Velocity", () -> rightMotor.getSelectedSensorVelocity());

    ShuffleboardLayout temps = tab.getLayout("Shooter Temperature", BuiltInLayouts.kList)
      .withSize(4, 8)
      .withPosition(8, 0);
    temps.addNumber("Left Motor Temperature", () -> leftMotor.getTemperature());
    temps.addNumber("Right Motor Temperature", () -> rightMotor.getTemperature());
    temps.addBoolean("Motors Overheating?", () -> isOverheated());
  }

  @Override
  public void periodic() {
    if (kep.getDouble(0) != kP) {
      kP = kep.getDouble(0);
      configurePID(kP, kI, kD, kF);
    }
    if (kei.getDouble(0) != kI) {
      kI = kei.getDouble(0);
      configurePID(kP, kI, kD, kF);
    }
    if (ked.getDouble(0) != kD) {
      kD = ked.getDouble(0);
      configurePID(kP, kI, kD, kF);
    }
    if (kef.getDouble(0) != kF) {
      kF = kef.getDouble(0);
      configurePID(kP, kI, kD, kF);
    }
  }

  public void configurePID(double kp, double ki, double kd, double kf) {
    leftMotor.config_kP(0, kp);
    leftMotor.config_kI(0, ki);
    leftMotor.config_kD(0, kd);
    leftMotor.config_kF(0, kf);
  }

  public boolean getLeftMotorOverTemp() {
    return leftMotor.getTemperature() >= kMotorShutdownTempCelcius;
  }

  public boolean getRightMotorOverTemp() {
    return rightMotor.getTemperature() >= kMotorShutdownTempCelcius;
  }

  public void setSpeed(double speedFeetPerSecond) {
    double ticksPerHundredMiliseconds = feetPerSecondToTicksPerOneHundredMs(speedFeetPerSecond);
    SmartDashboard.putNumber("Ticks per 100ms", ticksPerHundredMiliseconds);

    enableMotorCurrentLimiting();
    leftMotor.set(ControlMode.Velocity, ticksPerHundredMiliseconds);
  }

  public void stop() {
    disableMotorCurrentLimiting();
    leftMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public boolean isShooterToSpeed() {
    // TODO: Tune our deadband range
    return Utilities.withinTolerance(shooterSpeedFeetPerSecondWidget.getDouble(0), getMotorWheelSpeed(leftMotor), kShooterSpeedFeetPerSecondTolerance);
  }

  public boolean isOverheated() {
    return getLeftMotorOverTemp() || getRightMotorOverTemp();
  }

  private double getMotorRPM(TalonFX motor) {
    //Encoder ticks per 100 ms
    double speed = motor.getSelectedSensorVelocity();
    // Encoder ticks per second
    double tps = speed * 10.0;
    // Encoder revolutions per second
    double rps = tps / 2048.0;
    // Convert rps into revolutions per minute
    return rps * 60.0;
  }

  private double getMotorWheelSpeed(TalonFX motor) {
    double wheelDiameterInches = 4.0;
    double rpm = getMotorRPM(motor);
    double wheelRpm = rpm * (16.0 / 24.0); // 16/24 is the gear ratio (16 is the input gear of the falcons, 24 is the output gear of the wheel)
    return ((2.0 * Math.PI* wheelRpm) / 60.0) * ((wheelDiameterInches / 12.0) / 2.0); //This turns wheel RPM's into ft/s
  }

  private double feetPerSecondToTicksPerOneHundredMs(double speedFeetPerSecond) {
    // 4in wheel
    double wheelDiameterFeet = 4.0 / 12.0;
    double wheelRadiusFeet = wheelDiameterFeet / 2.0;
    double feetPerRotation = (Math.PI * 2.0 * wheelRadiusFeet);
    // 16 / 24 gear ratio
    double rotationsPerSecond = (speedFeetPerSecond / feetPerRotation) * (24.0 / 16.0);
    // 2048 ticks per rotation
    double ticksPerSecond = rotationsPerSecond * 2048.0;
    return ticksPerSecond / 10.0;
  }

  private void enableMotorCurrentLimiting() {
    toggleMotorCurrentLimiting(true);
  }

  private void disableMotorCurrentLimiting() {
    toggleMotorCurrentLimiting(false);
  }

  private void toggleMotorCurrentLimiting(boolean enable) {
    if (currentLimitConfiguration.enable != enable) {
      currentLimitConfiguration.enable = enable;
      leftMotor.configStatorCurrentLimit(currentLimitConfiguration, 0);
    }
  }

}
