package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SystemsCheckPositions;
import frc.robot.nerdyfiles.utilities.CTREUtils;
import frc.robot.nerdyfiles.utilities.Utilities;
/**
 * This subsystem runs the two shooter motors and gets them up to a constant set speed
 *
 * @author Nicholas S.
 */

public class Shooter extends SubsystemBase {

  private static double kP = 0.06;
  private static double kI = 0;
  private static double kD = 0.000;
  private static double kF = 0.05;

  private static double kMotorShutdownTempCelcius = 70;
  private static double kShooterSpeedFeetPerSecondTolerance = 7;

  // Top Wheel == Left Motor, Bottom Wheel == Right Motor
  public TalonFX leftMotor = new TalonFX(
    Constants.SHOOTER_LEFT_MOTOR,
    Constants.UPPER_CANIVORE_ID
  );
  public TalonFX rightMotor = new TalonFX(
    Constants.SHOOTER_RIGHT_MOTOR,
    Constants.UPPER_CANIVORE_ID
  );

  private double targetSpeed = 0.0;

  private StatorCurrentLimitConfiguration currentLimitConfiguration = CTREUtils.defaultCurrentLimit();

  public Shooter() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    configureShooterMotor(leftMotor);
    rightMotor.follow(leftMotor);

    leftMotor.setNeutralMode(NeutralMode.Coast);
    rightMotor.setNeutralMode(NeutralMode.Coast);

    leftMotor.setInverted(TalonFXInvertType.CounterClockwise);
    rightMotor.setInverted(TalonFXInvertType.Clockwise);

    leftMotor.enableVoltageCompensation(true);
    rightMotor.enableVoltageCompensation(true);

    setupShuffleboard(Constants.DashboardLogging.SHOOTER);
    // SmartDashboard.putNumber("Shooter Speed X", 0);
  }

  private ErrorCode configureShooterMotor(TalonFX motor) {
    TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.statorCurrLimit = currentLimitConfiguration;

    configuration.slot0.kP = kP;
    configuration.slot0.kI = kI;
    configuration.slot0.kD = kD;
    configuration.slot0.kF = kF;

    configuration.closedloopRamp = 0.2;

    configuration.voltageCompSaturation = 9;

    return motor.configAllSettings(configuration, 250);
  }

  private void setupShuffleboard(Boolean logEnable) {
    if (logEnable) {
      ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

      ShuffleboardLayout temps = tab.getLayout("Shooter Temperature", BuiltInLayouts.kList)
        .withSize(4, 8)
        .withPosition(8, 0);
      temps.addNumber("Left Motor Temperature", () -> leftMotor.getTemperature());
      temps.addNumber("Right Motor Temperature", () -> rightMotor.getTemperature());
      temps.addBoolean("Motors Overheating?", () -> isOverheated());

      ShuffleboardLayout speeds = tab.getLayout("Shooter Speeds", BuiltInLayouts.kList)
        .withSize(4, 8)
        .withPosition(4, 0);
      speeds.addNumber("Left Motor RPM", () -> getMotorRPM(leftMotor));
      speeds.addNumber("Right Motor RPM", () -> getMotorRPM(rightMotor));
      speeds.addNumber("Top Wheel Speed (ft per s)", () -> getMotorWheelSpeed(leftMotor));
      speeds.addNumber("Bottom Wheel Speed (ft per s)", () -> getMotorWheelSpeed(rightMotor));
      speeds.addNumber("Left Motor Velocity", () -> leftMotor.getSelectedSensorVelocity());
      speeds.addNumber("Right Motor Velocity", () -> rightMotor.getSelectedSensorVelocity());
    }

    if (Constants.DO_SYSTEMS_CHECK) {
      ShuffleboardTab systemsCheck = Constants.SYSTEMS_CHECK_TAB;

      systemsCheck.addNumber("L Shooter Temp (C)", () -> leftMotor.getTemperature())
        .withPosition(SystemsCheckPositions.L_SHOOTER_TEMP.x, SystemsCheckPositions.L_SHOOTER_TEMP.y)
        .withSize(3, 4)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Min", Constants.MOTOR_MINIMUM_TEMP_CELSIUS, "Max", Constants.MOTOR_SHUTDOWN_TEMP_CELSIUS));
      systemsCheck.addNumber("R Shooter Temp (C)", () -> rightMotor.getTemperature())
        .withPosition(SystemsCheckPositions.R_SHOOTER_TEMP.x, SystemsCheckPositions.R_SHOOTER_TEMP.y)
        .withSize(3, 4)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Min", Constants.MOTOR_MINIMUM_TEMP_CELSIUS, "Max", Constants.MOTOR_SHUTDOWN_TEMP_CELSIUS));
    }
  }

  @Override
  public void periodic() {
    if (isOverheated()) {
      stop();
    }
  }

  // ** Public API **

  public void setSpeed(double speedFeetPerSecond) {
    if (isOverheated()) {
      return;
    }

    if (speedFeetPerSecond != targetSpeed){
      targetSpeed = speedFeetPerSecond;
    }
  
    // speedFeetPerSecond = SmartDashboard.getNumber("Shooter Speed X", 0);
    double ticksPerHundredMiliseconds = feetPerSecondToTicksPerOneHundredMs(speedFeetPerSecond);

    enableMotorCurrentLimiting();
    leftMotor.set(ControlMode.Velocity, ticksPerHundredMiliseconds);
  }

  public void stop() {
    disableMotorCurrentLimiting();
    leftMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public boolean isShooterToSpeed() {
    return Utilities.withinTolerance(targetSpeed, getMotorWheelSpeed(leftMotor), kShooterSpeedFeetPerSecondTolerance);
  }

  // Mimics isShooterToSpeed(), but adds if the motor is over 10 ft/s so the LEDs do not activate when the motor is stopped
  public boolean isShooterToLEDSpeed() {
    return Utilities.withinTolerance(targetSpeed, getMotorWheelSpeed(leftMotor), kShooterSpeedFeetPerSecondTolerance) && getMotorWheelSpeed(leftMotor) > 10;
  }

  // ** Private API **

  public double getVelocity() {
    return leftMotor.getSelectedSensorVelocity();
  }

  private boolean isOverheated() {
    return isMotorOverheated(leftMotor) || isMotorOverheated(rightMotor);
  }

  private boolean isMotorOverheated(TalonFX motor) {
    return motor.getTemperature() >= kMotorShutdownTempCelcius;
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
