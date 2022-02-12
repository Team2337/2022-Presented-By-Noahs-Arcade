package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import java.util.Map;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities;
/**
 * This subsystem runs the two shooter motors and gets them up to a constant set speed
 * 
 * @author Nicholas S.
 */

public class Shooter extends SubsystemBase {

  public TalonFX leftMotor = new TalonFX(Constants.SHOOTER_LEFT_MOTOR);
  public TalonFX rightMotor = new TalonFX(Constants.SHOOTER_RIGHT_MOTOR);
  private StatorCurrentLimitConfiguration currentLimitConfigurationMotor = new StatorCurrentLimitConfiguration();
  //This is for 40.7 ft/s, RING OF FIRE!!!
  private double kP = 0.62; 
  private double kI = 0;
  private double kD = 0.000;
  private double kF = 0.055;
  private double speedTolerance = 0.1;
  private double topSpeed = 0;
  private double prevTopSpeed = 0;
  private int counter = 0;
  private static double kMotorShutdownTemp = 70; //In Degrees Celsius


  private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

  ShuffleboardLayout pid = tab.getLayout("PID Control", BuiltInLayouts.kList)
    .withSize(4, 8)
    .withPosition(0, 0);
  ShuffleboardLayout speeds = tab.getLayout("Shooter Speeds", BuiltInLayouts.kList)
    .withSize(4, 8)
    .withPosition(4, 0); 
  ShuffleboardLayout temps = tab.getLayout("Shooter Temperature", BuiltInLayouts.kList)
    .withSize(4, 8)
    .withPosition(8, 0); 
  ShuffleboardLayout speed = tab.getLayout("Motor Velocities", BuiltInLayouts.kList)
    .withSize(4, 8)
    .withPosition(12, 0); 

  public NetworkTableEntry kep = pid
    .add("kP", kP )
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();  
  public NetworkTableEntry kei = pid
    .add("kI", kI )
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();   
  public NetworkTableEntry ked = pid
    .add("kD", kD )
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();  
  public NetworkTableEntry kef = pid
    .add("kF", kF )
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();  
  public NetworkTableEntry shooter = speeds
    .add("Shooter Speed", 40.7)
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();

  public Shooter() {
    /** --- CONFIGURE MOTOR AND SENSOR SETTINGS --- **/
    // Configures motors to factory default
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    rightMotor.follow(leftMotor);

    /** --- SETS UP SETTINGS (Such as current limits) ON MOTORS AND SENSORS --- **/
    // Set up current limits
    currentLimitConfigurationMotor.currentLimit = 50; //50
    currentLimitConfigurationMotor.enable = true;
    currentLimitConfigurationMotor.triggerThresholdCurrent = 40;
    currentLimitConfigurationMotor.triggerThresholdTime = 3;
    // Implements these current limits on the motors
    leftMotor.configStatorCurrentLimit(currentLimitConfigurationMotor, 0);

    // Set a closed-loop ramp rate on the motors
    leftMotor.configClosedloopRamp(0.1);
    // Enable voltage compensation for all control modes on the motors
    leftMotor.enableVoltageCompensation(true);

    /** --- CONFIGURE PIDS --- **/
    // Implement variables into the PIDs
    configurePID(kP, kI, kD, kF);

    /** --- BRAKE MODES AND INVERSIONS --- **/
    // Sets up control mode.
    // Sets it to neutral mode so that the motors do not brake down to 0.
    leftMotor.setNeutralMode(NeutralMode.Coast);
    // Sets up inversions
    leftMotor.setInverted(TalonFXInvertType.Clockwise);
    rightMotor.setInverted(InvertType.OpposeMaster);

    temps.addNumber("Top Shooter Temperature", () -> leftMotor.getTemperature());
    temps.addNumber("Bottom Shooter Temperature", () -> rightMotor.getTemperature());
    temps.addBoolean("Motors Overheating?", () -> isMotorOverheated());
    speed.addNumber("Left Motor RPM", () -> getMotorRPM(leftMotor));
    speed.addNumber("Right Motor RPM", () -> getMotorRPM(rightMotor));
    speed.addNumber("Left Motor Velocity", () -> leftMotor.getSelectedSensorVelocity());
    speed.addNumber("Right Motor Velocity", () -> rightMotor.getSelectedSensorVelocity());
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

    //This makes it so the shooter speed isn't constantly rewriting itself
    if (shooter.getDouble(0) != topSpeed) {
      if (prevTopSpeed == shooter.getDouble(0)) {
        counter++;
      }
      if (counter == 50){
        topSpeed = shooter.getDouble(0);
        counter = 0;
      }
      prevTopSpeed = shooter.getDouble(0);

    }
  }

  public void configurePID(double kp, double ki, double kd, double kf){
    leftMotor.config_kP(0, kp);
    leftMotor.config_kI(0, ki);
    leftMotor.config_kD(0, kd);
    leftMotor.config_kF(0, kf);
  }

  public void stopShooter(){
    leftMotor.set(ControlMode.PercentOutput, 0.0);
  }
  public double getTopMotorSpeed(){
    return leftMotor.getMotorOutputPercent();
  }
  public double getBottomMotorSpeed(){
    return rightMotor.getMotorOutputPercent();
  }

  public boolean getTopMotorOverTemp() {
    return leftMotor.getTemperature() > kMotorShutdownTemp;
  }

  public boolean getBottomMotorOverTemp() {
    return rightMotor.getTemperature() > kMotorShutdownTemp;
  }

  public double getMotorRPM(TalonFX motor){
    //Encoder ticks per 100 ms
    double speed = motor.getSelectedSensorVelocity();
    // Encoder ticks per second
    double tps = speed * 10.0;
    // Encoder revolutions per second            
    double rps = tps / 2048.0;
    // Convert rps into revolutions per minute
    double rpm = rps * 60.0;
    return rpm;
  }

  public double getMotorWheelSpeed(TalonFX motor){
    double wheelDiameter = 4.0; //This is in inches.
    double rpm = getMotorRPM(motor);
    double wheelRpm = rpm * (16.0/24.0); //16/24 is the gear ratio (16 is the input gear of the falcons, 24 is the output gear of the wheel)
    double wheelSpeed = ((2.0*Math.PI*wheelRpm)/60.0)*((wheelDiameter/12.0)/2.0); //This turns wheel RPM's into ft/s
    return wheelSpeed;
  }
  
  public void setShooterSpeed(double speed) {
    // 4in wheel
    double wheelDiameterFeet = 4.0 / 12.0;
    double wheelRadiusFeet = wheelDiameterFeet / 2.0;
    double feetPerRotation = (Math.PI * 2.0 * wheelRadiusFeet);
    // 16 / 24 gear ratio
    double rotationsPerSecond = (speed / feetPerRotation) * (24.0 / 16.0);
    // 2048 ticks per rotation
    double ticksPerSecond = rotationsPerSecond * 2048.0;
    double ticksPerHundredMiliseconds = ticksPerSecond / 10.0;
    SmartDashboard.putNumber("Ticks per 100ms", ticksPerHundredMiliseconds);
    leftMotor.set(ControlMode.Velocity, ticksPerHundredMiliseconds);
  }

  public boolean isShooterToSpeed(){
    //TODO: Find out what this deadband range is
    if (Utilities.deadband((getTopMotorSpeed() - shooter.getDouble(0)), speedTolerance) == 0){
      return true;
    }
    else{
      return false;
    }
  }

  public boolean isMotorOverheated(){
    boolean motorOverTemp = false;
    if (getBottomMotorOverTemp() | getTopMotorOverTemp()) {
      motorOverTemp = true;
    } else {
      motorOverTemp = false;
    }
    return motorOverTemp;
  }

  public void configureMotorStart(){
    currentLimitConfigurationMotor.currentLimit = 50;
    leftMotor.configStatorCurrentLimit(currentLimitConfigurationMotor, 0);
  }

  public void configureMotorStop(){
    currentLimitConfigurationMotor.currentLimit = 0;
    leftMotor.configStatorCurrentLimit(currentLimitConfigurationMotor, 0);
  }
}
