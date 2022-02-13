package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem for the climber mechanism
 * 
 * @author  Nicholas S
 */
public class Climber extends SubsystemBase {
  private final AnalogInput stringPot = new AnalogInput(Constants.CLIMBER_STRING_POT_ID);
  private final TalonFX leftMotor = new TalonFX(Constants.LEFT_CLIMBER_MOTOR_ID);
  private final TalonFX rightMotor = new TalonFX(Constants.RIGHT_CLIMBER_MOTOR_ID);
  
  public Climber() {
    StatorCurrentLimitConfiguration currentLimitConfigurationMotor = new StatorCurrentLimitConfiguration();
    //Set settings on motor
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();
    
    rightMotor.follow(leftMotor);

    currentLimitConfigurationMotor.currentLimit = 50; //50
    currentLimitConfigurationMotor.enable = true;
    currentLimitConfigurationMotor.triggerThresholdCurrent = 40;
    currentLimitConfigurationMotor.triggerThresholdTime = 3;
    // Implements these current limits on the motors
    leftMotor.configStatorCurrentLimit(currentLimitConfigurationMotor, 0);

    leftMotor.setNeutralMode(NeutralMode.Brake);
    leftMotor.config_kP(0, .15);
    leftMotor.config_kI(0, 0);
    leftMotor.config_kD(0, 0);
    leftMotor.configAllowableClosedloopError(0, 100);
    leftMotor.configNominalOutputForward(.1);
    leftMotor.configNominalOutputReverse(.1);

    leftMotor.configOpenloopRamp(0.5);
    leftMotor.enableVoltageCompensation(true);
    leftMotor.setInverted(true);
    rightMotor.setInverted(InvertType.OpposeMaster);

    // Set up shuffleboard stuff
    ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
    
    ShuffleboardLayout climberWidget = climberTab.getLayout("climber Info", BuiltInLayouts.kList).withSize(3,2).withPosition(4, 0);
    climberWidget.addNumber("Speed", this::getClimberSpeed);
    climberWidget.addNumber("Left Temp", this::getLeftClimberTemperature);
    climberWidget.addNumber("Right Temp", this::getRightClimberTemperature);
    climberWidget.addNumber("String Pot", this::getStringPotVoltage);
    climberWidget.addNumber("Left Motor Position", this::getMotorOnePosition);
    climberWidget.addNumber("Right Motor Position", this::getMotorTwoPosition);
  }

  @Override
  public void periodic() {
  }

  /**
   * Starts the climber
   */
  public void start(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed);
  }
  /**
   * Holds the climber at a set position
   */
  public void hold(double setpoint){
    leftMotor.set(TalonFXControlMode.Position, setpoint);
  }
  
  /**
   * Stops the climber
   */
  public void stop() {
    leftMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * @return Gets the climber speed as a percent (between -1 and 1)
   */
  private double getClimberSpeed() {
    return (leftMotor.getMotorOutputPercent());
  }

  public double getMotorOnePosition(){
    return leftMotor.getSelectedSensorPosition();
  }

  public double getMotorTwoPosition(){
    return rightMotor.getSelectedSensorPosition();
  }

  /**
   * Gets the String Pot Voltage 
   */
  public double getStringPotVoltage(){
    return stringPot.getVoltage();
  }

  /**
   * Returns the temperature of the left climber motor (in Celsius)
   */
  private double getLeftClimberTemperature() {
    return leftMotor.getTemperature();
  }
  /**
   * Returns the temperature of the right climber motor (in Celsius)
   */
  private double getRightClimberTemperature(){
    return rightMotor.getTemperature();
  }
}