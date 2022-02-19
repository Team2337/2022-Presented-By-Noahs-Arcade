package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.nerdyfiles.utilities.CTREUtils;
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
  
  private StatorCurrentLimitConfiguration currentLimitConfiguration = CTREUtils.defaultCurrentLimit();

  public Climber() {
    //Set settings on motor
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();
    
    rightMotor.follow(leftMotor);

    leftMotor.setInverted(TalonFXInvertType.Clockwise); //True
    rightMotor.setInverted(InvertType.OpposeMaster);

    // Implements these current limits on the motors
    leftMotor.configStatorCurrentLimit(currentLimitConfiguration, 0);
    leftMotor.configClosedloopRamp(0.1);
    leftMotor.enableVoltageCompensation(true);

    leftMotor.setNeutralMode(NeutralMode.Brake);
    leftMotor.config_kP(0, .15);
    leftMotor.config_kI(0, 0);
    leftMotor.config_kD(0, 0);
    //Motor turns 16 times for one climber rotation, which is 6.283 inches, 2048 ticks in a rotation. Overall loss with this tolerance: 0.019 inches
    leftMotor.configAllowableClosedloopError(0, 100);
    leftMotor.configNominalOutputForward(.1);
    leftMotor.configNominalOutputReverse(.1);

    //setUpShuffleboard();
  }

  @Override
  public void periodic() {
  }

  /**
   * Starts the climber
   */
  public void setSpeed(double speed) {
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
  private double getLeftMotorSpeed() {
    return (leftMotor.getMotorOutputPercent());
  }

  public double getLeftMotorPosition(){
    return leftMotor.getSelectedSensorPosition();
  }

  public double getRightMotorPosition(){
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
  private double getLeftMotorTemperature() {
    return leftMotor.getTemperature();
  }
  /**
   * Returns the temperature of the right climber motor (in Celsius)
   */
  private double getRightMotorTemperature(){
    return rightMotor.getTemperature();
  }

  private void setupShuffleboard(){
    ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
    ShuffleboardLayout climberWidget = climberTab.getLayout("climber Info", BuiltInLayouts.kList).withSize(3,2).withPosition(4, 0);
    climberWidget.addNumber("Speed", this::getLeftMotorSpeed);
    climberWidget.addNumber("Left Temp", this::getLeftMotorTemperature);
    climberWidget.addNumber("Right Temp", this::getRightMotorTemperature);
    climberWidget.addNumber("String Pot", this::getStringPotVoltage);
    climberWidget.addNumber("Left Motor Position", this::getLeftMotorPosition);
    climberWidget.addNumber("Right Motor Position", this::getRightMotorPosition);
  } 
}