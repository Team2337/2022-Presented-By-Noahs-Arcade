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
  private final AnalogInput stringPot;
  private final TalonFX motor1;
  private final TalonFX motor2;
  public TalonFXConfiguration fxConfig;
  public StatorCurrentLimitConfiguration currentLimitConfigurationMotor = new StatorCurrentLimitConfiguration();

  
  public Climber() {
    // Initialize motor
    stringPot = new AnalogInput(Constants.STRING_POT_ID);
    motor1 = new TalonFX(Constants.CLIMBER1_MOTOR_ID);
    motor2 = new TalonFX(Constants.CLIMBER2_MOTOR_ID);
    
    
    //Set settings on motor
    motor1.configFactoryDefault();
    motor2.configFactoryDefault();
    
    motor2.follow(motor1);

    currentLimitConfigurationMotor.currentLimit = 50; //50
    currentLimitConfigurationMotor.enable = true;
    currentLimitConfigurationMotor.triggerThresholdCurrent = 40;
    currentLimitConfigurationMotor.triggerThresholdTime = 3;
        // Implements these current limits on the motors
    motor1.configStatorCurrentLimit(currentLimitConfigurationMotor, 0);

    motor1.setNeutralMode(NeutralMode.Brake);

    motor1.configOpenloopRamp(0.5);
    motor1.enableVoltageCompensation(true);
    motor1.setInverted(true);
    motor2.setInverted(InvertType.OpposeMaster);

    // Set up shuffleboard stuff
    ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
    
    ShuffleboardLayout climberWidget = climberTab.getLayout("climber Info", BuiltInLayouts.kList).withSize(3,2).withPosition(4, 0);
    climberWidget.addNumber("Speed", this::getClimberSpeed);
    climberWidget.addNumber("Temp", this::getClimberTemperature);
    climberWidget.addNumber("String Pot", this::getStringPotVoltage);
  }

  @Override
  public void periodic() {
  }

  /**
   * Starts the climber
   */
  public void start(double speed) {
    motor1.set(ControlMode.PercentOutput, speed);
  }
  /**
   * Holds the climber at a set position
   */
  public void hold(double setpoint){
    motor1.set(ControlMode.Position, setpoint);
}
  
  /**
   * Stops the climber
   */
  public void stop() {
    motor1.set(ControlMode.PercentOutput, 0);
  }

  /**
   * @return Gets the climber speed as a percent (between -1 and 1)
   */
  private double getClimberSpeed() {
    return ((motor1.getMotorOutputPercent() + motor2.getMotorOutputPercent())/2);
  }

  public double getMotorOnePosition(){
    return motor1.getSelectedSensorPosition();
  }

  public double getStringPotVoltage(){
    return stringPot.getVoltage();
  }



  /**
   * Returns the temperature of the climber motor (in Celsius)
   */
  private double getClimberTemperature() {
    return ((motor1.getTemperature() + motor2.getTemperature())/2);
  }
}