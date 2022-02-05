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
  public final AnalogInput stringPot;
  private final TalonFX motor1;
  private final TalonFX motor2;
  public TalonFXConfiguration fxConfig;
  public StatorCurrentLimitConfiguration currentLimitConfigurationMotor = new StatorCurrentLimitConfiguration();

  
  public Climber() {
    // Initialize motor
    stringPot = new AnalogInput(3);
    motor1 = new TalonFX(3);
    motor2 = new TalonFX(35);
    
    // TODO: make sure config settings are correct
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

    motor1.setNeutralMode(NeutralMode.Coast);

    motor1.configOpenloopRamp(0.5);
    motor1.enableVoltageCompensation(true);
    motor1.setInverted(true);
    motor2.setInverted(InvertType.OpposeMaster);

    // Set up shuffleboard stuff
    ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
    
    ShuffleboardLayout climberWidget = climberTab.getLayout("climber Info", BuiltInLayouts.kList).withSize(3,2).withPosition(4, 0);
    climberWidget.addNumber("Speed", this::getClimberSpeed);
    climberWidget.addNumber("Temp", this::getClimberTemperature);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("String Pot", stringPot.getVoltage());
  }

  /**
   * Starts the climber
   */
  public void startClimber(double speed) {
    motor1.set(ControlMode.PercentOutput, speed);
  }
  
  /**
   * Stops the climber
   */
  public void stopClimber() {
    double position = motor1.getSelectedSensorPosition();
    double position2 = motor2.getSelectedSensorPosition();
    motor1.set(ControlMode.Position, position);
    motor2.set(ControlMode.Position, position2);
  }

  /**
   * @return Gets the climber speed as a percent (between -1 and 1)
   */
  private double getClimberSpeed() {
    return ((motor1.getMotorOutputPercent() + motor2.getMotorOutputPercent())/2);
  }

  /**
   * Returns the temperature of the climber motor (in Celsius)
   */
  private double getClimberTemperature() {
    return ((motor1.getTemperature() + motor2.getTemperature())/2);
  }
}