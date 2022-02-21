package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.AnalogInput;
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

  private final static double kP = 0.15;
  private final static double kI = 0.0;
  private final static double kD = 0.0;
  private final static double tolerance = 100;

  public final double START = 0;
  public final double LOW_RUNG = 100000;
  public final double MID_RUNG = 277350;
  public final double RICKABOOT = 120000;

  private double maxSpeed = 0.25;
  private double nominalForwardSpeed = 0.1;
  private double nominalReverseSpeed = -nominalForwardSpeed;

  private double minEncoderValue = 500;
  private double maxEncoderValue = 277350;
  private double minStringpotValue = 0.5;
  private double maxStringpotValue = 2.95;
  private double kConversion = (2.95 - 0.5) / 277350;  //value to convert stringpot to encoder value

  public boolean climberActivated = false;
  private boolean debugClimber = true;

  
  public Climber() {

    //Setup config file
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.forwardSoftLimitThreshold = maxEncoderValue;
    motorConfig.forwardSoftLimitEnable = true;
    motorConfig.reverseSoftLimitThreshold = minEncoderValue;
    motorConfig.reverseSoftLimitEnable = true;

    motorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    //motorConfig.primaryPID.selectedFeedbackCoefficient = 1; //TODO use to convert to inches?
    motorConfig.peakOutputForward = maxSpeed;
    motorConfig.peakOutputReverse = -maxSpeed;
    motorConfig.nominalOutputForward = nominalForwardSpeed;
    motorConfig.nominalOutputReverse = nominalReverseSpeed;
    motorConfig.slot0.kP = kP;
    motorConfig.slot0.kI = kI;
    motorConfig.slot0.kD = kD;
    motorConfig.slot0.allowableClosedloopError = tolerance;

    //Setup motors
    leftMotor.configFactoryDefault();
    leftMotor.getAllConfigs(motorConfig);
    leftMotor.setInverted(true);
    leftMotor.setNeutralMode(NeutralMode.Brake);
    leftMotor.setSelectedSensorPosition(0); //TODO set based on stringpot if valid reading?
    //leftMotor.setSelectedSensorPosition((stringPot.getVoltage()*kConversion); // set encoder based on stringpot

    rightMotor.configFactoryDefault();
    rightMotor.follow(leftMotor);
    //leftMotor.setInverted(false); add for safety??
    rightMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setInverted(InvertType.OpposeMaster);
  }

  
  @Override
  public void periodic() {
    if(debugClimber){
    SmartDashboard.putNumber("stringpot", getStringPotVoltage());
    SmartDashboard.putNumber("encoder", getLeftMotorPosition());
    }
  }
  /**
   * sets the climber boolean as active after the <start> button is pressed th first time.
   */
  public void activateClimber(){
    climberActivated = true;
  }  
  /**
   * returns the climberActivated boolean status.
   */
  public boolean getClimberStatus(){
    return climberActivated;
  }
  /**
   * returns ture if the string pot is within acceptable values.
   * If the stringpot is below this value, it maybe unplugged, untied/unhooked from the climber, or worse.
   */
  public boolean getStringPotHealth(){
    return ((getStringPotVoltage() > 0.1) && (getStringPotVoltage() < 3.0));
  }

   /**
   * sets the climber position to move to using the encoder in the TalonFX
   */
  public void setPositionUsingEncoder(double setpoint){
    leftMotor.set(TalonFXControlMode.Position, setpoint);
  }  
     /**
   * Holds the climber in the current position using the encoder in the TalonFX
   */
  public void holdPositionUsingEncoder(){
    double position = leftMotor.getSelectedSensorPosition();
    setPositionUsingEncoder(position);
  }
  /**
   * Starts the climber by joystick
   */
  public void setSpeed(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed);
  }
 /**
  * Stops the climber
  */
  public void stop() {
    leftMotor.set(ControlMode.PercentOutput, 0);
  }
  /**
   * @return the String Pot Voltage 
   */
  public double getStringPotVoltage(){
    return stringPot.getVoltage();
  }
 /**
  * Gets the left motor encoder reading
  */ 
  public double getLeftMotorPosition(){
    return leftMotor.getSelectedSensorPosition();
  }
 /**
  * Gets the right motor encoder reading
  */ 
  public double getRightMotorPosition(){
    return rightMotor.getSelectedSensorPosition();
  }
   /**
  * Turn on/off soft limits on climber in extreme cases
  */ 
  public void enableClimberSoftlimits(){
    //TODO read some switch on driverstation to say true or false.
    leftMotor.overrideSoftLimitsEnable(false);
  }
 /**
  * Turn on/off soft limits on climber in extreme cases
  */ 
  public void enableDebug(boolean state){
    this.debugClimber = state;
  }
 /**
  * Toggle on/off soft limits on climber in extreme cases
  */ 
  public void toggleDebug(){
    this.debugClimber = !this.debugClimber;
  }
  /**
   * @return Gets the climber speed as a percent (between -1 and 1)
   */
  private double getClimberSpeed() {
    return (leftMotor.getMotorOutputPercent());
  }
  /**
   * @return the temperature of the left climber motor (in Celsius)
   */
  private double getLeftClimberTemperature() {
    return leftMotor.getTemperature();
  }
  /**
   * @return the temperature of the right climber motor (in Celsius)
   */
  private double getRightClimberTemperature(){
    return rightMotor.getTemperature();
  }
}