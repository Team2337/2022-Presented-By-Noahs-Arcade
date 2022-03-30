package frc.robot.subsystems;


import java.util.Map;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SystemsCheckPositions;
import frc.robot.nerdyfiles.utilities.CTREUtils;

/**
 * Subsystem for the climber mechanism
 */
public class Climber extends SubsystemBase {
    
  //Servos
  public final Servo leftHookServo = new Servo(Constants.LEFT_SERVO_ID);
  public final Servo rightHookServo = new Servo(Constants.RIGHT_SERVO_ID);
  
  private final AnalogInput stringPot = new AnalogInput(Constants.CLIMBER_STRING_POT_ID);
  private final TalonFX leftMotor = new TalonFX(
    Constants.CLIMBER_LEFT_MOTOR_ID,//,
    Constants.UPPER_CANIVORE_ID
  );
  private final TalonFX rightMotor = new TalonFX(
    Constants.CLIMBER_RIGHT_MOTOR_ID,//,
    Constants.UPPER_CANIVORE_ID
  );
  
  private static double servoSpeed = 1;

  private final static double kP = 0.08;
  private final static double kI = 0.0;
  private final static double kD = 0.0;
  private final static double tolerance = 50;

  public final double START = 0.59;
  public final double TRAVEL_LOCATION = 1.0;
  public final double LOW_RUNG = 100000;
  public final double MID_RUNG = 2.5;
  public final double RICKABOOT = 1.4; // 1.65 Stringpot

  private static final double MAX_UP_SPEED = 1.0;
  private static final double MAX_DOWN_SPEED = 0.7;

  private double MIN_STRINGPOT_VALUE = 0.35;
  private double MAX_STRINGPOT_VALUE = 3.05;

  private double nominalForwardSpeed = 0.1;
  private double nominalReverseSpeed = -nominalForwardSpeed;

  public boolean climberActivated = false;
  public Climber() {
    //Setup config file
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.forwardSoftLimitThreshold = stringpotToEncoder(MAX_STRINGPOT_VALUE);
    motorConfig.forwardSoftLimitEnable = true;
    motorConfig.reverseSoftLimitThreshold = stringpotToEncoder(MIN_STRINGPOT_VALUE);
    motorConfig.reverseSoftLimitEnable = true;
    motorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    motorConfig.peakOutputForward = MAX_UP_SPEED;
    motorConfig.peakOutputReverse = -MAX_DOWN_SPEED;
    motorConfig.nominalOutputForward = nominalForwardSpeed;
    motorConfig.nominalOutputReverse = nominalReverseSpeed;
    motorConfig.slot0.kP = kP;
    motorConfig.slot0.kI = kI;
    motorConfig.slot0.kD = kD;
    motorConfig.slot0.allowableClosedloopError = tolerance;

    leftHookServo.setDisabled();
    rightHookServo.setDisabled();
    
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();
    leftMotor.configAllSettings(motorConfig);

    leftMotor.setNeutralMode(NeutralMode.Brake);

    rightMotor.follow(leftMotor);

    leftMotor.setInverted(TalonFXInvertType.CounterClockwise);
    rightMotor.setInverted(TalonFXInvertType.OpposeMaster);

    leftMotor.configStatorCurrentLimit(CTREUtils.defaultCurrentLimit(), 0);

    // This formula is used for converting Stringpot to encoder movements, so we only need one PID.
    double voltageRound = Double.parseDouble(String.format("%.2f",getStringPotVoltage()));
    int setpoint = (int)stringpotToEncoder(voltageRound);
    leftMotor.setSelectedSensorPosition(setpoint);

    setupShuffleboard(Constants.DashboardLogging.CLIMBER);
  }

  private void setupShuffleboard(Boolean logEnable) {
    if (logEnable) {
      ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
      ShuffleboardLayout climberWidget = climberTab.getLayout("climber Info", BuiltInLayouts.kList).withSize(3, 2)
          .withPosition(4, 0);
      climberWidget.addNumber("Speed", this::getSpeed);
      climberWidget.addNumber("Left Temp", this::getLeftMotorTemperature);
      climberWidget.addNumber("Right Temp", this::getRightMotorTemperature);
      climberWidget.addNumber("String Pot", this::getStringPotVoltage);
      climberWidget.addNumber("Encoder Position", this::getEncoderPosition);
      double voltageRound = Double.parseDouble(String.format("%.2f",getStringPotVoltage()));
      double setpoint = (112676 * voltageRound - 39538);
      SmartDashboard.putNumber("Voltage Round", voltageRound);
      SmartDashboard.putNumber("Voltage to Ticks", setpoint);
    }
    SmartDashboard.putNumber("String pot ", getStringPotVoltage());

    // Systems check
    if (Constants.DO_SYSTEMS_CHECK) {
      ShuffleboardTab systemsCheck = Constants.SYSTEMS_CHECK_TAB;
      
      systemsCheck.addBoolean("String Pot", () -> (stringPot.getVoltage() > 0))
        .withPosition(SystemsCheckPositions.STRING_POT.x, SystemsCheckPositions.STRING_POT.y)
        .withSize(3, 3);
      systemsCheck.addNumber("L Climber Temp (C)", () -> leftMotor.getTemperature())
        .withPosition(SystemsCheckPositions.L_CLIMBER_TEMP.x, SystemsCheckPositions.L_CLIMBER_TEMP.y)
        .withSize(3, 4)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Min", Constants.MOTOR_MINIMUM_TEMP_CELSIUS, "Max", Constants.MOTOR_SHUTDOWN_TEMP_CELSIUS));
      systemsCheck.addNumber("R Climber Temp (C)", () -> rightMotor.getTemperature())
        .withPosition(SystemsCheckPositions.R_CLIMBER_TEMP.x, SystemsCheckPositions.R_CLIMBER_TEMP.y)
        .withSize(3, 4)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Min", Constants.MOTOR_MINIMUM_TEMP_CELSIUS, "Max", Constants.MOTOR_SHUTDOWN_TEMP_CELSIUS));
    }
  }

  @Override
  public void periodic() {
  }

  //True if Connected, 0.26 is around the value we get when we do not have anything plugged into the port.
  public boolean getStringPotHealth() {
    return ((getStringPotVoltage() > 0.26) && (getStringPotVoltage() < 4.3));
  }
  //Equation found using Google Sheets
  public double stringpotToEncoder(double stringpot){
    return ((Constants.getInstance().CLIMBER_SLOPE * stringpot) - Constants.getInstance().CLIMBER_Y_INTERCEPT);
  }

  public void setPosition(double setpoint) {
    //If setpoint is within the range for a stringpot, 0-5V, convert to ticks, we will save value as Stringpot, so doesn't matter if we have it or not.
    if (setpoint < 5){
      setpoint = stringpotToEncoder(setpoint); //Converts to encoder ticks
    }
    //Else, we just use the ticks anyway and set
    leftMotor.set(TalonFXControlMode.Position, setpoint);
  }

  public void hold() {
    double position = getEncoderPosition();
    setPosition(position);
  }
  
  public double getEncoderPosition() {
    return leftMotor.getSelectedSensorPosition();
  }

  public void releaseServos(){
    leftHookServo.setSpeed(-servoSpeed);
    rightHookServo.setSpeed(servoSpeed);
  }

  public void stop() {
    setSpeed(0.0);
  }

  public void setSpeed(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed);
  }

  private double getSpeed() {
    return leftMotor.getMotorOutputPercent();
  }

  public double getStringPotVoltage() {
    return stringPot.getVoltage();
  }

  public boolean isStringPotConnected() {
    return getStringPotVoltage() > 0;
  }

  /**
   * Temp in Celcius
   */
  private double getLeftMotorTemperature() {
    return leftMotor.getTemperature();
  }

  /**
   * Temp in Celcius
   */
  private double getRightMotorTemperature() {
    return rightMotor.getTemperature();
  }

  public void disableBrakeMode() {
    leftMotor.setNeutralMode(NeutralMode.Coast);
    rightMotor.setNeutralMode(NeutralMode.Coast);
  }

}

