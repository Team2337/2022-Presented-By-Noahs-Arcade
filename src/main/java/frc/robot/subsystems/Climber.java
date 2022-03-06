package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
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
import frc.robot.nerdyfiles.utilities.CTREUtils;

/**
 * Subsystem for the climber mechanism
 */
public class Climber extends SubsystemBase {

  private final AnalogInput stringPot = new AnalogInput(Constants.CLIMBER_STRING_POT_ID);
  private final TalonFX leftMotor = new TalonFX(Constants.CLIMBER_LEFT_MOTOR_ID);
  private final TalonFX rightMotor = new TalonFX(Constants.CLIMBER_RIGHT_MOTOR_ID);

  private final static double kP = 0.15;
  private final static double kI = 0.0;
  private final static double kD = 0.0;
  private final static double tolerance = 50;

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
  private double kConversion = 277350/(2.95 - 0.5) ;  //value to convert stringpot to encoder value

  public boolean climberActivated = false;
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

    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();
    leftMotor.configAllSettings(motorConfig);

    leftMotor.setNeutralMode(NeutralMode.Brake);

    rightMotor.follow(leftMotor);

    leftMotor.setInverted(TalonFXInvertType.Clockwise);
    rightMotor.setInverted(TalonFXInvertType.OpposeMaster);

    leftMotor.configStatorCurrentLimit(CTREUtils.defaultCurrentLimit(), 0);
    // TODO: If set set a nominal voltage we can enable voltage compensation
    // leftMotor.enableVoltageCompensation(true);

    // PID for our positional hold
    /*leftMotor.config_kP(0, 0.15);
    leftMotor.config_kI(0, 0);
    leftMotor.config_kD(0, 0); */
    leftMotor.setSelectedSensorPosition(0);

    //leftMotor.setSelectedSensorPosition(getStringPotVoltage()*kConversion);
    // Motor turns 16 times for one climber rotation, which is 6.283 inches, 2048
    // ticks in a rotation. Overall loss with this tolerance: 0.019 inches
    /*leftMotor.configAllowableClosedloopError(0, 100);
    leftMotor.configNominalOutputForward(0.1);
    leftMotor.configNominalOutputReverse(0.1); */

    setupShuffleboard(Constants.DashboardLogging.CLIMBER);
  }

  private void setupShuffleboard(Boolean logEnable) {
    Double hahafunny = getStringPotVoltage()*kConversion;
    if (logEnable) {
      ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
      ShuffleboardLayout climberWidget = climberTab.getLayout("climber Info", BuiltInLayouts.kList).withSize(3, 2)
          .withPosition(4, 0);
      climberWidget.addNumber("Speed", this::getSpeed);
      climberWidget.addNumber("Left Temp", this::getLeftMotorTemperature);
      climberWidget.addNumber("Right Temp", this::getRightMotorTemperature);
      climberWidget.addNumber("String Pot", this::getStringPotVoltage);
      climberWidget.addNumber("Encoder Position", this::getPosition);
      climberWidget.addBoolean("Status", this::getClimberStatus);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Voltage to Ticks", getStringPotVoltage()*kConversion);
  }

  public void activateClimber() {
    climberActivated = true;
  }

  public boolean getClimberStatus() {
    return climberActivated;
  }

  public boolean getStringPotHealth() {
    return ((getStringPotVoltage() > 0.1) && (getStringPotVoltage() < 3.0));
  }

  public void setPositionUsingEncoder(double setpoint) {
    leftMotor.set(TalonFXControlMode.Position, setpoint);
  }

  public void hold() {
    double position = leftMotor.getSelectedSensorPosition();
    setPositionUsingEncoder(position);
  }
  
  public double getPosition() {
    return leftMotor.getSelectedSensorPosition();
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



}
