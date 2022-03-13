package frc.robot.subsystems;

import java.lang.invoke.VolatileCallSite;
import java.util.function.Supplier;

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
  private final TalonFX leftMotor = new TalonFX(
    Constants.CLIMBER_LEFT_MOTOR_ID//,
    //Constants.UPPER_CANIVORE_ID
  );
  private final TalonFX rightMotor = new TalonFX(
    Constants.CLIMBER_RIGHT_MOTOR_ID//,
   // Constants.UPPER_CANIVORE_ID
  );

  private final static double kP = 0.15;
  private final static double kI = 0.0;
  private final static double kD = 0.0;
  private final static double tolerance = 50;

  public final double START = 0;
  public final double LOW_RUNG = 100000;
  public final double MID_RUNG = 2.5;
  public final double RICKABOOT = 1.65; // 1.65 Stringpot
  public final double THIRD_RUNG_MAX = 2;
  public final double THIRD_RUNG_MIN = 1.5;
  public final double HOOKS_ARE_SET = 1;

  public final double PITCH_RANGE = 0;

  private double maxSpeed = 1;
  private double nominalForwardSpeed = 0.1;
  private double nominalReverseSpeed = -nominalForwardSpeed;

  private double minEncoderValue = 30606;
  private double maxEncoderValue = 277350;
    //value to convert stringpot to encoder value

  public boolean climberActivated = false;
  public Climber() {
    //Setup config file
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.forwardSoftLimitThreshold = maxEncoderValue;
    motorConfig.forwardSoftLimitEnable = true;
    motorConfig.reverseSoftLimitThreshold = minEncoderValue;
    motorConfig.reverseSoftLimitEnable = true;
    //motorConfig.reverseSoftLimitEnable = false;
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
    // This formula is used for converting Stringpot to encoder movements, so we only need one PID.
    double voltageRound = Double.parseDouble(String.format("%.2f",getStringPotVoltage()));
    double setpoint = stringpotToEncoder(voltageRound);
    leftMotor.setSelectedSensorPosition(setpoint);
    // Motor turns 25 times for one climber rotation, which is 6.283 inches, 2048
    // ticks in a rotation. Overall loss with this tolerance: 0.019 inches
    /*leftMotor.configAllowableClosedloopError(0, 100);
    leftMotor.configNominalOutputForward(0.1);
    leftMotor.configNominalOutputReverse(0.1); */

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
      climberWidget.addBoolean("Status", this::getClimberStatus);
      double voltageRound = Double.parseDouble(String.format("%.2f",getStringPotVoltage()));
      double setpoint = (112676 * voltageRound - 39538);
      SmartDashboard.putNumber("Voltage Round", voltageRound);
      SmartDashboard.putNumber("Voltage to Ticks", setpoint);
    }
  }

  @Override
  public void periodic() {
  }

  public void activateClimber() {
    climberActivated = true;
  }

  public boolean getClimberStatus() {
    return climberActivated;
  }
  //True if Connected
  public boolean getStringPotHealth() {
    return ((getStringPotVoltage() > 0.1) && (getStringPotVoltage() < 4.3));
  }
  //Equation found using Google Sheets
  public double stringpotToEncoder(double stringpot){
    return ((112676 * stringpot) - 39538);
  }
  public boolean climberWithinThirdRungRange(){
    return (THIRD_RUNG_MIN < getStringPotVoltage() && getStringPotVoltage() < THIRD_RUNG_MAX);
  }

  public void setPosition(double setpoint) {
    //If setpoint is within the range for a stringpot, 0-5V, convert to ticks, we will save value as Stringpot, so doesn't matter if we have it or not.
    if (setpoint < 5){
      setpoint = stringpotToEncoder(setpoint); //Convert to ENCODER TICKS!!!
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
