package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

  private static final double STRING_POT_UPPER_SOFT_LIMIT = 3.0;
  private Double stringPotStartingVoltage;
  private double stringPotVoltage;

  private static final double ENCODER_UPPER_SOFT_LIMIT = 3000;
  private static final double ENCODER_LOWER_SOFT_LIMIT = 0; // Zero == where the motor boots
  private double motorPosition;

  public Climber() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    rightMotor.follow(leftMotor);

    leftMotor.setInverted(TalonFXInvertType.Clockwise);
    rightMotor.setInverted(TalonFXInvertType.OpposeMaster);

    leftMotor.configStatorCurrentLimit(CTREUtils.defaultCurrentLimit(), 0);
    // TODO: If set set a nominal voltage we can enable voltage compensation
    // leftMotor.enableVoltageCompensation(true);

    leftMotor.setNeutralMode(NeutralMode.Brake);

    // PID for our positional hold
    leftMotor.config_kP(0, 0.15);
    leftMotor.config_kI(0, 0);
    leftMotor.config_kD(0, 0);
    // Motor turns 16 times for one climber rotation, which is 6.283 inches, 2048
    // ticks in a rotation. Overall loss with this tolerance: 0.019 inches
    leftMotor.configAllowableClosedloopError(0, 100);
    leftMotor.configNominalOutputForward(0.1);
    leftMotor.configNominalOutputReverse(0.1);

    // TODO: Internal encoder needs to boot to zero

    setupShuffleboard(Constants.DashboardLogging.CLIMBER);
  }

  private void setupShuffleboard(Boolean logEnable) {
    if (logEnable) {
      ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
      ShuffleboardLayout climberWidget = climberTab.getLayout("climber Info", BuiltInLayouts.kList).withSize(3, 2)
          .withPosition(4, 0);
      climberWidget.addNumber("Speed", this::getSpeed);
      climberWidget.addNumber("Left Temp", this::getLeftMotorTemperatureCelsius);
      climberWidget.addNumber("Right Temp", this::getRightMotorTemperatureCelsius);
      climberWidget.addNumber("String Pot", this::getStringPotVoltage);
      climberWidget.addNumber("Position", this::getPosition);
    }
  }

  @Override
  public void periodic() {
    stringPotVoltage = stringPot.getVoltage();
    motorPosition = leftMotor.getSelectedSensorPosition();

    // TODO: Timeout on setting our starting voltage...
    if (isStringPotConnected() && stringPotStartingVoltage == null) {
      stringPotStartingVoltage = stringPotVoltage;
    }
  }

  /** Public API */

  private boolean isStringPotConnected() {
    return getStringPotVoltage() > 0;
  }

  public void holdAtCurrentPosition() {
    leftMotor.set(ControlMode.Position, leftMotor.getSelectedSensorPosition());
  }

  // TODO: This should only be used pubicly when overriding
  public void setSpeed(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stop() {
    setSpeed(0.0);
  }

  /**
   * Get the position of the climber from 0 -> 1.
   * If the string pot is working - we will scale the string pot values.
   * If the string pot ISN'T working - we will scale the internal encoder values
   */
  public double getPosition() {
    if (isStringPotConnected() && areStringPotLimitsSetup()) {
      final double lowerLimit = getStringPotLowerLimit();
      final double upperLimit = getStringPotUpperLimit();
      // Scale between our upper limit and lower limit
      double range = upperLimit - lowerLimit;
      return getStringPotVoltage() / range;
    } else {
      // If our string pot isn't connected or we don't have our string pot
      // limts setup properly, get our climber position via encoders.
      final double lowerLimit = ENCODER_LOWER_SOFT_LIMIT;
      final double upperLimit = ENCODER_UPPER_SOFT_LIMIT;
      double range = upperLimit - lowerLimit;
      return getMotorPosition() / range;
    }
  }

  /** String Pot */

  private double getStringPotVoltage() {
    return stringPotVoltage;
  }

  private boolean areStringPotLimitsSetup() {
    return getStringPotLowerLimit() != null && getStringPotUpperLimit() != null;
  }

  private Double getStringPotLowerLimit() {
    // If our string pot is not connected - we have no lower limit
    if (!isStringPotConnected()) {
      return null;
    }
    // stringPotStartingVoltage may still be null - so either null or value
    return stringPotStartingVoltage;
  }

  private Double getStringPotUpperLimit() {
    // If our starting voltage is above our upper limit, our upper
    // limit is going to be our starting voltage.
    // In the case that the upper and lower limit are the starting
    // voltages, the climber should not move.
    if (!isStringPotConnected()) {
      return null;
    }
    if (stringPotStartingVoltage != null && stringPotStartingVoltage > STRING_POT_UPPER_SOFT_LIMIT) {
      return stringPotStartingVoltage;
    }
    return STRING_POT_UPPER_SOFT_LIMIT;
  }

  /** Motor Encoders */

  private double getMotorPosition() {
    return leftMotor.getSelectedSensorPosition();
  }

  /** System Shuffleboard */

  private double getSpeed() {
    return leftMotor.getMotorOutputPercent();
  }

  private double getLeftMotorTemperatureCelsius() {
    return leftMotor.getTemperature();
  }

  private double getRightMotorTemperatureCelsius() {
    return rightMotor.getTemperature();
  }

}
