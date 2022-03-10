package frc.robot.subsystems;

import java.util.function.Predicate;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberSetpoint;
import frc.robot.nerdyfiles.utilities.CTREUtils;

/**
 * Subsystem for the climber mechanism
 */
public class Climber extends SubsystemBase {

  private static final double MAX_SPEED = 1.0;

  private enum ClimberSensor {
    STRING_POT,
    ENCODER
  }

  // ClimberPosition is an internal class that encapsulates how our
  // climber is reading it's values + the value for the setpoint
  private class ClimberPosition {
    private double value;
    private ClimberSensor sensor;

    ClimberPosition(double value, ClimberSensor sensor) {
      this.value = value;
      this.sensor = sensor;
    }
  };

  private final AnalogInput stringPot = new AnalogInput(Constants.CLIMBER_STRING_POT_ID);
  private final TalonFX leftMotor = new TalonFX(Constants.CLIMBER_LEFT_MOTOR_ID);
  private final TalonFX rightMotor = new TalonFX(Constants.CLIMBER_RIGHT_MOTOR_ID);

  private boolean hasRobotBeenEnabledFirstTime = false;

  // Starting voltage is roughly ~0.33v
  private static final double STRING_POT_UPPER_SOFT_LIMIT = 3.0;
  private static final double STRING_POT_LOWER_SOFT_LIMIT = 0.1;

  private Double stringPotStartingVoltage;
  private PIDController stringPotPIDController = new PIDController(6.0, 0.0, 0.0);

  private static final double ENCODER_UPPER_SOFT_LIMIT = 100000;
  private static final double ENCODER_LOWER_SOFT_LIMIT = 0; // Zero == where the motor boots
  private static final double ENCODER_ALLOWABLE_CLOSED_LOOP_ERROR_TICKS = 100;

  private ClimberSetpoint setpoint;
  private ClimberSetpoint currentSetpoint = ClimberSetpoint.START;

  public Climber() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    // Only need to call configure on the left motor
    configureClimberMotor(leftMotor);
    rightMotor.follow(leftMotor);

    leftMotor.setSelectedSensorPosition(0);

    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);

    leftMotor.setInverted(TalonFXInvertType.Clockwise);
    rightMotor.setInverted(TalonFXInvertType.OpposeMaster);

    stringPotPIDController.setTolerance(0.01);

    setupShuffleboard(Constants.DashboardLogging.CLIMBER);
  }

  private static ErrorCode configureClimberMotor(TalonFX motor) {
    motor.configFactoryDefault();

    TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.initializationStrategy = SensorInitializationStrategy.BootToZero;

    configuration.statorCurrLimit = CTREUtils.defaultCurrentLimit();

    configuration.slot0.kP = 0.15;
    configuration.slot0.kI = 0.0;
    configuration.slot0.kD = 0.1;

    // Motor turns 16 times for one climber rotation, which is 6.283 inches, 2048
    // ticks in a rotation. Overall loss with this tolerance: 0.019 inches
    configuration.slot0.allowableClosedloopError = 100; // ticks

    // Nominal outputs configured to help fight gravity while we're holding a position
    // configuration.nominalOutputForward = 0.1;
    // configuration.nominalOutputReverse = 0.1;

    return motor.configAllSettings(configuration, 250);
  }

  private void setupShuffleboard(Boolean logEnable) {
    if (logEnable) {
      ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
      ShuffleboardLayout climberWidget = climberTab.getLayout("climber Info", BuiltInLayouts.kList).withSize(3, 2)
          .withPosition(4, 0);
      climberWidget.addNumber("Speed", this::getSpeed);
      climberWidget.addNumber("Left Temp", this::getLeftMotorTemperatureCelsius);
      climberWidget.addNumber("Right Temp", this::getRightMotorTemperatureCelsius);
      climberWidget.addString("Current Setpoint", () -> this.currentSetpoint == null ? "N/A" : this.currentSetpoint.toString());
      climberWidget.addNumber("String Pot Voltage (V)", this::getStringPotVoltage);
      climberWidget.addNumber("Left Motor Position (Ticks)", this::getMotorPosition);
      climberWidget.addNumber("Start String Pot Position", () -> stringPotHasStartingVoltage() ? stringPotStartingVoltage : 0.0);
      climberWidget.addNumber("String Pot Controller Error", () -> stringPotPIDController.getPositionError());
      climberWidget.addNumber("Encoder Controller Target", () -> leftMotor.getClosedLoopTarget());
      climberWidget.addNumber("Encoder Controller Error", () -> leftMotor.getClosedLoopError());
    }
  }

  @Override
  public void periodic() {
    // ALWAYS clear our setpoint when we disable. If we re-enable, we don't want the climber
    // moving to the last set position.
    if (DriverStation.isDisabled() && setpoint != null) {
      setpoint = null;
    }

    if (DriverStation.isEnabled() && !hasRobotBeenEnabledFirstTime) {
      hasRobotBeenEnabledFirstTime = true;
    }

    // If we've enabled the robot for the first time and we do not have a starting
    // string pot voltage, let's assume we're never going to get one and fallback
    // to using our encoders.
    if (!hasRobotBeenEnabledFirstTime && isStringPotConnected() && !stringPotHasStartingVoltage()) {
      stringPotStartingVoltage = getStringPotVoltage();
    }

    // Attempt to move our climber to it's new setpoint. If the move to setpoint method
    // does NOT properly handle it's cases - we'll stop the climber.
    boolean shouldStopClimber = periodicMoveToSetpoint();
    if (shouldStopClimber) {
      stop();
    }
  }

  /** Public API */

  /**
   * Get the currently held setpoint for the clibmer. If the climber is
   * moving between setpoints, this will report the previous setpoint.
   * The new setpoint will be reported once the climber reaches that setpoint.
   */
  public ClimberSetpoint getCurrentSetpoint() {
    return this.currentSetpoint;
  }

  public void moveToSetpoint(ClimberSetpoint setpoint) {
    // Note: Rickaboot and start should be the only automatic position moves available
    if (setpoint != ClimberSetpoint.START && setpoint != ClimberSetpoint.RICKABOOT) {
      return;
    }

    if (setpoint != this.setpoint) {
      // The setpoint we're attempting to move to has changed - reset our internal state
      stringPotPIDController.reset();
    }
    this.setpoint = setpoint;
  }

  public void holdAtCurrentPosition() {
    setPosition(getMotorPosition());
  }

  public void stop() {
    setpoint = null;
    setSpeed(0.0);
  }

  /** Internal API */

  private void setCurrentSetpoint(ClimberSetpoint setpoint) {
    this.currentSetpoint = setpoint;
    this.setpoint = null;
  }

  private static boolean withinRange(ClimberPosition position, Predicate<Double> stringPotLimitPredicate, Predicate<Double> encoderLimitPredicate) {
    if (position.sensor == ClimberSensor.STRING_POT) {
      return stringPotLimitPredicate.test(position.value);
    } else if (position.sensor == ClimberSensor.ENCODER) {
      return encoderLimitPredicate.test(position.value);
    }
    return false;
  }

  private boolean belowUpperLimit(ClimberPosition position) {
    return withinRange(
      position,
      value -> value < getStringPotUpperLimit(),
      value -> value < ENCODER_UPPER_SOFT_LIMIT
    );
  }

  private boolean aboveLowerLimt(ClimberPosition position) {
    return withinRange(
      position,
      value -> getStringPotLowerLimit() < value,
      value -> ENCODER_LOWER_SOFT_LIMIT < value
    );
  }

  private boolean periodicMoveToSetpoint() {
    // No setpoint to move to - no problem
    if (setpoint == null) {
      return true;
    }

    // Attempt to move our climber to it's new setpoint
    ClimberPosition currentPosition = getPosition();
    ClimberPosition setpointPosition = positionForSetpoint(setpoint);

    // Unsupported setpoint
    if (setpointPosition == null) {
      return true;
    }

    // Our values need to both be coming from the same sensor in order to move the climber
    if (setpointPosition.sensor != currentPosition.sensor) {
      return true;
    }

    if (setpointPosition.sensor == ClimberSensor.STRING_POT) {
      double output = stringPotPIDController.calculate(
        currentPosition.value,
        setpointPosition.value
      );

      output = Math.copySign(Math.min(Math.abs(output), MAX_SPEED), output);

      SmartDashboard.putNumber("Climber Output", output);

      // If we've arrived at our setpoint - clear out the setpoint field so we don't
      // attempt to hold the climber at the setpoint. The brake mode motors do that for us.
      if (stringPotPIDController.atSetpoint()) {
        // We've arrived! Update our setpoint accordingly.
        setCurrentSetpoint(setpoint);
        return true;
      }

      if (output > 0 && belowUpperLimit(currentPosition)) {
        // Only allow moving upward if we're below our upper limit
        setSpeed(output);
        return false;
      } else if (output < 0 && aboveLowerLimt(currentPosition)) {
        // Only allow moving downward if we're above our lower limit
        setSpeed(output);
        return false;
      }

      // If neither case matches above - we're not safe to move the climber
      return true;
    } else if (setpointPosition.sensor == ClimberSensor.ENCODER) {
      // Check to see if our error is less than our allowable error. If it is - we've arrived.
      if (Math.abs(leftMotor.getClosedLoopError()) < ENCODER_ALLOWABLE_CLOSED_LOOP_ERROR_TICKS) {
        // We've arrived! Update our setpoint accordingly.
        setCurrentSetpoint(setpoint);
        return true;
      }

      if (setpointPosition.value > currentPosition.value && belowUpperLimit(currentPosition)) {
        // Only allow moving upward if we're below our upper limit
        setPosition(setpointPosition.value);
        return false;
      } else if (setpointPosition.value < currentPosition.value && aboveLowerLimt(currentPosition)) {
        // Only allow moving downward if we're above our lower limit
        setPosition(setpointPosition.value);
        return false;
      }

      // If neither case matches above - we're not safe to move the climber
      return true;
    }

    return true; // Fallback to ALWAYS stopping the climber
  }

  private ClimberPosition positionForSetpoint(ClimberSetpoint setpoint) {
    if (shouldUseStringPot()) {
      switch (setpoint) {
        case START:
          return new ClimberPosition(
            stringPotStartingVoltage,
            ClimberSensor.STRING_POT
          );
        case RICKABOOT:
          return new ClimberPosition(
            1.65,
            ClimberSensor.STRING_POT
          );
      }
    } else {
      switch (setpoint) {
        case START:
          return new ClimberPosition(
            0.0,
            ClimberSensor.ENCODER
          );
        case RICKABOOT:
          return new ClimberPosition(
            148500, // TODO: This needs to be fixed...
            ClimberSensor.ENCODER
          );
      }
    }
    return null;
  }

  private ClimberPosition getPosition() {
    if (shouldUseStringPot()) {
      return new ClimberPosition(
        getStringPotVoltage(),
        ClimberSensor.STRING_POT
      );
    } else {
      return new ClimberPosition(
        getMotorPosition(),
        ClimberSensor.ENCODER
      );
    }
  }

  // TODO: Make private - provide some other method here to move via joystick control
  public void setSpeed(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed);
  }

  private void setPosition(double position) {
    if (leftMotor.getClosedLoopTarget() == position) {
      return;
    }
    leftMotor.set(ControlMode.Position, position);
  }

  /** String Pot */

  private boolean shouldUseStringPot() {
    // return false; // For setting the encoder values
    return stringPotHasStartingVoltage() && isStringPotConnected() && areStringPotLimitsSetup();
  }

  private boolean stringPotHasStartingVoltage() {
    return stringPotStartingVoltage != null;
  }

  private double getStringPotVoltage() {
    return stringPot.getVoltage();
  }

  private boolean isStringPotConnected() {
    return getStringPotVoltage() > 0;
  }

  private boolean areStringPotLimitsSetup() {
    return getStringPotLowerLimit() != null && getStringPotUpperLimit() != null;
  }

  private Double getStringPotLowerLimit() {
    if (!isStringPotConnected()) {
      return null;
    }
    // If our starting voltage is below our lower limit, our lower
    // limit is going to be our starting voltage.
    if (stringPotHasStartingVoltage() && stringPotStartingVoltage < STRING_POT_LOWER_SOFT_LIMIT) {
      return stringPotStartingVoltage;
    }
    return STRING_POT_LOWER_SOFT_LIMIT;
  }

  private Double getStringPotUpperLimit() {
    if (!isStringPotConnected()) {
      return null;
    }
    // If our starting voltage is above our upper limit, our upper
    // limit is going to be our starting voltage.
    if (stringPotHasStartingVoltage() && stringPotStartingVoltage > STRING_POT_UPPER_SOFT_LIMIT) {
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
